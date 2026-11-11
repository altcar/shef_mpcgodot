#!/usr/bin/env python3
"""Godot MPC bridge: browser WebSocket <-> controller TCP (and legacy UDP).

The Godot web export cannot open raw UDP/TCP sockets. This single-file,
dependency-free bridge does three jobs in one asyncio process:

  * serves the exported Web build over HTTP (with the optional COOP/COEP
    headers a threaded Godot build needs);
  * hosts the browser WebSocket endpoint (`/ws`);
  * hosts the TCP server used by the MPC controller.

Packet API is unchanged. One message is always one raw packet:

  controller -> simulation   16 bytes   little-endian double steer, double speed
  simulation -> controller  5584 bytes  little-endian 698 doubles

TCP is a byte stream, so the bridge uses *fixed-size* messages: read exactly
16 bytes from the controller, write exactly 5584 bytes to it. There is no
length prefix, which keeps the payload byte-for-byte identical to the old UDP
packets.

The optional UDP relay lets the original MATLAB `untitled.m` keep working
unchanged while the simulation runs in the browser:

  MATLAB -> bridge UDP :4246  (16-byte control)
  bridge -> MATLAB UDP :4247  (5584-byte sensor)

Usage:
    python tools/bridge.py --web-root build/web
    python tools/bridge.py --web-root build/web --verbose
    python tools/bridge.py --tcp-host 0.0.0.0 --tcp-port 4248
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import hashlib
import math
import mimetypes
import posixpath
import ssl
import struct
import sys
import time
import urllib.parse
from pathlib import Path
from typing import Optional

CONTROL_SIZE = 16
SENSOR_SIZE = 5584

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
MAX_MESSAGE = 1 << 20
HEADER_LIMIT = 64 * 1024

MIME_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".htm": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".mjs": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".wasm": "application/wasm",
    ".pck": "application/octet-stream",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
    ".webmanifest": "application/manifest+json",
    ".map": "application/json",
    ".txt": "text/plain; charset=utf-8",
}


def log(message: str) -> None:
    print(f"[bridge] {message}", flush=True)


def offer_latest(queue: asyncio.Queue, item: Optional[bytes]) -> None:
    """Put an item into a size-1 queue, discarding the stale item if full."""
    try:
        queue.put_nowait(item)
        return
    except asyncio.QueueFull:
        pass
    try:
        queue.get_nowait()
    except asyncio.QueueEmpty:
        pass
    try:
        queue.put_nowait(item)
    except asyncio.QueueFull:
        pass


class Hub:
    """Owns the current browser/controller sets and moves packets between them."""

    def __init__(self, web_root: Path, isolation: bool, verbose: bool = False) -> None:
        self.web_root = web_root
        self.isolation = isolation
        self.verbose = verbose

        self.browsers: set["BrowserClient"] = set()
        self.controllers: set["ControllerClient"] = set()
        self.udp: Optional[UdpRelay] = None

        self.latest_control = struct.pack("<dd", 0.0, 0.0)
        self.latest_sensor: Optional[bytes] = None

        self.control_packets = 0
        self.sensor_packets = 0
        self.rejected_control_packets = 0

    # -- controller -> browser -------------------------------------------------
    def on_control(self, packet: bytes, source: str) -> None:
        if len(packet) != CONTROL_SIZE:
            self.rejected_control_packets += 1
            return
        try:
            steer, speed = struct.unpack("<dd", packet)
        except struct.error:
            self.rejected_control_packets += 1
            return
        if not (math.isfinite(steer) and math.isfinite(speed)):
            self.rejected_control_packets += 1
            return
        # Sanity bounds protect the stream if a controller gets out of sync.
        if abs(steer) > 10.0 or abs(speed) > 200.0:
            self.rejected_control_packets += 1
            if self.verbose:
                log(f"rejected out-of-range control from {source}: steer={steer} speed={speed}")
            return

        self.latest_control = packet
        self.control_packets += 1
        if self.verbose and self.control_packets % 200 == 0:
            log(f"control from {source}: steer={steer:+.3f} speed={speed:.3f}")
        for browser in tuple(self.browsers):
            browser.push_control(packet)

    # -- browser -> controller -------------------------------------------------
    def on_sensor(self, packet: bytes, source: str = "browser") -> None:
        if len(packet) != SENSOR_SIZE:
            return
        self.latest_sensor = packet
        self.sensor_packets += 1
        for controller in tuple(self.controllers):
            controller.push_sensor(packet)
        if self.udp is not None:
            self.udp.send_sensor(packet)

    # -- membership ------------------------------------------------------------
    def add_browser(self, browser: "BrowserClient") -> None:
        self.browsers.add(browser)
        # Start a fresh page with the latest command instead of zero steering.
        browser.push_control(self.latest_control)

    def remove_browser(self, browser: "BrowserClient") -> None:
        self.browsers.discard(browser)

    def add_controller(self, controller: "ControllerClient") -> None:
        self.controllers.add(controller)
        if self.latest_sensor is not None:
            controller.push_sensor(self.latest_sensor)

    def remove_controller(self, controller: "ControllerClient") -> None:
        self.controllers.discard(controller)


class BrowserClient:
    """One browser WebSocket connection."""

    def __init__(self, hub: Hub, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        self.hub = hub
        self.reader = reader
        self.writer = writer
        self.peer = writer.get_extra_info("peername")
        self.closed = False
        self._control_queue: asyncio.Queue = asyncio.Queue(maxsize=1)
        self._write_lock = asyncio.Lock()
        self._writer_task = asyncio.create_task(self._writer_loop())

    def push_control(self, packet: bytes) -> None:
        if not self.closed:
            offer_latest(self._control_queue, packet)

    async def _writer_loop(self) -> None:
        try:
            while True:
                packet = await self._control_queue.get()
                if packet is None:
                    return
                await self.send_frame(0x2, packet)
        except (asyncio.CancelledError, ConnectionError, OSError):
            return

    async def send_frame(self, opcode: int, payload: bytes) -> None:
        if self.closed:
            return
        async with self._write_lock:
            if self.closed:
                return
            self.writer.write(ws_frame(opcode, payload))
            await self.writer.drain()

    async def run(self) -> None:
        fragments = bytearray()
        message_opcode: Optional[int] = None
        try:
            while True:
                fin, opcode, payload = await read_frame(self.reader)

                if opcode == 0x8:  # close
                    try:
                        await self.send_frame(0x8, payload[:125])
                    except (ConnectionError, OSError):
                        pass
                    return
                if opcode == 0x9:  # ping
                    await self.send_frame(0xA, payload)
                    continue
                if opcode == 0xA:  # pong
                    continue

                if opcode == 0x0:  # continuation
                    if message_opcode is None:
                        return
                    fragments.extend(payload)
                elif opcode in (0x1, 0x2):  # text / binary
                    message_opcode = opcode
                    fragments = bytearray(payload)
                else:
                    return

                if len(fragments) > MAX_MESSAGE:
                    return

                if not fin:
                    continue

                if message_opcode == 0x2:
                    self.hub.on_sensor(bytes(fragments))
                elif message_opcode == 0x1 and self.hub.verbose:
                    text = bytes(fragments).decode("utf-8", "replace")
                    log(f"browser text message ignored: {text[:120]!r}")

                fragments = bytearray()
                message_opcode = None
        except (asyncio.IncompleteReadError, ConnectionError, OSError):
            return
        finally:
            self.close()

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        offer_latest(self._control_queue, None)
        try:
            self.writer.close()
        except Exception:
            pass


class ControllerClient:
    """One TCP controller connection."""

    def __init__(self, hub: Hub, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        self.hub = hub
        self.reader = reader
        self.writer = writer
        self.peer = writer.get_extra_info("peername")
        self.closed = False
        self._sensor_queue: asyncio.Queue = asyncio.Queue(maxsize=1)
        self._writer_task = asyncio.create_task(self._writer_loop())

    def push_sensor(self, packet: bytes) -> None:
        if not self.closed:
            offer_latest(self._sensor_queue, packet)

    async def _writer_loop(self) -> None:
        try:
            while True:
                packet = await self._sensor_queue.get()
                if packet is None:
                    return
                self.writer.write(packet)
                await self.writer.drain()
        except (asyncio.CancelledError, ConnectionError, OSError):
            return

    async def run(self) -> None:
        try:
            while True:
                packet = await self.reader.readexactly(CONTROL_SIZE)
                self.hub.on_control(packet, "tcp")
        except (asyncio.IncompleteReadError, ConnectionError, OSError):
            return
        finally:
            self.close()

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        offer_latest(self._sensor_queue, None)
        try:
            self.writer.close()
        except Exception:
            pass


class UdpRelay(asyncio.DatagramProtocol):
    """Legacy compatibility: the original MATLAB UDP controller."""

    def __init__(self, hub: Hub, destination: tuple[str, int]) -> None:
        self.hub = hub
        self.destination = destination
        self.transport: Optional[asyncio.DatagramTransport] = None
        # Do not send sensor packets until a UDP controller has spoken to us.
        # On Windows, sending to a port with no listener produces ICMP port
        # unreachable errors which can reset the datagram socket.
        self.active = False

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = transport  # type: ignore[assignment]

    def datagram_received(self, data: bytes, addr) -> None:
        if len(data) == CONTROL_SIZE:
            self.active = True
            self.hub.on_control(data, "udp")

    def error_received(self, exc: Exception) -> None:
        self.active = False
        if self.hub.verbose:
            log(f"legacy UDP error: {exc}")

    def send_sensor(self, packet: bytes) -> None:
        if not self.active or self.transport is None:
            return
        try:
            self.transport.sendto(packet, self.destination)
        except OSError:
            self.active = False


# ---------------------------------------------------------------------------
# WebSocket framing
# ---------------------------------------------------------------------------


async def read_frame(reader: asyncio.StreamReader) -> tuple[bool, int, bytes]:
    header = await reader.readexactly(2)
    first, second = header
    fin = bool(first & 0x80)
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    length = second & 0x7F

    if length == 126:
        length = struct.unpack("!H", await reader.readexactly(2))[0]
    elif length == 127:
        length = struct.unpack("!Q", await reader.readexactly(8))[0]

    if length > MAX_MESSAGE:
        raise ConnectionError(f"WebSocket frame too large: {length} bytes")
    if not masked:
        raise ConnectionError("client WebSocket frames must be masked")

    mask = await reader.readexactly(4)
    payload = await reader.readexactly(length) if length else b""
    if length:
        payload = bytes(byte ^ mask[index & 3] for index, byte in enumerate(payload))
    return fin, opcode, payload


def ws_frame(opcode: int, payload: bytes) -> bytes:
    header = bytearray([0x80 | opcode])
    length = len(payload)
    if length < 126:
        header.append(length)
    elif length <= 0xFFFF:
        header.append(126)
        header.extend(struct.pack("!H", length))
    else:
        header.append(127)
        header.extend(struct.pack("!Q", length))
    return bytes(header) + payload


# ---------------------------------------------------------------------------
# HTTP / WebSocket server
# ---------------------------------------------------------------------------


async def handle_connection(
    hub: Hub, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
) -> None:
    try:
        raw = await asyncio.wait_for(reader.readuntil(b"\r\n\r\n"), timeout=15.0)
    except (asyncio.IncompleteReadError, asyncio.LimitOverrunError, asyncio.TimeoutError, ConnectionError):
        close_writer(writer)
        return

    try:
        request_line = raw.split(b"\r\n", 1)[0].decode("latin-1")
        method, target, _version = request_line.split(" ", 2)
    except (UnicodeDecodeError, ValueError):
        await http_error(writer, 400, "Bad Request")
        return

    headers: dict[str, str] = {}
    for line in raw.decode("latin-1").split("\r\n")[1:]:
        if not line:
            continue
        key, _, value = line.partition(":")
        headers[key.strip().lower()] = value.strip()

    path = urllib.parse.urlsplit(target).path
    is_websocket = (
        headers.get("upgrade", "").lower() == "websocket"
        and path == "/ws"
    )

    if is_websocket:
        try:
            await handle_websocket(hub, reader, writer, headers)
        except (ConnectionError, OSError):
            pass
        finally:
            close_writer(writer)
        return

    try:
        await serve_static(hub, method, target, writer)
    except (ConnectionError, OSError):
        pass
    finally:
        close_writer(writer)


async def handle_websocket(
    hub: Hub,
    reader: asyncio.StreamReader,
    writer: asyncio.StreamWriter,
    headers: dict[str, str],
) -> None:
    key = headers.get("sec-websocket-key", "")
    version = headers.get("sec-websocket-version", "")
    if not key or version != "13":
        await http_error(writer, 400, "WebSocket handshake requires version 13")
        return

    accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode("ascii")).digest()).decode("ascii")
    response = (
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Accept: {accept}\r\n"
        "\r\n"
    )
    writer.write(response.encode("ascii"))
    await writer.drain()

    client = BrowserClient(hub, reader, writer)
    hub.add_browser(client)
    log(f"browser connected ({client.peer}); browsers={len(hub.browsers)}")
    try:
        await client.run()
    finally:
        hub.remove_browser(client)
        client.close()
        log(f"browser disconnected ({client.peer}); browsers={len(hub.browsers)}")
        close_writer(writer)


async def handle_controller(
    hub: Hub, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
) -> None:
    client = ControllerClient(hub, reader, writer)
    hub.add_controller(client)
    log(f"controller connected ({client.peer}); controllers={len(hub.controllers)}")
    try:
        await client.run()
    finally:
        hub.remove_controller(client)
        client.close()
        log(f"controller disconnected ({client.peer}); controllers={len(hub.controllers)}")
        close_writer(writer)


async def serve_static(
    hub: Hub,
    method: str,
    target: str,
    writer: asyncio.StreamWriter,
) -> None:
    if method.upper() not in ("GET", "HEAD"):
        await http_error(writer, 405, "Method Not Allowed")
        return

    request_path = urllib.parse.unquote(urllib.parse.urlsplit(target).path).replace("\\", "/")
    if request_path in ("", "/"):
        request_path = "/index.html"

    relative = posixpath.normpath(request_path).lstrip("/")
    if relative in ("", ".") or relative.startswith("../") or relative == "..":
        await http_error(writer, 403, "Forbidden")
        return

    root = hub.web_root.resolve()
    full_path = (root / relative).resolve()
    try:
        full_path.relative_to(root)
    except ValueError:
        await http_error(writer, 403, "Forbidden")
        return

    if full_path.is_dir():
        full_path = full_path / "index.html"

    if not full_path.is_file():
        if relative == "index.html":
            message = (
                "No Web build found in "
                f"{root}. Export it first:\n"
                '  godot --headless --path . --export-release "Web" build/web/index.html\n'
            )
        else:
            message = f"Not found: /{relative}\n"
        await http_error(writer, 404, message)
        return

    data = full_path.read_bytes()
    content_type = (
        MIME_TYPES.get(full_path.suffix.lower())
        or mimetypes.guess_type(str(full_path))[0]
        or "application/octet-stream"
    )

    headers = [
        "HTTP/1.1 200 OK",
        f"Content-Type: {content_type}",
        f"Content-Length: {len(data)}",
        "Cache-Control: no-store",
        "Connection: close",
        "X-Content-Type-Options: nosniff",
    ]
    if hub.isolation:
        headers.extend(
            [
                "Cross-Origin-Opener-Policy: same-origin",
                "Cross-Origin-Embedder-Policy: require-corp",
                "Cross-Origin-Resource-Policy: cross-origin",
            ]
        )
    writer.write(("\r\n".join(headers) + "\r\n\r\n").encode("latin-1"))
    if method.upper() != "HEAD":
        writer.write(data)
    await writer.drain()


async def http_error(writer: asyncio.StreamWriter, code: int, message: str) -> None:
    reasons = {
        400: "Bad Request",
        403: "Forbidden",
        404: "Not Found",
        405: "Method Not Allowed",
        500: "Internal Server Error",
    }
    body = message.encode("utf-8")
    headers = [
        f"HTTP/1.1 {code} {reasons.get(code, 'Error')}",
        "Content-Type: text/plain; charset=utf-8",
        f"Content-Length: {len(body)}",
        "Cache-Control: no-store",
        "Connection: close",
    ]
    try:
        writer.write(("\r\n".join(headers) + "\r\n\r\n").encode("latin-1") + body)
        await writer.drain()
    except (ConnectionError, OSError):
        pass
    finally:
        close_writer(writer)


def close_writer(writer: asyncio.StreamWriter) -> None:
    try:
        writer.close()
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Startup
# ---------------------------------------------------------------------------


async def stats_loop(hub: Hub, interval: float) -> None:
    last_control = 0
    last_sensor = 0
    last_time = time.monotonic()
    while True:
        await asyncio.sleep(interval)
        now = time.monotonic()
        elapsed = max(now - last_time, 1e-6)
        control_rate = (hub.control_packets - last_control) / elapsed
        sensor_rate = (hub.sensor_packets - last_sensor) / elapsed
        last_control = hub.control_packets
        last_sensor = hub.sensor_packets
        last_time = now
        log(
            f"browsers={len(hub.browsers)} controllers={len(hub.controllers)} "
            f"control={control_rate:.1f}/s sensor={sensor_rate:.1f}/s"
        )


async def run(args: argparse.Namespace) -> None:
    root = Path(args.web_root).resolve()
    hub = Hub(root, isolation=args.isolation, verbose=args.verbose)

    ssl_context: Optional[ssl.SSLContext] = None
    if args.tls_cert or args.tls_key:
        if not (args.tls_cert and args.tls_key):
            raise SystemExit("--tls-cert and --tls-key must be supplied together")
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_context.load_cert_chain(args.tls_cert, args.tls_key)

    http_server = await asyncio.start_server(
        lambda reader, writer: handle_connection(hub, reader, writer),
        host=args.host,
        port=args.http_port,
        ssl=ssl_context,
    )
    tcp_server = await asyncio.start_server(
        lambda reader, writer: handle_controller(hub, reader, writer),
        host=args.tcp_host,
        port=args.tcp_port,
    )

    udp_transport = None
    if args.udp:
        loop = asyncio.get_running_loop()
        try:
            udp_transport, protocol = await loop.create_datagram_endpoint(
                lambda: UdpRelay(hub, (args.udp_dest_host, args.udp_dest_port)),
                local_addr=(args.udp_host, args.udp_listen_port),
            )
            hub.udp = protocol  # type: ignore[assignment]
        except OSError as exc:
            log(f"warning: legacy UDP relay disabled ({exc})")

    scheme = "https" if ssl_context else "http"
    ws_scheme = "wss" if ssl_context else "ws"
    log(f"web root:            {root}")
    log(f"web UI:              {scheme}://localhost:{args.http_port}/")
    log(f"browser WebSocket:   {ws_scheme}://localhost:{args.http_port}/ws")
    log(f"TCP controller:      {args.tcp_host}:{args.tcp_port}")
    if args.udp and hub.udp is not None:
        log(
            "legacy UDP:         "
            f"listen {args.udp_host}:{args.udp_listen_port} -> "
            f"sensor dest {args.udp_dest_host}:{args.udp_dest_port}"
        )
    if not root.is_dir():
        log(f"warning: web root does not exist yet: {root}")
    log("ready")

    stats_task: Optional[asyncio.Task] = None
    if args.stats_interval > 0:
        stats_task = asyncio.create_task(stats_loop(hub, args.stats_interval))

    try:
        await asyncio.Event().wait()
    finally:
        if stats_task is not None:
            stats_task.cancel()
        for server in (http_server, tcp_server):
            server.close()
            try:
                await server.wait_closed()
            except Exception:
                pass
        if udp_transport is not None:
            udp_transport.close()
        for browser in tuple(hub.browsers):
            browser.close()
        for controller in tuple(hub.controllers):
            controller.close()


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Serve the Godot Web build and relay the MPC packet API over WebSocket/TCP.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--host", default="0.0.0.0", help="HTTP/WebSocket bind address")
    parser.add_argument("--http-port", type=int, default=8080, help="HTTP/WebSocket port")
    parser.add_argument("--web-root", default="build/web", help="directory containing the exported index.html")
    parser.add_argument("--tcp-host", default="127.0.0.1", help="TCP controller bind address")
    parser.add_argument("--tcp-port", type=int, default=4248, help="TCP controller port")
    parser.add_argument("--udp", dest="udp", action="store_true", help="enable the legacy MATLAB UDP relay")
    parser.add_argument("--no-udp", dest="udp", action="store_false", help="disable the legacy UDP relay")
    parser.set_defaults(udp=True)
    parser.add_argument("--udp-host", default="127.0.0.1", help="legacy UDP bind address")
    parser.add_argument("--udp-listen-port", type=int, default=4246, help="legacy UDP control listen port")
    parser.add_argument("--udp-dest-host", default="127.0.0.1", help="legacy UDP sensor destination host")
    parser.add_argument("--udp-dest-port", type=int, default=4247, help="legacy UDP sensor destination port")
    parser.add_argument(
        "--no-isolation",
        dest="isolation",
        action="store_false",
        help="do not send COOP/COEP headers (only needed for threaded Web exports)",
    )
    parser.set_defaults(isolation=True)
    parser.add_argument("--tls-cert", default="", help="TLS certificate file for HTTPS/WSS")
    parser.add_argument("--tls-key", default="", help="TLS private key file for HTTPS/WSS")
    parser.add_argument("--stats-interval", type=float, default=5.0, help="seconds between stats lines (0 disables)")
    parser.add_argument("--verbose", action="store_true", help="log packet samples and connection detail")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
