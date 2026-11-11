from __future__ import annotations

import asyncio
import base64
import os
import socket
import struct
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import bridge  # noqa: E402


def masked_frame(opcode: int, payload: bytes) -> bytes:
    header = bytearray([0x80 | opcode])
    length = len(payload)
    mask = os.urandom(4)
    if length < 126:
        header.append(0x80 | length)
    elif length <= 0xFFFF:
        header.append(0x80 | 126)
        header.extend(struct.pack("!H", length))
    else:
        header.append(0x80 | 127)
        header.extend(struct.pack("!Q", length))
    masked = bytes(byte ^ mask[index & 3] for index, byte in enumerate(payload))
    return bytes(header) + mask + masked


async def read_ws_frame(reader: asyncio.StreamReader):
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
    mask = await reader.readexactly(4) if masked else b""
    payload = await reader.readexactly(length) if length else b""
    if masked:
        payload = bytes(byte ^ mask[index & 3] for index, byte in enumerate(payload))
    return fin, opcode, payload


class TestWebSocketClient:
    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        self.reader = reader
        self.writer = writer

    @classmethod
    async def connect(cls, host: str, port: int) -> "TestWebSocketClient":
        reader, writer = await asyncio.open_connection(host, port)
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        request = (
            "GET /ws HTTP/1.1\r\n"
            f"Host: {host}:{port}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n"
            "\r\n"
        )
        writer.write(request.encode("ascii"))
        await writer.drain()
        response = await asyncio.wait_for(reader.readuntil(b"\r\n\r\n"), 2.0)
        if b"101 Switching Protocols" not in response:
            raise AssertionError(f"WebSocket handshake failed: {response!r}")
        return cls(reader, writer)

    async def send_binary(self, payload: bytes) -> None:
        self.writer.write(masked_frame(0x2, payload))
        await self.writer.drain()

    async def recv_message(self, timeout: float = 2.0):
        async def _read():
            while True:
                _fin, opcode, payload = await read_ws_frame(self.reader)
                if opcode == 0x8:
                    return None
                if opcode == 0x9:
                    self.writer.write(masked_frame(0xA, payload))
                    await self.writer.drain()
                    continue
                if opcode == 0xA:
                    continue
                return opcode, payload

        return await asyncio.wait_for(_read(), timeout)

    async def close(self) -> None:
        try:
            self.writer.write(masked_frame(0x8, b""))
            await self.writer.drain()
        except (ConnectionError, OSError):
            pass
        self.writer.close()
        try:
            await self.writer.wait_closed()
        except (ConnectionError, OSError):
            pass


class BridgeTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.web_root = Path(self.temp_dir.name)
        self.index_bytes = b"<!doctype html><title>MPC</title>"
        (self.web_root / "index.html").write_bytes(self.index_bytes)
        (self.web_root / "test.wasm").write_bytes(b"\x00asm\x01\x00\x00\x00")

        self.hub = bridge.Hub(self.web_root, isolation=True, verbose=False)
        self.http_server = await asyncio.start_server(
            lambda r, w: bridge.handle_connection(self.hub, r, w),
            "127.0.0.1",
            0,
        )
        self.tcp_server = await asyncio.start_server(
            lambda r, w: bridge.handle_controller(self.hub, r, w),
            "127.0.0.1",
            0,
        )
        self.http_port = self.http_server.sockets[0].getsockname()[1]
        self.tcp_port = self.tcp_server.sockets[0].getsockname()[1]
        self._closers = []

    async def asyncTearDown(self) -> None:
        for closer in self._closers:
            try:
                await closer()
            except Exception:
                pass
        for server in (self.http_server, self.tcp_server):
            server.close()
            try:
                await server.wait_closed()
            except Exception:
                pass
        await asyncio.sleep(0.05)
        self.temp_dir.cleanup()

    async def wait_for(self, predicate, timeout: float = 2.0) -> None:
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        while not predicate():
            if loop.time() > deadline:
                raise AssertionError("condition was not met before the timeout")
            await asyncio.sleep(0.01)

    async def connect_browser(self) -> TestWebSocketClient:
        client = await TestWebSocketClient.connect("127.0.0.1", self.http_port)
        self._closers.append(client.close)
        await self.wait_for(lambda: len(self.hub.browsers) == 1)
        return client

    async def test_tcp_control_reaches_browser(self) -> None:
        browser = await self.connect_browser()
        initial = await browser.recv_message()
        self.assertIsNotNone(initial)
        self.assertEqual(initial[0], 0x2)
        self.assertEqual(len(initial[1]), bridge.CONTROL_SIZE)

        reader, writer = await asyncio.open_connection("127.0.0.1", self.tcp_port)

        async def close_controller():
            writer.close()
            await writer.wait_closed()

        self._closers.append(close_controller)
        await self.wait_for(lambda: len(self.hub.controllers) == 1)

        control = struct.pack("<dd", 0.25, 12.5)
        writer.write(control)
        await writer.drain()

        message = await browser.recv_message()
        self.assertIsNotNone(message)
        self.assertEqual(message[0], 0x2)
        self.assertEqual(message[1], control)

    async def test_browser_sensor_reaches_tcp_controller(self) -> None:
        reader, writer = await asyncio.open_connection("127.0.0.1", self.tcp_port)

        async def close_controller():
            writer.close()
            await writer.wait_closed()

        self._closers.append(close_controller)
        await self.wait_for(lambda: len(self.hub.controllers) == 1)

        browser = await self.connect_browser()
        await browser.recv_message()  # initial control packet

        sensor = struct.pack("<698d", *[float(value) for value in range(698)])
        await browser.send_binary(sensor)

        received = await asyncio.wait_for(reader.readexactly(bridge.SENSOR_SIZE), 2.0)
        self.assertEqual(received, sensor)

    async def test_legacy_udp_round_trip(self) -> None:
        loop = asyncio.get_running_loop()
        udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_client.bind(("127.0.0.1", 0))
        udp_client.setblocking(False)

        def close_udp_client():
            udp_client.close()

        self._closers.append(close_udp_client)
        udp_client_port = udp_client.getsockname()[1]

        transport, protocol = await loop.create_datagram_endpoint(
            lambda: bridge.UdpRelay(self.hub, ("127.0.0.1", udp_client_port)),
            local_addr=("127.0.0.1", 0),
        )
        self.hub.udp = protocol
        relay_port = transport.get_extra_info("sockname")[1]

        async def close_udp_transport():
            transport.close()

        self._closers.append(close_udp_transport)

        browser = await self.connect_browser()
        await browser.recv_message()  # initial control packet

        control = struct.pack("<dd", -0.5, 9.0)
        await loop.sock_sendto(udp_client, control, ("127.0.0.1", relay_port))
        message = await browser.recv_message()
        self.assertIsNotNone(message)
        self.assertEqual(message[1], control)

        sensor = struct.pack("<698d", *[0.0] * 698)
        await browser.send_binary(sensor)
        received, _addr = await asyncio.wait_for(loop.sock_recvfrom(udp_client, 65536), 2.0)
        self.assertEqual(received, sensor)

    async def test_http_serves_build_with_isolation_headers(self) -> None:
        reader, writer = await asyncio.open_connection("127.0.0.1", self.http_port)
        writer.write(b"GET / HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n")
        await writer.drain()
        raw = await asyncio.wait_for(reader.read(), 2.0)
        writer.close()
        await writer.wait_closed()

        head, _, body = raw.partition(b"\r\n\r\n")
        self.assertIn(b"200 OK", head)
        self.assertIn(b"Content-Type: text/html", head)
        self.assertIn(b"Cross-Origin-Opener-Policy: same-origin", head)
        self.assertIn(b"Cross-Origin-Embedder-Policy: require-corp", head)
        self.assertEqual(body, self.index_bytes)

    async def test_wasm_mime_type(self) -> None:
        reader, writer = await asyncio.open_connection("127.0.0.1", self.http_port)
        writer.write(b"GET /test.wasm HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n")
        await writer.drain()
        raw = await asyncio.wait_for(reader.read(), 2.0)
        writer.close()
        await writer.wait_closed()

        head, _, body = raw.partition(b"\r\n\r\n")
        self.assertIn(b"200 OK", head)
        self.assertIn(b"Content-Type: application/wasm", head)
        self.assertEqual(body, b"\x00asm\x01\x00\x00\x00")

    async def test_missing_build_returns_helpful_404(self) -> None:
        (self.web_root / "index.html").unlink()
        reader, writer = await asyncio.open_connection("127.0.0.1", self.http_port)
        writer.write(b"GET / HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n")
        await writer.drain()
        raw = await asyncio.wait_for(reader.read(), 2.0)
        writer.close()
        await writer.wait_closed()

        self.assertIn(b"404 Not Found", raw)
        self.assertIn(b"godot --headless", raw)


if __name__ == "__main__":
    unittest.main()
