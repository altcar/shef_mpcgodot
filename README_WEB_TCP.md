# Running the MPC simulation in a browser (optional TCP controller)

> **Optional local/lab mode.** The GitHub Pages deployment uses the in-browser
> JavaScript controller described in [README.md](README.md); it does not need
> this bridge. Use this document when you want the simulation in a browser but
> the controller running outside the page in Python or MATLAB.

This adds a browser transport to the Godot MPC demo **without changing the
existing packet API**.

- Native/desktop Godot keeps using UDP exactly as before.
- In browser + bridge mode, the Web export speaks WebSocket to a small bridge.
- The bridge exposes a **TCP server** for the MPC controller.
- The original 16-byte control packet and 5584-byte sensor packet are reused
  byte-for-byte. No field, offset, or scale is changed.
- The original MATLAB `untitled.m` still works unchanged through the bridge's
  optional UDP relay.

## Architecture

```
  MATLAB mpc_tcp.m / tools/mpc_controller.py
                    |
                    | TCP :4248
                    |   16 B control  ->
                    |   <- 5584 B sensor
                    v
          +-------------------+
          |  tools/bridge.py  |
          |                   |
          |  TCP server       |
          |  WebSocket server |
          |  HTTP file server |
          |  legacy UDP relay |
          +-------------------+
             |            |
   WebSocket |            | UDP :4246 / :4247
   /ws       |            | (only for the old MATLAB script)
             v            v
   Browser: Godot Web    MATLAB untitled.m
   (HTML5 / WASM)
```

Why a bridge? Browsers cannot open raw UDP or TCP sockets. Godot Web supports
`WebSocketPeer`, so the browser side is WebSocket and the controller side is
ordinary TCP.

## Quick start

### 1. Install Godot 4.6 and the Web export templates

In the Godot editor: **Editor > Manage Export Templates > Download and Install**.
The project is configured for the **GL Compatibility** renderer, which is what
the Web platform requires.

### 2. Export the Web build

From the project root:

```bash
godot --headless --path . --export-release "Web" build/web/index.html
cp web/mpc_controller.js build/web/
```

Or use **Project > Export > Web > Export Project** and choose
`build/web/index.html`, then copy `web/mpc_controller.js` next to it.

`export_presets.cfg` is committed with a single-threaded Web preset. That works
on any static host and avoids requiring `SharedArrayBuffer` headers. The bridge
still sends the COOP/COEP headers, so you can switch to a threaded export later
without changing the bridge.

`build/web/` is generated output and is ignored by git. Re-export after changing
the project.

### 3. Start the bridge

```bash
python tools/bridge.py --web-root build/web
```

Defaults:

| Endpoint | Default |
| --- | --- |
| Browser UI | `http://localhost:8080/` |
| Browser WebSocket | `ws://localhost:8080/ws` |
| Controller TCP | `127.0.0.1:4248` |
| Legacy MATLAB UDP control in | `127.0.0.1:4246` |
| Legacy MATLAB UDP sensor out | `127.0.0.1:4247` |

The HTTP server binds `0.0.0.0`, so another machine on the LAN can open
`http://<bridge-ip>:8080/`. The TCP controller binds `127.0.0.1` by default;
use `--tcp-host 0.0.0.0` if the controller runs on another machine.

Open the page with the WebSocket override, otherwise the browser uses the
in-page JavaScript controller and the bridge sees no sensor frames:

```
http://localhost:8080/?ws=ws://localhost:8080/ws
```

The URL can also be set on the `udp` node's `server_url` property in the
inspector.

### 4. Start the controller

Python (no NumPy/SciPy required):

```bash
python tools/mpc_controller.py --host 127.0.0.1 --port 4248 --speed 15
```

MATLAB:

```matlab
cd matlab
mpc_tcp
```

The car starts moving as soon as the controller connects and the browser is
sending sensor frames. Both controllers implement the same NMPC cost and
weights as the original MATLAB script. The Python controller uses SciPy's
SLSQP when SciPy is installed; otherwise it uses a built-in projected-gradient
solver. `--plot` shows the same four panels as the MATLAB monitor.

### 5. Optional: keep using the original UDP MATLAB controller

Start the bridge normally (the legacy UDP relay is on by default), open the
browser page with the WebSocket override (so the browser sends sensor frames to
the bridge), then run the original `untitled.m` unchanged. It sends to UDP
`4246` and listens on `4247`, which the bridge relays to/from the browser.

If port `4246` is already used by a native Godot instance, start the bridge with
`--no-udp` and use the TCP controller instead.

## Packet API (unchanged)

All values are little-endian IEEE-754 doubles.

### Controller -> simulation: 16 bytes

| Offset | Type | Field |
| --- | --- | --- |
| 0 | double | `steer` (yaw rate, rad/s) |
| 8 | double | `speed` (m/s) |

### Simulation -> controller: 5584 bytes (698 doubles)

| Offset (bytes) | Count | Field |
| --- | --- | --- |
| 0 | 13 | state: `pos.x, pos.y, pos.z, quat.w, quat.x, quat.y, quat.z, lin_vel.x, lin_vel.y, lin_vel.z, ang_vel.x, ang_vel.y, ang_vel.z` |
| 104 | 360 | LiDAR distances, 1 degree per sample |
| 2984 | 320 | lane intensity, 5 horizons x 64 bins, horizon-major |
| 5544 | 5 | cross-track error at 0, 2.5, 5.0, 7.5, 10.0 m |

TCP is a byte stream, so the bridge uses **fixed-size framing**, not a length
prefix:

- read exactly 16 bytes from the controller per command;
- write exactly 5584 bytes to the controller per sensor frame.

This keeps the bytes on the wire identical to the old UDP packets. The bridge
keeps only the newest packet in each direction, so a slow controller catches up
instead of building a backlog.

WebSocket messages use the same payloads: one binary message is one packet.

## How the Godot side works

`udp.gd` is still the same node, with the same public members used by the rest
of the project:

```gdscript
var steering_input: float
var speed_input: float
func send_data(buffer: PackedByteArray) -> void
# group: "network_transceivers"
```

Internally:

- `OS.has_feature("web") == false` -> the original `PacketPeerUDP` on 4246/4247.
- `OS.has_feature("web") == true` -> the in-page `MPCController` JavaScript
  object by default (no network), or `WebSocketPeer` to
  `ws(s)://<page host>/ws` when `?ws=`/`server_url` is supplied.

The WebSocket URL can be forced in two ways:

1. Query string: `http://localhost:8080/?ws=ws://192.168.1.20:8080/ws`
2. The `server_url` export on the `udp` node.

Reconnects are automatic with a 0.5 s to 5 s backoff.

`im_ulaser.gd` now finds the transport by looking for a node with a
`send_data()` method instead of calling `send_data` on every node in the group.
This removes the "nonexistent function" errors caused by the IMU nodes sharing
the group.

## HTTPS / WSS

If the page is served over HTTPS, the browser will refuse a `ws://` connection
because it is mixed content. Either:

- serve the bridge with TLS:

  ```bash
  python tools/bridge.py --web-root build/web \
      --tls-cert cert.pem --tls-key key.pem
  ```

- or put the bridge behind a TLS reverse proxy (Caddy, nginx, Cloudflare
  Tunnel, etc.) and pass the WebSocket upgrade for `/ws`.

Then open the page with the WebSocket override, for example
`https://.../?ws=wss://host/ws`. The override is required because the page
otherwise uses the in-page JavaScript controller, which does not need a bridge.

The bridge has no authentication. It is intended for a trusted local network or
a lab machine. If you expose it publicly, put it behind a reverse proxy with
authentication and TLS; do not publish the controller TCP port.

## Troubleshooting

**The page loads but the car does not move**

- Open the page with `?ws=...` when using the TCP bridge; otherwise the page is
  using the in-page JavaScript controller and the bridge sees no sensors.
- Check the bridge output: it prints browser and controller connection counts.
- Make sure the controller connected to TCP `4248`, not UDP.
- The browser must be the foreground tab sending sensor frames; a suspended tab
  stops sending and the controller will time out.
- Use only one browser tab. Multiple tabs all send sensor frames to the same
  controller.

**`WebSocket connection failed` / `connection closed`**

- Start `tools/bridge.py` before opening the page.
- If the page is HTTPS, use WSS (see above).
- Check the URL: the default is the same host and port as the page, path `/ws`.
- Windows may ask for firewall permission the first time the bridge binds
  `0.0.0.0:8080`.

**`Failed to load ... .wasm`**

Serve the build through the bridge (or any server that maps `.wasm` to
`application/wasm`). Opening `index.html` from the filesystem will not work.

**`Legacy UDP relay disabled`**

Port `4246` is already bound, usually by a native Godot instance or another
copy of the bridge. Stop the other process or use `--no-udp`.

**Threaded Web export fails to start**

A threaded export needs cross-origin isolation. The bridge sends the required
headers by default. If you host the files elsewhere, either enable
**Progressive Web App > Ensure Cross Origin Isolation Headers** in the export
preset, or configure `Cross-Origin-Opener-Policy: same-origin` and
`Cross-Origin-Embedder-Policy: require-corp` on the static host. With the
committed single-threaded preset this is not required; pass `--no-isolation` to
the bridge if you do not want the headers at all.

**MATLAB `read` times out**

The controller only receives sensor frames while the browser is running. Increase
the `tcpclient` `Timeout`, or start the browser before the MATLAB script.

**Export warnings that are safe to ignore**

The current project already contains two unrelated warnings: `demo_menu.tscn`
references `res://icon.png` while the repository ships `icon.svg`, and the scaled
`StaticBody3D` in `mpccar.tscn` is not fully supported by Jolt's shape scaling.
Neither warning affects the main MPC scene, and both are present before these
changes.

## Files added/changed

| File | Purpose |
| --- | --- |
| `udp.gd` | UDP on native, WebSocket on Web; public API unchanged |
| `im_ulaser.gd` | Finds the transport explicitly; sends the same buffer |
| `road_demos.gd` | Finds the transport explicitly |
| `export_presets.cfg` | Web export preset (`build/web/index.html`) |
| `tools/bridge.py` | HTTP + WebSocket + TCP + legacy UDP bridge |
| `tools/mpc_controller.py` | TCP NMPC controller, packet self-check, optional plot |
| `matlab/mpc_tcp.m` | MATLAB TCP version of the original controller |
| `tests/test_bridge.py` | WebSocket/TCP/UDP/HTTP relay tests |
| `tests/test_controller.py` | Packet layout and NMPC tests |
| `untitled.m` | Unchanged original UDP controller |

## Running the tests

```bash
python -m unittest discover -s tests -v
python tools/mpc_controller.py --check
```
