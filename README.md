# shef_mpcgodot — browser-only MPC simulation

Godot lane-following simulation with the NMPC controller ported to JavaScript and
running **inside the same browser page** as the Godot Web build.

This version is designed for **GitHub Pages**:

- Godot is exported to WebAssembly/WebGL.
- The NMPC controller is plain JavaScript (`web/mpc_controller.js`).
- `udp.gd` talks to it through `JavaScriptBridge`.
- There is **no server, no WebSocket, no TCP bridge, and no external service**.

## Architecture

```
GitHub Pages (static HTTPS)
┌──────────────────────────────────────────────────────────────┐
│  index.html (custom Godot shell)                             │
│                                                              │
│  ┌──────────────────────┐        ┌─────────────────────────┐ │
│  │ Godot Web (WASM)     │        │ mpc_controller.js       │ │
│  │  im_ulaser.gd        │  JS    │  13 state + 5 CTE       │ │
│  │  udp.gd ─────────────┼───────>│  NMPC + lane snap       │ │
│  │       <──────────────┼────────│  steer + speed          │ │
│  └──────────────────────┘        └─────────────────────────┘ │
│                                                              │
│  No network requests after the static files load.            │
└──────────────────────────────────────────────────────────────┘
```

The JavaScript port matches the MATLAB controller:

- 5-step unicycle prediction
- same CTE / steering / steering-smoothness / lane-edge weights
- same lane-snap rule
- same ±1.5 rad/s steering limits

## Deploy to GitHub Pages

1. Push the repository to GitHub.
2. In **Settings → Pages → Build and deployment**, set **Source** to
   **GitHub Actions**.
3. Push to `main` (or run the **Deploy browser MPC to GitHub Pages** workflow
   manually).

The workflow in `.github/workflows/pages.yml`:

- exports the project with Godot 4.6 in a container;
- copies `web/mpc_controller.js` next to the exported `index.html`;
- publishes `build/web` as the Pages artifact.

The site will be available at:

```
https://altcar.github.io/shef_mpcgodot/
```

(GitHub Pages project URL for `altcar/shef_mpcgodot`.)

Everything is served from that URL: `index.html`, `index.js`, `index.wasm`,
`index.pck`, and `mpc_controller.js`.

## Local preview (no bridge)

```bash
godot --headless --path . --export-release "Web" build/web/index.html
cp web/mpc_controller.js build/web/
python -m http.server 8080 --directory build/web
```

Open <http://localhost:8080/>.

On Windows, use `Copy-Item web\mpc_controller.js build\web\` instead of `cp`.

The page starts the car at 15 m/s by default in browser-controller mode.
Override it with:

```
http://localhost:8080/?speed=20
```

## Optional TCP / MATLAB mode

The original TCP bridge and MATLAB/Python controllers are still in the
repository for local lab use, but they are **not deployed to GitHub Pages**.
See [README_WEB_TCP.md](README_WEB_TCP.md).

To force the browser back onto the WebSocket bridge instead of the in-page JS
controller, open the page with:

```
http://localhost:8080/?ws=ws://127.0.0.1:8080/ws
```

Without `?ws=`, the browser uses the JavaScript controller and needs no server.

## Files

| File | Purpose |
| --- | --- |
| `udp.gd` | UDP on native, JS controller on Web, optional WebSocket bridge |
| `web/mpc_controller.js` | Browser NMPC controller (MATLAB port) |
| `web/shell.html` | Custom Godot Web shell; loads the controller and the game |
| `export_presets.cfg` | Web export preset with the custom shell |
| `.github/workflows/pages.yml` | Export + deploy to GitHub Pages |
| `im_ulaser.gd` | Sends sensor frames to the transport |
| `road_demos.gd` | Applies steering/speed to the car |
| `tools/bridge.py` | Optional local HTTP/WebSocket/TCP/UDP bridge |
| `tools/mpc_controller.py` | Optional Python TCP controller |
| `matlab/mpc_tcp.m` | Optional MATLAB TCP controller |
| `untitled.m` | Original MATLAB UDP controller (unchanged) |

## Verification

- Godot 4.6 headless Web export succeeds.
- Headless Chrome + WebGL2 test: Godot loads, `MPCController` receives sensor
  frames, and the car moves with no bridge running.
- The same build works when served from a subpath (`/web/`), matching a GitHub
  Pages project URL.
- 13 Python bridge/controller tests pass (`python -m unittest discover -s tests`).
- Native UDP behaviour is unchanged.

The project still emits two pre-existing, unrelated warnings during export:
`demo_menu.tscn` references a missing `icon.png`, and the scaled
`StaticBody3D` in `mpccar.tscn` triggers a Jolt scale warning.
