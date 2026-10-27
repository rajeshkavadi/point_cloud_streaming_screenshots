# Remote LiDAR Point-Cloud Snapshot Service

One-button, single-shot point-cloud screenshots for a remote browser user.
Click **Fetch Point Cloud** in Chrome on a *different network*; the host
captures exactly **one** current `PointCloud2` message, renders **five** PNG
views of that same message, and returns them to the browser.

This is **not** a streaming application: snapshots are produced only when the
user clicks the button. One opt-in exception: the **Live View**
(`GET /api/stream`) streams rendered JPEG frames continuously while a viewer
is connected (see §1 and §5).

---

## 1. Architecture

```
ROS 2 bag playback  (ros2 bag play, looping)
        |
        v
Registered/raw PointCloud2 topic   (e.g. /ouster/points)
        |
        v
ROS 2 Python subscriber           (pc_snapshot/ros_subscriber.py, background thread)
        |
        v
PointCloud2 decoder -> numpy      (pc_snapshot/decoder.py, ROS-free)
        |
        v
SnapshotStore: latest immutable snapshot   (pc_snapshot/store.py)
        |
        v  (POST /api/snapshot captures ONE immutable reference)
Open3D headless renderer (+ Matplotlib fallback)   (pc_snapshot/renderer.py)
        |
        v
Five PNG files (top, front, back, left, right) + metadata
        |
        v
FastAPI HTTP API                  (pc_snapshot/api.py)
        |
        v
Tailscale private network         (host <-> remote Windows browser)
        |
        v
Browser frontend (HTML/CSS/JS)    (static/)
```

The subscriber, decoder, renderer, and API are deliberately independent of
WSL so the exact same code can later run on an NVIDIA Jetson Orin (see §9).

### Key behaviours

- **One message per snapshot.** The subscriber decodes each message into a
  read-only numpy snapshot. `POST /api/snapshot` captures a reference to
  exactly one such snapshot and renders all five views from that single
  point set and one shared bounding box. The live subscriber buffer is never
  mutated or blocked by rendering.
- **Bounded work.** A non-blocking semaphore caps simultaneous render jobs
  (default 1); a second request while one is running returns HTTP 409.
  Point counts are capped (`max_render_points`) with deterministic stride
  downsampling. Rendered PNGs live in a temp registry with a max-entry cap
  and a TTL sweeper.
- **Honest staleness.** If the latest message is older than
  `stale_after_sec`, the response carries a `warnings[]` entry (or is
  rejected when `stale_policy = "reject"`). A stale message is never silently
  labelled "current".
- **Real errors.** Missing topic, no message yet, empty cloud, decode
  failure, and render failure each return a specific HTTP status + message.
- **Opt-in live view (added later by request).** `GET /api/stream` streams
  rendered frames of the incoming topic using `multipart/x-mixed-replace`
  (the same MJPEG mechanism Flask/OpenCV camera tutorials use — a plain
  `<img>` tag displays it). Rendering runs only while at least one viewer is
  connected, targets a bounded FPS (`stream_fps`, default 5) at a reduced
  resolution (`stream_width x stream_height`), and stops after
  `stream_idle_stop_sec` of no viewers. Snapshots remain one-shot on demand.

### Renderer threading (important implementation note)

This Open3D/Filament build has three hard constraints: a renderer may only
be **used on the thread that created it**, destroying one renderer while
another exists **panics the process**, and every new renderer re-pays a ~5 s
first-render warm-up. The service therefore keeps **exactly one**
OffscreenRenderer, owned by a single dedicated `o3d-render` thread; snapshot
and live-stream jobs are both submitted to that thread
(`renderer.submit_render`). It is destroyed once, on that thread, at
shutdown (`atexit` / `__main__`). Forced-backend tests
(`--renderer open3d` path) verify the Open3D path actually renders instead
of silently falling back to Matplotlib.

---

## 2. Environment used for development

Detected on the build host (WSL 2):

| Item | Value |
|------|-------|
| OS | Ubuntu 22.04.5 LTS on WSL 2 (kernel 6.18.54) |
| Python | 3.10.12 |
| ROS 2 | Humble (`/opt/ros/humble`) |
| `rclpy` / `rosbag2_py` / `sensor_msgs` | available |
| Open3D | 0.20.0 (headless via Vulkan / llvmpipe) |
| Matplotlib | 3.5.1 (fallback renderer) |
| FastAPI / Uvicorn | installed via `requirements.txt` |
| WSL network mode | **NAT** (eth0 = 172.30.35.79, not mirrored) |

### Dataset

A single SQLite bag file was provided:

```
/home/rajesh/point_cloud_streaming_screenshots/Outdoor_cut1_dataset_ros2.db3  (~3.3 GB)
```

`ros2 bag info` / direct SQLite inspection shows these topics:

| Topic | Type | Msgs |
|-------|------|------|
| `/mid360/livox/imu` | sensor_msgs/Imu | 9055 |
| `/gnss_pose` | geometry_msgs/PoseStamped | 4528 |
| `/ouster/imu` | sensor_msgs/Imu | 5659 |
| `/ouster/points` | **sensor_msgs/PointCloud2** | 453 |
| `/avia/livox/imu` | sensor_msgs/Imu | 9214 |
| `/avia/livox/lidar` | **sensor_msgs/PointCloud2** | 453 |
| `/mid360/livox/lidar` | **sensor_msgs/PointCloud2** | 453 |

**Important findings:**

- The bag contains **no registered point cloud** (no `/cloud_registered`).
  It holds **raw** LiDAR (`/ouster/points`, `/mid360/livox/lidar`,
  `/avia/livox/lidar`) plus IMU and GNSS pose.
- Therefore the default topic is the raw `/ouster/points`
  (131,072 points/message, frame `os_sensor`, 10 Hz). Rendering works
  out of the box on this raw cloud.
- To get a **motion-compensated, registered** cloud you must run FAST-LIO2
  on the raw LiDAR+IMU (see §9). A prebuilt FAST-LIO2 ROS 2 workspace exists
  at `/home/rajesh/workspace-tools/fastlio_ws/install`. When you point the
  service at a FAST-LIO2 output topic (e.g. `/cloud_registered`), no code
  changes are needed — just change `--topic`.
- The dataset file was **not** modified or deleted by this project.

---

## 3. Setup

```bash
cd /home/rajesh/point_cloud_streaming_screenshots/snapshot_service

# ROS 2 (Humble) must be sourced for the subscriber + bag playback.
source /opt/ros/humble/setup.bash

# Python deps (FastAPI, Uvicorn, Open3D, Matplotlib, Pillow, pytest, httpx)
pip3 install -r requirements.txt
```

---

## 4. Run (local development)

### 4.1 One command (service + looping bag)

```bash
source /opt/ros/humble/setup.bash
./scripts/start_all.sh /home/rajesh/point_cloud_streaming_screenshots/Outdoor_cut1_dataset_ros2.db3
```

This starts:
- the HTTP service on `http://127.0.0.1:8085` (topic `/ouster/points`), and
- `ros2 bag play <db3> --loop` so point clouds keep arriving.

Health check output is printed automatically. Stop everything with:

```bash
./scripts/stop_all.sh
```

### 4.2 Manual (two terminals)

Terminal 1 — bag playback:
```bash
source /opt/ros/humble/setup.bash
./scripts/run_bag.sh /home/rajesh/point_cloud_streaming_screenshots/Outdoor_cut1_dataset_ros2.db3
```

Terminal 2 — snapshot service:
```bash
source /opt/ros/humble/setup.bash
./scripts/run_service.sh /ouster/points 8085
```

Then open `http://127.0.0.1:8085/` in Chrome and click **Fetch Point Cloud**.

### 4.3 Test without a bag (synthetic publisher)

```bash
source /opt/ros/humble/setup.bash
python3 scripts/synthetic_publisher.py --topic /ouster/points --rate 5
# then run the service in another terminal
```

### 4.4 Useful options

Every option can be set via CLI flag, `PCSNAP_*` environment variable, or a
JSON config file (CLI > env > file > defaults).

```bash
python3 -m pc_snapshot \
  --topic /ouster/points \
  --port 8085 \
  --renderer auto \          # auto | open3d | matplotlib
  --up-axis z \              # z (ROS/robotics) or y (aerospace)
  --color-mode height \      # height | intensity
  --max-render-points 300000 \
  --clip-range 60 \          # radial clip in meters (0 = off)
  --stale-after 5.0 \
  --api-token mysecret \     # optional; require X-API-Token header
  --stream-fps 5 \           # live view target FPS (0.1 - 30)
  --stream-view front  # top | front | back | left | right
  # --no-stream              # disable GET /api/stream entirely
```

Copy `config.example.json` and pass it with `--config my.json` (or
`PCSNAP_CONFIG=my.json`) for file-based configuration.

The renderer backend is `auto`: it uses **Open3D** when the offscreen
renderer initializes (works in WSL 2 via Vulkan/llvmpipe), and automatically
falls back to **Matplotlib (Agg)** if Open3D is unavailable. You can force
either with `--renderer open3d` or `--renderer matplotlib`.

---

## 5. HTTP API

| Method | Path | Description |
|--------|------|-------------|
| GET | `/api/health` | Status, ROS connection, topic, message age, frame id, diagnostics |
| POST | `/api/snapshot` | Capture one message, render five views, return metadata + image URLs |
| GET | `/api/snapshot/{id}/{view}.png` | One PNG (`top`, `front`, `back`, `left`, `right`) |
| GET | `/api/snapshot/{id}/download.zip` | ZIP of all five PNGs + `metadata.json` |
| GET | `/api/stream?view=…` | **Live view**: endless JPEG frame stream (`multipart/x-mixed-replace`) of the current topic |
| GET | `/` | Browser frontend |

`POST /api/snapshot` returns JSON like:

```json
{
  "snapshot_id": "bfae64ca...",
  "capture_timestamp": 1792999557.6,
  "ros_message_stamp": 1747409937.88,
  "frame_id": "os_sensor",
  "topic": "/ouster/points",
  "original_point_count": 131072,
  "valid_point_count": 131072,
  "rendered_point_count": 131024,
  "render_seconds": 6.7,
  "views": ["top", "front", "back", "left", "right"],
  "images": { "top": ".../top.png", "front": "...", "back": "...", "left": "...", "right": "..." },
  "download_zip": ".../download.zip",
  "warnings": []
}
```

Status codes: `200` OK · `401` bad/missing token (when `api_token` set) ·
`404` unknown snapshot/view · `409` no message / stale (reject mode) / render
busy / empty cloud · `500` render failure.

**Auth:** set `api_token` (or `PCSNAP_API_TOKEN`) to require an
`X-API-Token` header (or `?token=`) on `POST /api/snapshot`. Leave it empty
for a private Tailscale-only network. This is intentionally simple; the real
protection is that the API is only reachable inside your tailnet.

**Live view** — `GET /api/stream?view=front&token=…`:

- Content type `multipart/x-mixed-replace; boundary=frame`; each part is one
  JPEG frame. Browsers render it natively in `<img src="…">` — no WebSocket,
  no video codec, no client-side player. This is exactly the mechanism used
  by Flask/OpenCV MJPEG camera-streaming tutorials, with a rendered point
  cloud as the frame source instead of a camera.
- `view` selects the camera angle (default `stream_view`, currently
  `front`). front/back/left/right are first-person: the camera sits at the
  sensor origin and looks outward, as if you were standing at the Livox
  Mid-360. The token must be passed as `?token=` here (an `<img>` tag
  cannot set headers).
- While waiting for the first frame or new data the endpoint re-sends a
  placeholder JPEG as a keep-alive, so the browser always shows status.
- Measured on this host through a Cloudflare quick tunnel: **~4.6 fps** at
  960×540, ~90 KB/frame (~3.6 Mbit/s), one-time ~5 s warm-up on the first
  render after service start, `401` without the token.

---

## 6. Browser frontend

`static/index.html` provides:

- Title **Remote LiDAR Snapshot Viewer** and a connection indicator.
- A configurable **API endpoint** (defaults to the page's own origin; saved
  in `localStorage`) and an optional API token field.
- A **Fetch Point Cloud** button (single POST; disabled while running).
- A **Live View** panel: **Start/Stop Live View** button, a camera-angle
  selector, an `<img>` bound to `/api/stream`, and a status readout
  (fps · points, refreshed from `/api/health`). Rendering starts only when
  you click Start and stops when you click Stop (or all viewers leave).
- A five-panel layout labelled Top / Front / Back / Left / Right —
  front/back/left/right are rendered as if standing at the sensor.
- Capture time, ROS message time, frame id, point counts, render time,
  warnings, and an error area.
- Per-image **Download PNG** links and a **Download All Images ZIP** button.

Snapshot images change only when you click **Fetch Point Cloud** — there is
no automatic snapshot refresh. The Live View stream, when started, is
continuous by design.

---

## 7. Cross-network access with Tailscale

The host (WSL) and the remote client are on different networks. Tailscale
creates a private WireGuard mesh so the remote Windows browser can reach the
host without exposing anything to the public Internet.

> **This project does not require any paid service.** You must create your own
> (free) Tailscale network and obtain the host's Tailscale IP. Placeholder
> values are marked `<...>` below — replace them with your own.

### 7.1 WSL networking reality check (important)

On this host WSL 2 uses **NAT** networking (`eth0` = `172.30.35.79`, not
mirrored). Consequences:

- The WSL IP is **not** reachable from other machines. Only the Windows host
  can reach it directly.
- Tailscale installed **on Windows** does **not** automatically expose a
  service running **inside WSL**.

You therefore need the Windows→WSL port forwarded (see 7.3). To check your
mode, run in WSL: `cat /etc/wsl.conf` and on Windows
`wsl.exe --version` (look for `networkingMode`; absent/mirrored vs NAT).

### 7.2 Install and authenticate Tailscale

On the **Windows host** and on the **remote Windows client**:

1. Download and install Tailscale from `https://tailscale.com/download`
   (Windows).
2. Open the Start menu → **Tailscale** → sign in with the same account
   (Google/GitHub/Microsoft). This joins both machines to one tailnet.
3. In the systray icon you'll see the machine's Tailscale IP, e.g.
   `100.x.y.z` (a `100.x.y.z` CGNAT address). You can also run
   `tailscale status` in PowerShell to list all devices and their IPs.

Identify the **host's** Tailscale IP — that is the address the remote browser
will use. *(Leave this as your own value; this project ships no IP.)*

### 7.3 Forward the Windows host port into WSL (WSL NAT)

Because WSL is NAT'ed, publish the service port from Windows into WSL with
`netsh` **inside an Administrator PowerShell**:

```powershell
# Find the current WSL eth0 IP (it changes on WSL restart):
wsl.exe -e bash -c "ip -4 addr show eth0 | grep -oP 'inet \K[\d.]+'"

# Publish Windows port 8085 to that WSL IP (run as Administrator):
netsh interface portproxy add v4tov4 listenport=8085 listenaddress=0.0.0.0 connectport=8085 connectaddress=<WSL_IP>
```

Notes:
- `connectaddress` is the **WSL eth0 IP** (e.g. `172.30.35.79`), which changes
  when WSL restarts — re-run the two commands after each restart, or pin a
  mirrored/NAT IP in `.wslconfig`.
- This requires **Administrator** rights. No router port-forwarding is used or
  needed; Tailscale handles the cross-Internet path.

### 7.4 Windows Firewall (narrow rule — do not disable the firewall)

Allow **only** the Tailscale interface and **only** TCP 8085, from inside an
**Administrator PowerShell**:

```powershell
# Get the Tailscale adapter's interface alias (usually "Tailscale"):
Get-NetAdapter | Where-Object { $_.InterfaceDescription -match "Tailscale" } | Select-Object Name

# Inbound rule: TCP 8085, scoped to the Tailscale interface only:
New-NetFirewallRule -DisplayName "PC Snapshot (Tailscale only)" `
  -Direction Inbound -Protocol TCP -LocalPort 8085 `
  -InterfaceAlias "Tailscale" -Action Allow -Profile Private
```

Do **not** scope the rule to `Any` interface — restricting it to the Tailscale
adapter keeps the port closed on LAN/public interfaces.

### 7.5 Verify

On the host, confirm the service listens on all interfaces
(`--host 0.0.0.0`, the default) and that `curl http://127.0.0.1:8085/api/health`
works.

On the **remote Windows client** (Chrome):

1. Open `http://<HOST_TAILSCALE_IP>:8085/api/health` — you should see the JSON
   health payload.
2. Open `http://<HOST_TAILSCALE_IP>:8085/` — the viewer loads.
3. Click **Fetch Point Cloud** — five PNGs appear.

Replace `<HOST_TAILSCALE_IP>` with the value from 7.2. In the UI the **API
endpoint** box can stay empty (it defaults to the page origin, i.e. the
Tailscale address you just opened).

> Cross-network testing requires two machines on a tailnet. On this single
> build host only localhost operation was verified end-to-end; the Tailscale
> steps above are the documented procedure to run on your two Windows
> machines.

### 7.6 HTTPS / mixed content

This service serves the UI and the API from the **same origin** over plain
HTTP on a private tailnet, so there is no mixed-content problem. If you later
terminate HTTPS in front of it, serve the UI and API together (or use
`tailscale serve` to add HTTPS) rather than pointing an `https://` page at an
`http://` API, which browsers block as mixed content.

### 7.7 Alternative: Cloudflare Tunnel (when UDP is blocked)

Some office/corporate networks block the **UDP** that Tailscale's WireGuard
needs. **Cloudflare Tunnel** (`cloudflared`) is a free alternative that uses
only outbound **HTTPS (TCP 443)** — which such networks almost always allow —
and gives the remote browser a public `https://...trycloudflare.com` URL. The
remote client needs **no app and no login**, just a browser.

Security difference: unlike Tailscale (private mesh), a Cloudflare quick-tunnel
URL is **publicly reachable by anyone who has the link** (the URL is
unguessable but not private). Protect it by enabling the API token (below) and
by using a Cloudflare Access login if you need stronger control.

Setup (host, inside WSL):

```bash
# 1) Install cloudflared (official binary)
curl -sL -o /tmp/cloudflared https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64
chmod +x /tmp/cloudflared && sudo mv /tmp/cloudflared /usr/local/bin/cloudflared

# 2) Start the service WITH an API token (required, since the URL is public)
PCSNAP_API_TOKEN="<choose-a-secret>" ./scripts/start_all.sh /path/to/bag.db3

# 3) Start a quick tunnel (no Cloudflare account needed)
cloudflared tunnel --url http://localhost:8085
```

`cloudflared` prints a URL like `https://xxxx-yyyy.trycloudflare.com`. Open it
in the remote browser, click **Fetch Point Cloud**, and enter the API token in
the **API token** field when prompted (or it will return HTTP 401).

Notes:
- The quick-tunnel URL **changes every time** you restart `cloudflared`. For a
  stable URL, create a named tunnel with a free Cloudflare account
  (`cloudflared tunnel create` + your own domain).
- This exposes the service to the public Internet (behind the token). Disable
  the token only if you understand that risk. To keep it fully private instead,
  use Tailscale (§7.1–§7.6).
- Only ever expose the token-protected API; do not remove the token on a public
  URL.

---

## 8. Tests

```bash
cd /home/rajesh/point_cloud_streaming_screenshots/snapshot_service
source /opt/ros/humble/setup.bash   # not required for tests, but harmless
python3 -m pytest tests/ -q
```

57 tests pass. Coverage includes:

- **Decoder** (`test_decoder.py`): XYZ + intensity, no-intensity, big-endian,
  row padding, non-finite filtering, empty messages, non-standard field
  offsets / mixed datatypes, missing-field and truncated-buffer errors,
  deterministic downsampling, read-only snapshot arrays.
- **Store** (`test_store.py`): latest-message-wins, capture-before-any-message,
  empty-message rejection, immutable capture, staleness, snapshot consistency
  when new messages arrive mid-render, thread safety.
- **Renderer** (`test_renderer.py`): five views from one snapshot, camera
  geometry for z-up and y-up (first-person front/back/left/right from the
  sensor origin), bounds/colors, non-blank PNGs, distinct views,
  valid PNG bytes, intensity mode, empty/single-point clouds, downsampled
  counts, **forced Open3D backend** (no silent Matplotlib fallback) and
  shared-renderer frame reuse.
- **API** (`test_api.py`): health with/without data, snapshot rejected when no
  data, stale warn vs reject, full flow returning five valid PNGs, all five
  images from one snapshot id, invalid view/id + path-traversal rejection, ZIP
  contents + metadata, registry eviction, required-token auth, downsample
  warning, frontend served.
- **Live stream** (`test_stream.py`): multipart framing and JPEG validity,
  placeholder heartbeats when no data, frame publication while a client
  watches, client ref-counting, render-loop idle stop, endpoint status codes
  (401/400/404), generator cleanup on disconnect, health `stream` block.

Tests are ROS-free (they use duck-typed fake PointCloud2 messages from
`tests/fixtures.py`), so they run without a bag or a live ROS graph.

---

## 9. Jetson Orin migration (later)

The service is already modular (subscriber, decoder, renderer, API,
frontend, scripts). To move it to a Jetson Orin with a Livox Mid-360:

1. **OS / ROS:** install JetPack; install ROS 2 Humble (or the distro your
   JetPack/FAST-LIO build targets). Copy this `snapshot_service/` directory.
2. **Python deps:** `pip3 install -r requirements.txt`. On Jetson, build or
   install an Open3D wheel matching your CUDA/Ubuntu, or rely on the
   automatic Matplotlib fallback (`--renderer auto`).
3. **Livox driver:** install the Livox ROS 2 driver and the Mid-360
   launch/config so it publishes the Mid-360 point cloud.
4. **FAST-LIO2:** build FAST-LIO2 for your ROS distro with the Livox
   Mid-360 config; run it on the raw LiDAR + IMU to produce a registered
   cloud (commonly `/cloud_registered`).
5. **Point the service at the registered topic:**
   `python3 -m pc_snapshot --topic /cloud_registered --up-axis z`. No code
   changes are required; only the topic (and clip/range options) change.
6. **Tailscale:** install and authenticate Tailscale on the Jetson
   (`tailscale up`), then repeat §7.3–§7.5 for the Jetson's tailnet IP.
7. **Frontend:** unchanged — point the browser at the Jetson's tailnet IP.

Do **not** install Jetson-specific drivers inside WSL; they belong on the
Jetson only.

---

## 10. Project layout

```
snapshot_service/
├── README.md
├── requirements.txt
├── config.example.json
├── pc_snapshot/
│   ├── __init__.py
│   ├── __main__.py          # CLI entrypoint
│   ├── config.py            # env + JSON + CLI configuration
│   ├── decoder.py           # PointCloud2 -> numpy (ROS-free)
│   ├── store.py             # latest-message holder + immutable snapshot
│   ├── ros_subscriber.py    # rclpy subscriber (background thread)
│   ├── renderer.py          # Open3D headless + Matplotlib fallback
│   ├── jobs.py              # bounded render jobs + snapshot registry + zip
│   └── api.py               # FastAPI app
├── static/
│   ├── index.html
│   ├── style.css
│   └── app.js
├── scripts/
│   ├── run_bag.sh
│   ├── run_service.sh
│   ├── start_all.sh
│   ├── stop_all.sh
│   └── synthetic_publisher.py
└── tests/
    ├── conftest.py
    ├── fixtures.py          # synthetic PointCloud2-like test messages
    ├── test_decoder.py
    ├── test_store.py
    ├── test_renderer.py
    └── test_api.py
```

## 11. License / data note

This code is original. The provided `.db3` dataset is third-party and is
**not** redistributed by this project; only the local file path is referenced.
