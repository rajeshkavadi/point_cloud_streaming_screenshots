// LiDAR point-cloud viewer: Three.js + WebSocket (binary relay or rosbridge).
import * as THREE from "three";
import { OrbitControls } from "three/addons/OrbitControls.js";

const PC2 = window.PC2;

// ---------------------------------------------------------------------------
// DOM
// ---------------------------------------------------------------------------
const $ = (id) => document.getElementById(id);
const ui = {
  transport: $("transport"), wsUrl: $("ws-url"), topic: $("topic"),
  connect: $("btn-connect"), disconnect: $("btn-disconnect"),
  pointSize: $("point-size"), pointSizeVal: $("point-size-val"),
  colorMode: $("color-mode"), maxPoints: $("max-points"),
  maxPointsVal: $("max-points-val"), clipRange: $("clip-range"),
  resetView: $("btn-reset-view"), tests: $("btn-tests"),
  connDot: $("conn-dot"), connText: $("conn-text"),
  rate: $("stat-rate"), points: $("stat-points"), bw: $("stat-bw"),
  frames: $("stat-frames"), dropped: $("stat-dropped"),
  statTopic: $("stat-topic"), decode: $("stat-decode"),
  errors: $("errors"),
};

function setStatus(kind, text) {
  ui.connDot.className = "dot " + kind;
  ui.connText.textContent = text;
}
function showError(text) {
  const div = document.createElement("div");
  div.textContent = new Date().toLocaleTimeString() + " — " + text;
  ui.errors.prepend(div);
  while (ui.errors.children.length > 12) ui.errors.lastChild.remove();
}

// ---------------------------------------------------------------------------
// Three.js scene
// ---------------------------------------------------------------------------
const viewport = $("viewport");
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0d10);
const camera = new THREE.PerspectiveCamera(60, 1, 0.1, 2000);
camera.position.set(0, -30, 15);
const renderer = new THREE.WebGLRenderer({ antialias: false });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
viewport.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.1;

scene.add(new THREE.AxesHelper(5));
const grid = new THREE.GridHelper(100, 50, 0x223, 0x1a222c);
scene.add(grid);

// Preallocated GPU buffers — reused for every message, never recreated.
const MAX_CAPACITY = 200000;
const positions = new Float32Array(MAX_CAPACITY * 3);
const colors = new Float32Array(MAX_CAPACITY * 3);
const geometry = new THREE.BufferGeometry();
geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
geometry.setDrawRange(0, 0);
const material = new THREE.PointsMaterial({
  size: 2, vertexColors: true, sizeAttenuation: false,
});
const cloud = new THREE.Points(geometry, material);
scene.add(cloud);

function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener("resize", resize);
resize();

// Turbo-like color LUT for height/intensity coloring
const LUT = new Uint8Array(256 * 3);
(function buildLUT() {
  for (let i = 0; i < 256; i++) {
    const t = i / 255;
    LUT[i * 3] = Math.round(255 * Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 3))));
    LUT[i * 3 + 1] = Math.round(255 * Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 2))));
    LUT[i * 3 + 2] = Math.round(255 * Math.min(1, Math.max(0, 1.5 - Math.abs(4 * t - 1))));
  }
})();

let firstFrameDone = false;
function fitCameraIfNeeded(n) {
  if (firstFrameDone || n === 0) return;
  firstFrameDone = true;
  let minx = Infinity, miny = Infinity, minz = Infinity;
  let maxx = -Infinity, maxy = -Infinity, maxz = -Infinity;
  for (let i = 0; i < n; i++) {
    const x = positions[i * 3], y = positions[i * 3 + 1], z = positions[i * 3 + 2];
    if (x < minx) minx = x; if (x > maxx) maxx = x;
    if (y < miny) miny = y; if (y > maxy) maxy = y;
    if (z < minz) minz = z; if (z > maxz) maxz = z;
  }
  const cx = (minx + maxx) / 2, cy = (miny + maxy) / 2, cz = (minz + maxz) / 2;
  const radius = Math.max(maxx - minx, maxy - miny, maxz - minz, 1) / 2;
  controls.target.set(cx, cy, cz);
  camera.position.set(cx, cy - radius * 2.2, cz + radius * 1.2);
  camera.far = Math.max(2000, radius * 20);
  camera.updateProjectionMatrix();
  grid.position.set(cx, cy, minz);
}

function applyColors(n, intensity) {
  const mode = ui.colorMode.value;
  let lo = 0, hi = 1;
  if (mode === "height") {
    lo = Infinity; hi = -Infinity;
    for (let i = 0; i < n; i++) {
      const z = positions[i * 3 + 2];
      if (z < lo) lo = z; if (z > hi) hi = z;
    }
  } else if (mode === "intensity" && intensity) {
    hi = 0;
    for (let i = 0; i < n; i++) if (intensity[i] > hi) hi = intensity[i];
    lo = 0;
  }
  const span = hi - lo || 1;
  for (let i = 0; i < n; i++) {
    let t;
    if (mode === "uniform") t = -1;
    else if (mode === "intensity" && intensity) t = Math.min(1, Math.max(0, intensity[i] / span));
    else t = Math.min(1, Math.max(0, (positions[i * 3 + 2] - lo) / span));
    if (t < 0) {
      colors[i * 3] = 0.9; colors[i * 3 + 1] = 0.93; colors[i * 3 + 2] = 1.0;
    } else {
      const k = (t * 255) | 0;
      colors[i * 3] = LUT[k * 3] / 255;
      colors[i * 3 + 1] = LUT[k * 3 + 1] / 255;
      colors[i * 3 + 2] = LUT[k * 3 + 2] / 255;
    }
  }
}

/** Update GPU buffers from flat xyz arrays — no scene recreation. */
function updateCloud(x, y, z, intensity) {
  const n = Math.min(x.length, MAX_CAPACITY);
  for (let i = 0; i < n; i++) {
    positions[i * 3] = x[i];
    positions[i * 3 + 1] = y[i];
    positions[i * 3 + 2] = z[i];
  }
  applyColors(n, intensity);
  geometry.attributes.position.needsUpdate = true;
  geometry.attributes.color.needsUpdate = true;
  geometry.setDrawRange(0, n);
  fitCameraIfNeeded(n);
  ui.points.textContent = n.toLocaleString();
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();

// ---------------------------------------------------------------------------
// Stats (latest-frame-wins: incoming frames simply overwrite)
// ---------------------------------------------------------------------------
const stats = {
  frames: 0, dropped: 0, bytesWindow: 0, lastSeq: null,
  windowStart: performance.now(),
};

function noteFrame(byteLength, seq) {
  stats.frames++;
  stats.bytesWindow += byteLength || 0;
  if (seq != null) {
    if (stats.lastSeq != null && seq > stats.lastSeq + 1) {
      stats.dropped += seq - stats.lastSeq - 1;
    }
    stats.lastSeq = seq;
  }
  const now = performance.now();
  const dt = (now - stats.windowStart) / 1000;
  if (dt >= 1.0) {
    ui.rate.textContent = (stats.frames / dt).toFixed(1);
    ui.bw.textContent = (stats.bytesWindow / 1024 / dt).toFixed(1);
    ui.frames.textContent = stats.frames;
    ui.dropped.textContent = stats.dropped;
    stats.frames = 0; stats.bytesWindow = 0; stats.windowStart = now;
  }
}

// ---------------------------------------------------------------------------
// WebSocket transports
// ---------------------------------------------------------------------------
let ws = null;
let reconnectTimer = null;
let reconnectAttempts = 0;
const RECONNECT_MAX = 5;

function sendCmd(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

function applySettingsToServer() {
  sendCmd({ op: "set_max_points", max_points: +ui.maxPoints.value });
  const clip = +ui.clipRange.value;
  sendCmd({ op: "set_clip", clip_range: clip > 0 ? clip : null, z_min: null, z_max: null });
}

function handleBinaryFrame(buf) {
  const view = new DataView(buf);
  const magic = String.fromCharCode(view.getUint8(0), view.getUint8(1), view.getUint8(2), view.getUint8(3));
  if (magic !== "PCRD") { showError("bad binary frame magic: " + magic); return; }
  const hdrLen = view.getUint32(4); // network byte order
  const hdr = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 8, hdrLen)));
  const n = hdr.points;
  const payload = new Float32Array(buf, 8 + hdrLen, (buf.byteLength - 8 - hdrLen) / 4);
  const x = payload.subarray(0, n);
  const y = payload.subarray(n, 2 * n);
  const z = payload.subarray(2 * n, 3 * n);
  const intensity = hdr.has_intensity ? payload.subarray(3 * n, 4 * n) : null;
  ui.statTopic.textContent = hdr.topic + " [" + (hdr.frame_id || "?") + "]";
  ui.decode.textContent = (hdr.decode_ms || 0) + " ms (relay)";
  updateCloud(x, y, z, intensity);
  noteFrame(buf.byteLength, hdr.seq);
}

function handleRosbridgeMessage(msg) {
  try {
    const t0 = performance.now();
    const r = PC2.decodePointCloud2(msg.msg, { maxPoints: +ui.maxPoints.value });
    ui.decode.textContent = (performance.now() - t0).toFixed(1) + " ms (browser)";
    ui.statTopic.textContent = msg.topic;
    updateCloud(r.positions.subarray(0, r.count * 3), null, null, null);
    if (r.skipped > 0 && stats.frames % 100 === 0) {
      showError(`skipped ${r.skipped} invalid points`);
    }
    noteFrame((msg.msg.data && msg.msg.data.length) || 0, null);
  } catch (e) {
    showError("decode error: " + e.message);
  }
}

function connect() {
  disconnect(false);
  const url = ui.wsUrl.value.trim();
  const transport = ui.transport.value;
  setStatus("", "connecting…");
  ui.connect.disabled = true;
  try {
    ws = new WebSocket(url);
  } catch (e) {
    setStatus("err", "invalid URL");
    showError("invalid WebSocket URL: " + e.message);
    ui.connect.disabled = false;
    return;
  }
  if (transport === "binary") ws.binaryType = "arraybuffer";

  ws.onopen = () => {
    setStatus("on", "connected");
    ui.connect.disabled = false;
    ui.disconnect.disabled = false;
    reconnectAttempts = 0;
    if (transport === "rosbridge") {
      ws.send(JSON.stringify({
        op: "subscribe", topic: ui.topic.value.trim(),
        type: "sensor_msgs/msg/PointCloud2", throttle_rate: 0, queue_length: 1,
      }));
    } else {
      sendCmd({ op: "set_topic", topic: ui.topic.value.trim() });
      applySettingsToServer();
    }
  };
  ws.onmessage = (ev) => {
    if (typeof ev.data === "string") {
      let m;
      try { m = JSON.parse(ev.data); } catch { return; }
      if (m.op === "publish") handleRosbridgeMessage(m);
      else if (m.op === "stats") {
        ui.rate.textContent = m.send_hz;
        ui.bw.textContent = (m.kbps_out / 8).toFixed(1);
      } else if (m.op === "status" && m.topic !== undefined) {
        ui.statTopic.textContent = m.topic;
      } else if (m.op === "error") {
        showError("server: " + (m.msg || JSON.stringify(m)));
      }
    } else if (ev.data instanceof ArrayBuffer) {
      handleBinaryFrame(ev.data);
    }
  };
  ws.onclose = (ev) => {
    ui.connect.disabled = false;
    ui.disconnect.disabled = true;
    if (ev.code === 1000 && !reconnectTimer) {
      setStatus("off", "disconnected");
      return;
    }
    setStatus("err", `connection lost (${ev.code})`);
    showError(`WebSocket closed: code=${ev.code} reason=${ev.reason || "none"}`);
    scheduleReconnect();
  };
  ws.onerror = () => showError("WebSocket error (see console) — check URL, relay running, firewall");
}

function scheduleReconnect() {
  if (reconnectAttempts >= RECONNECT_MAX) {
    setStatus("err", "gave up reconnecting");
    return;
  }
  const delay = Math.min(1000 * 2 ** reconnectAttempts, 10000);
  reconnectAttempts++;
  showError(`reconnecting in ${delay / 1000}s (attempt ${reconnectAttempts}/${RECONNECT_MAX})`);
  reconnectTimer = setTimeout(() => { reconnectTimer = null; connect(); }, delay);
}

function disconnect(userInitiated) {
  if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
  if (ws) {
    ws.onclose = null;
    try { ws.close(1000); } catch {}
    ws = null;
  }
  if (userInitiated) {
    setStatus("off", "disconnected");
    ui.connect.disabled = false;
    ui.disconnect.disabled = true;
  }
}

// ---------------------------------------------------------------------------
// UI wiring
// ---------------------------------------------------------------------------
ui.connect.onclick = connect;
ui.disconnect.onclick = () => disconnect(true);
ui.resetView.onclick = () => { firstFrameDone = false; };
ui.tests.onclick = () => window.open("tests.html", "_blank");
ui.pointSize.oninput = () => {
  material.size = +ui.pointSize.value;
  ui.pointSizeVal.textContent = "(" + ui.pointSize.value + " px)";
};
ui.colorMode.onchange = () => {}; // colors recompute on next frame
ui.maxPoints.oninput = () => {
  ui.maxPointsVal.textContent = "(" + (+ui.maxPoints.value).toLocaleString() + ")";
  applySettingsToServer();
};
ui.clipRange.onchange = applySettingsToServer;
ui.topic.onchange = () => {
  if (ui.transport.value === "rosbridge" && ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ op: "unsubscribe", topic: ui.topic.value.trim() }));
    ws.send(JSON.stringify({
      op: "subscribe", topic: ui.topic.value.trim(),
      type: "sensor_msgs/msg/PointCloud2", throttle_rate: 0, queue_length: 1,
    }));
  } else {
    sendCmd({ op: "set_topic", topic: ui.topic.value.trim() });
  }
};

// ---------------------------------------------------------------------------
// Init from config.json (served next to index.html)
// ---------------------------------------------------------------------------
fetch("config.json")
  .then((r) => r.json())
  .then((cfg) => {
    ui.transport.value = cfg.transport || "binary";
    ui.wsUrl.value = cfg.ws_url || "ws://localhost:9091";
    ui.topic.value = cfg.topic || "/ouster/points";
    ui.pointSize.value = cfg.point_size || 2;
    ui.pointSize.oninput();
    ui.colorMode.value = cfg.color_mode || "height";
    ui.maxPoints.value = cfg.max_points || 50000;
    ui.maxPoints.oninput();
    ui.clipRange.value = cfg.clip_range || 0;
    if (cfg.auto_connect) connect();
  })
  .catch(() => {
    ui.wsUrl.value = "ws://localhost:9091";
    ui.topic.value = "/ouster/points";
    showError("config.json not found — using defaults");
  });
