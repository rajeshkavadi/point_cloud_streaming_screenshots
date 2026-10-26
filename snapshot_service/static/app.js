/* Remote LiDAR Snapshot Viewer frontend.
 *
 * Single-shot behavior: requests are only sent when the user clicks
 * "Fetch Point Cloud". No polling of images, no streaming.
 */
"use strict";

const VIEWS = ["top", "side", "front", "perspective"];
const LS_BASE = "pcsnap_api_base";
const LS_TOKEN = "pcsnap_api_token";
const REQUEST_TIMEOUT_MS = 240000; // rendering can take a while on big clouds

const $ = (id) => document.getElementById(id);

let busy = false;
let lastZipUrl = null;

function apiBase() {
  let base = ($("api-base").value || "").trim().replace(/\/+$/, "");
  if (!base) base = window.location.origin;
  return base;
}

function authHeaders() {
  const token = ($("api-token").value || "").trim();
  return token ? { "X-API-Token": token } : {};
}

function saveSettings() {
  localStorage.setItem(LS_BASE, $("api-base").value.trim());
  localStorage.setItem(LS_TOKEN, $("api-token").value.trim());
}

function loadSettings() {
  const base = localStorage.getItem(LS_BASE);
  const token = localStorage.getItem(LS_TOKEN);
  if (base) $("api-base").value = base;
  if (token) $("api-token").value = token;
}

function setBusy(state) {
  busy = state;
  $("btn-fetch").disabled = state;
  $("spinner").hidden = !state;
  $("btn-fetch").textContent = state ? "Capturing…" : "Fetch Point Cloud";
}

function showError(msg) {
  const el = $("error");
  el.hidden = !msg;
  el.textContent = msg || "";
}

function setConn(ok, text) {
  const dot = $("conn-dot");
  dot.className = "dot " + (ok === null ? "dot-unknown" : ok ? "dot-ok" : "dot-bad");
  $("conn-text").textContent = text;
}

function fmtTime(unixSec) {
  if (!unixSec && unixSec !== 0) return "—";
  const d = new Date(unixSec * 1000);
  return d.toLocaleString() + " (" + unixSec.toFixed(3) + ")";
}

async function checkHealth() {
  setConn(null, "Checking API…");
  try {
    const resp = await fetch(apiBase() + "/api/health", {
      headers: authHeaders(),
      signal: AbortSignal.timeout(10000),
    });
    if (!resp.ok) throw new Error("HTTP " + resp.status);
    const h = await resp.json();
    const rosOk = h.ros && h.ros.connected;
    const received = h.point_cloud && h.point_cloud.message_received;
    let txt = "API online";
    if (h.ros && !h.ros.listener_alive) {
      txt += " — ROS listener DOWN";
      if (h.ros.listener_start_error) txt += " (" + h.ros.listener_start_error + ")";
      setConn(false, txt);
    } else if (!received) {
      txt += " — waiting for point cloud on " + (h.point_cloud ? h.point_cloud.topic : "?");
      setConn(null, txt);
    } else {
      const age = h.point_cloud.latest_message_age_sec;
      const stale = h.point_cloud.stale ? " (stale)" : "";
      txt += ` — cloud OK, ${h.point_cloud.latest_point_count} pts, age ${age}s${stale}`;
      setConn(true, txt);
    }
    $("status").textContent =
      `Topic: ${(h.point_cloud || {}).topic || "?"} | renderer: ${h.render ? h.render.renderer : "?"}`;
    return true;
  } catch (err) {
    setConn(false, "API unreachable: " + err.message);
    return false;
  }
}

async function fetchSnapshot() {
  if (busy) return; // prevent accidental repeated requests
  setBusy(true);
  showError("");
  saveSettings();
  try {
    const resp = await fetch(apiBase() + "/api/snapshot", {
      method: "POST",
      headers: authHeaders(),
      signal: AbortSignal.timeout(REQUEST_TIMEOUT_MS),
    });
    let body = null;
    try { body = await resp.json(); } catch (_) { /* non-JSON error */ }
    if (!resp.ok) {
      const detail = body && body.detail ? body.detail : "HTTP " + resp.status;
      throw new Error(detail);
    }
    displaySnapshot(body);
    checkHealth();
  } catch (err) {
    showError("Snapshot failed: " + err.message);
  } finally {
    setBusy(false);
  }
}

function displaySnapshot(data) {
  for (const view of VIEWS) {
    const url = data.images[view] + "?t=" + Date.now(); // cache-buster
    $("img-" + view).src = url;
    $("dl-" + view).href = data.images[view];
  }
  lastZipUrl = data.download_zip;
  $("btn-zip").disabled = false;

  $("m-id").textContent = (data.snapshot_id || "").slice(0, 12) + "…";
  $("m-capture").textContent = fmtTime(data.capture_timestamp);
  $("m-ros").textContent = data.ros_message_stamp
    ? fmtTime(data.ros_message_stamp) : "not available";
  $("m-frame").textContent = data.frame_id || "not available";
  $("m-topic").textContent = data.topic || "—";
  $("m-points").textContent =
    `${(data.valid_point_count || 0).toLocaleString()} / ${(data.rendered_point_count || 0).toLocaleString()}` +
    (data.original_point_count && data.original_point_count !== data.valid_point_count
      ? ` (msg had ${data.original_point_count.toLocaleString()})` : "");
  $("m-rendertime").textContent = (data.render_seconds || 0).toFixed(2) + " s";

  const warnRow = $("m-warnings-row");
  if (data.warnings && data.warnings.length) {
    $("m-warnings").textContent = data.warnings.join("; ");
    warnRow.hidden = false;
  } else {
    warnRow.hidden = true;
  }
  $("meta").hidden = false;
}

async function downloadViaFetch(url, filename) {
  // fetch -> blob -> object URL so cross-origin downloads work reliably
  const resp = await fetch(url, { headers: authHeaders() });
  if (!resp.ok) throw new Error("download failed: HTTP " + resp.status);
  const blob = await resp.blob();
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 5000);
}

function init() {
  loadSettings();
  $("api-base").value = $("api-base").value || window.location.origin;
  $("btn-fetch").addEventListener("click", fetchSnapshot);
  $("btn-check").addEventListener("click", checkHealth);
  $("api-base").addEventListener("change", () => { saveSettings(); checkHealth(); });
  $("api-token").addEventListener("change", () => { saveSettings(); checkHealth(); });
  $("btn-zip").addEventListener("click", async () => {
    if (!lastZipUrl) return;
    $("btn-zip").disabled = true;
    try {
      await downloadViaFetch(lastZipUrl, "point_cloud_snapshot.zip");
    } catch (err) {
      showError("ZIP download failed: " + err.message);
    } finally {
      $("btn-zip").disabled = false;
    }
  });
  // Individual downloads go through fetch for cross-origin reliability.
  for (const view of VIEWS) {
    $("dl-" + view).addEventListener("click", async (ev) => {
      ev.preventDefault();
      try {
        await downloadViaFetch(ev.target.href, view + ".png");
      } catch (err) {
        showError("PNG download failed: " + err.message);
      }
    });
  }
  checkHealth();
}

document.addEventListener("DOMContentLoaded", init);
