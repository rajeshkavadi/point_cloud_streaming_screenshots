#!/usr/bin/env python3
"""PointCloud2 -> binary WebSocket relay.

Subscribes to a ROS 2 sensor_msgs/PointCloud2 topic, keeps only the latest
message (latest-message-wins, bounded memory), optionally clips/downsamples,
and broadcasts packed float32 XYZ(+intensity) frames to browser clients over
a WebSocket. Designed for Internet streaming: ~12-16 bytes/point vs ~90+
bytes/point for rosbridge JSON.

Requires (inside a sourced ROS 2 environment):
    pip3 install websockets numpy
Usage:
    source /opt/ros/humble/setup.bash
    python3 relay/pc_relay.py --topic /ouster/points --port 9091 \
        --rate 10 --max-points 50000 --clip-range 120

Client -> server JSON commands:
    {"op":"subscribe"}                       start/stop is implicit, frames
                                             flow once a topic is publishing
    {"op":"set_topic","topic":"/foo"}        change topic at runtime
    {"op":"set_max_points","max_points":N}
    {"op":"set_clip","clip_range":M,"z_min":Z0,"z_max":Z1}
Server -> client:
    text JSON: {"op":"status"|"info"|"stats"|"error", ...}
    binary frame: magic 'PCRD' | uint32 hdr_len | JSON hdr | float32 payload
    payload = x[n], y[n], z[n], (i[n]) little-endian
"""

import argparse
import asyncio
import json
import struct
import threading
import time

import numpy as np
import websockets

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from pc_unpack import unpack_xyz_intensity, process_frame

MAGIC = b"PCRD"
HEADER_STRUCT = struct.Struct("!4sI")


def pack_frame(header: dict, x, y, z, intensity) -> bytes:
    hdr = json.dumps(header, separators=(",", ":")).encode("utf-8")
    parts = [HEADER_STRUCT.pack(MAGIC, len(hdr)), hdr,
             x.tobytes(), y.tobytes(), z.tobytes()]
    if intensity is not None:
        parts.append(intensity.tobytes())
    return b"".join(parts)


class RelayState:
    """Shared between the rclpy thread and the asyncio loop. Latest-frame-wins:
    only one frame is ever retained."""

    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None          # packed float32 arrays (already filtered)
        self.header = None         # json-serializable metadata
        self.seq = 0
        self.frames_rx = 0
        self.bytes_rx = 0
        self.desired_topic = None
        self.current_topic = None
        self.topic_requested = threading.Event()
        self.max_points = 50000
        self.clip_range = None
        self.z_min = None
        self.z_max = None
        self.last_rx_wall = None


class RelayRos(Node):
    def __init__(self, state: RelayState, topic: str):
        super().__init__("pc_relay")
        self.state = state
        self.state.desired_topic = topic
        self.sub = None
        # Periodic check lets clients change topic from the asyncio thread
        # safely; subscription (re)creation happens here in the spin thread.
        self.create_timer(0.5, self._maybe_resubscribe)
        self._maybe_resubscribe()

    def _maybe_resubscribe(self):
        s = self.state
        if s.desired_topic != s.current_topic:
            if self.sub is not None:
                self.destroy_subscription(self.sub)
                self.sub = None
            topic = s.desired_topic
            self.sub = self.create_subscription(
                PointCloud2, topic, self._callback, 1)  # depth 1: drop stale
            s.current_topic = topic
            print(f"[relay] subscribed to {topic}")

    def _callback(self, msg: PointCloud2):
        s = self.state
        t0 = time.perf_counter()
        try:
            x, y, z, intensity = unpack_xyz_intensity(
                msg.data, [(f.name, f.offset, f.datatype, f.count) for f in msg.fields],
                msg.point_step, msg.is_bigendian)
        except Exception as e:  # malformed message: report once per burst
            if s.frames_rx % 50 == 0:
                print(f"[relay] unpack error: {e}")
            return
        with s.lock:
            xr, yr, zr, ir, stats = process_frame(
                x, y, z, intensity,
                max_points=s.max_points, clip_range=s.clip_range,
                z_min=s.z_min, z_max=s.z_max)
            s.seq += 1
            s.frames_rx += 1
            s.bytes_rx += int(len(msg.data))
            s.last_rx_wall = time.time()
            s.header = {
                "op": "frame",
                "seq": s.seq,
                "topic": s.current_topic,
                "frame_id": msg.header.frame_id,
                "stamp_ns": int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec),
                "points": int(xr.size),
                "total_points": stats["input"],
                "filtered_invalid": stats["filtered_invalid"],
                "filtered_clip": stats["filtered_clip"],
                "has_intensity": ir is not None,
                "decode_ms": round((time.perf_counter() - t0) * 1000.0, 2),
            }
            s.frame = (xr, yr, zr, ir, dict(s.header))


async def client_sender(websocket, state: RelayState, send_rate_hz: float):
    """Sends latest frames; independent from the receive loop."""
    period = 1.0 / max(send_rate_hz, 0.1)
    last_seq_sent = -1
    last_send = 0.0
    bytes_sent = 0
    frames_sent = 0
    stats_t = time.time()
    try:
        while True:
            await asyncio.sleep(0.005)
            now = time.perf_counter()
            if now - last_send < period:
                continue
            with state.lock:
                if state.frame is None or state.seq == last_seq_sent:
                    continue
                xr, yr, zr, ir, hdr = state.frame
                seq = state.seq
            payload = pack_frame(hdr, xr, yr, zr, ir)
            await websocket.send(payload)
            last_seq_sent = seq
            last_send = now
            bytes_sent += len(payload)
            frames_sent += 1
            if time.time() - stats_t >= 2.0:
                await websocket.send(json.dumps({
                    "op": "stats",
                    "send_hz": round(frames_sent / (time.time() - stats_t), 2),
                    "kbps_out": round(bytes_sent * 8 / 1024 / (time.time() - stats_t), 1),
                    "points": hdr.get("points", 0),
                    "frames_rx_total": state.frames_rx,
                }))
                stats_t = time.time()
                bytes_sent = 0
                frames_sent = 0
    except (websockets.ConnectionClosed, asyncio.CancelledError):
        pass


async def client_receiver(websocket, state: RelayState):
    try:
        async for raw in websocket:
            try:
                cmd = json.loads(raw)
            except (json.JSONDecodeError, TypeError):
                continue
            op = cmd.get("op")
            if op == "set_topic":
                state.desired_topic = str(cmd.get("topic", state.desired_topic))
                await websocket.send(json.dumps(
                    {"op": "status", "text": f"switching topic to {state.desired_topic}"}))
            elif op == "set_max_points":
                state.max_points = max(1, int(cmd.get("max_points", state.max_points)))
            elif op == "set_clip":
                state.clip_range = cmd.get("clip_range", state.clip_range)
                state.z_min = cmd.get("z_min", state.z_min)
                state.z_max = cmd.get("z_max", state.z_max)
            elif op == "ping":
                await websocket.send(json.dumps({"op": "pong", "t": cmd.get("t")}))
    except (websockets.ConnectionClosed, asyncio.CancelledError):
        pass


async def handler(websocket, state: RelayState, send_rate_hz: float):
    print(f"[relay] client connected: {websocket.remote_address}")
    await websocket.send(json.dumps({
        "op": "status", "text": "connected",
        "topic": state.current_topic, "protocol": "pcrd-v1"}))
    sender = asyncio.create_task(client_sender(websocket, state, send_rate_hz))
    try:
        await client_receiver(websocket, state)
    finally:
        sender.cancel()


async def monitor(state: RelayState):
    while True:
        await asyncio.sleep(5.0)
        with state.lock:
            age = None if state.last_rx_wall is None else round(time.time() - state.last_rx_wall, 1)
            hdr = state.header
        msg = {"op": "status", "text": "alive",
               "topic": state.current_topic,
               "frames_rx": state.frames_rx,
               "last_msg_age_s": age,
               "points_last": None if hdr is None else hdr.get("points")}
        print(f"[relay] frames_rx={state.frames_rx} topic={state.current_topic} "
              f"age={age}s points={None if hdr is None else hdr.get('points')}")


async def main_async(args, state):
    async with websockets.serve(
            lambda ws: handler(ws, state, args.rate), args.host, args.port,
            max_size=16 * 1024 * 1024, ping_interval=20, ping_timeout=20):
        print(f"[relay] WebSocket listening on ws://{args.host}:{args.port}")
        await monitor(state)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--topic", default="/ouster/points", help="PointCloud2 topic")
    p.add_argument("--host", default="0.0.0.0", help="bind address (restrict to LAN/TS iface for security)")
    p.add_argument("--port", type=int, default=9091)
    p.add_argument("--rate", type=float, default=10.0, help="max frames/s sent to each client")
    p.add_argument("--max-points", type=int, default=50000, help="cap points per frame")
    p.add_argument("--clip-range", type=float, default=None, help="drop points beyond this radius (m)")
    p.add_argument("--z-min", type=float, default=None)
    p.add_argument("--z-max", type=float, default=None)
    args = p.parse_args()

    state = RelayState()
    state.max_points = args.max_points
    state.clip_range = args.clip_range
    state.z_min = args.z_min
    state.z_max = args.z_max

    rclpy.init()
    node = RelayRos(state, args.topic)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        asyncio.run(main_async(args, state))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
