#!/usr/bin/env python3
"""Publish a synthetic spinning PointCloud2 for testing the viewer end-to-end
without any dataset. Also injects a few NaN points to exercise filtering.

    source /opt/ros/humble/setup.bash
    python3 relay/synthetic_cloud.py --topic /synthetic/points --rate 10
"""

import argparse
import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField


def build_cloud(n_points: int, t: float) -> PointCloud2:
    angles = np.linspace(0.0, 2.0 * math.pi, n_points, endpoint=False)
    rings = np.linspace(-1.0, 1.0, n_points)          # fake elevation spread
    radius = 8.0 + 2.0 * np.sin(3.0 * angles + t)     # wavy ring
    x = (radius * np.cos(angles + t)).astype(np.float32)
    y = (radius * np.sin(angles + t)).astype(np.float32)
    z = (rings + 0.3 * np.sin(5.0 * angles + 2.0 * t)).astype(np.float32)
    intensity = (0.5 + 0.5 * np.sin(7.0 * angles)).astype(np.float32)
    # Inject invalid points (viewer/relay must skip them)
    x[0], y[5], z[9] = np.nan, np.inf, -np.inf

    n = x.size
    point_step = 16
    data = np.zeros((n, point_step), dtype=np.uint8)
    data[:, 0:4] = x.view(np.uint8).reshape(n, 4)
    data[:, 4:8] = y.view(np.uint8).reshape(n, 4)
    data[:, 8:12] = z.view(np.uint8).reshape(n, 4)
    data[:, 12:16] = intensity.view(np.uint8).reshape(n, 4)

    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = "map"
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * n
    msg.is_dense = False
    msg.data = data.tobytes()
    return msg


class SyntheticCloud(Node):
    def __init__(self, topic: str, rate: float, n_points: int):
        super().__init__("synthetic_cloud")
        self.pub = self.create_publisher(PointCloud2, topic, 1)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.t = 0.0
        self.n_points = n_points

    def tick(self):
        self.t += 0.1
        self.pub.publish(build_cloud(self.n_points, self.t))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--topic", default="/synthetic/points")
    p.add_argument("--rate", type=float, default=10.0)
    p.add_argument("--points", type=int, default=20000)
    args = p.parse_args()
    rclpy.init()
    node = SyntheticCloud(args.topic, args.rate, args.points)
    print(f"[synthetic] publishing {args.points} points on {args.topic} at {args.rate} Hz")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
