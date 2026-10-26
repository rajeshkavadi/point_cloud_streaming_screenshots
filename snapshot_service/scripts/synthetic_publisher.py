#!/usr/bin/env python3
"""Publish a synthetic PointCloud2 for testing without a bag.

Publishes a 20 m x 20 m "outdoor" scene (ground plane + two walls + a vehicle-
shaped box) as sensor_msgs/PointCloud2 on --topic at --rate Hz, with an
optional intensity field. Useful to validate the snapshot service end-to-end
before a real dataset is configured.

Usage (inside sourced ROS 2):
    python3 synthetic_publisher.py --topic /ouster/points --rate 5
"""

import argparse
import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def make_cloud(t: float, include_intensity: bool = True) -> PointCloud2:
    """Build one synthetic frame deterministically from time t."""
    pts = []
    # Ground plane 20x20 at z=0
    for i in range(-10, 11):
        for j in range(-10, 11):
            x, y = float(i), float(j)
            z = 0.02 * math.sin(0.3 * x + t) + 0.02 * math.cos(0.3 * y)
            pts.append((x, y, z, 30.0 + 2.0 * z))
    # Two walls
    for i in range(0, 41):
        z = i * 0.1
        pts.append((10.0, -10.0 + 0.5 * (i % 20), z, 120.0))
        pts.append((-10.0 + 0.5 * (i % 20), 10.0, z, 90.0))
    # Vehicle-sized box (4x2x1.5 m) that slowly "moves"
    cx = 2.0 * math.sin(0.2 * t)
    for dx in (-2.0, 2.0):
        for dy in (-1.0, 1.0):
            for k in range(0, 16):
                z = k * 0.1
                pts.append((cx + dx, dy, z + 0.05, 200.0))
    # Roof
    for rx in range(-19, 20):
        for ry in (-9, 9):
            pts.append((cx + rx * 0.1, ry * 0.1, 1.5, 180.0))

    n = len(pts)
    header = Header()
    header.stamp = rclpy.clock.Clock().now().to_msg()
    header.frame_id = "camera_init"

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    if include_intensity:
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 16
        buf = b"".join(struct.pack("<ffff", *p) for p in pts)
    else:
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 12
        buf = b"".join(struct.pack("<fff", p[0], p[1], p[2]) for p in pts)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * n
    msg.data = buf
    msg.is_dense = True
    return msg


class SyntheticPublisher(Node):
    def __init__(self, topic: str, rate: float):
        super().__init__("pc_snapshot_synthetic_publisher")
        self.pub = self.create_publisher(PointCloud2, topic, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.t = 0.0
        self.get_logger().info(f"publishing synthetic clouds on {topic} at {rate} Hz")

    def tick(self):
        self.t += 0.2
        self.pub.publish(make_cloud(self.t))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/ouster/points")
    ap.add_argument("--rate", type=float, default=5.0)
    ap.add_argument("--no-intensity", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = SyntheticPublisher(args.topic, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
