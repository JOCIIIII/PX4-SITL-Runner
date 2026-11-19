#!/usr/bin/env python3
"""Forward FAST-LIO's /Odometry and /fast_lio/health from *_raw topics, leaving holes - the output gaps seen on the
Jetson on 2026-10-31 (3.2-3.5 s, sometimes longer), reproduced on a bag replay so the bridge's reaction can be
tested on this PC (scripts/dev/slam/replay_bridge.sh HOLES=...).

    python3 hole_relay.py --every 40 --hole 3.4 [--first 30]      a hole every 40 s from 30 s after the first message
    python3 hole_relay.py --at 25:3.4,80:12                        holes at 25 s (3.4 s long) and 80 s (12 s long)
Prints each hole as it happens.
"""
import argparse
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray


class HoleRelay(Node):
    def __init__(self, holes):
        super().__init__('hole_relay')
        self.holes = holes                      # [(start_s, length_s)] relative to the first message
        self.t0 = None
        self.announced = set()
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_o = self.create_publisher(Odometry, '/Odometry', 20)
        self.pub_h = self.create_publisher(Float32MultiArray, '/fast_lio/health', 20)
        self.create_subscription(Odometry, '/Odometry_raw', lambda m: self.relay(m, self.pub_o), qos)
        self.create_subscription(Float32MultiArray, '/fast_lio/health_raw', lambda m: self.relay(m, self.pub_h), qos)
        self.get_logger().info('holes (s from first message): ' + ', '.join(f'{s:.0f}+{l:.1f}' for s, l in holes))

    def relay(self, msg, pub):
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        for i, (s, l) in enumerate(self.holes):
            if s <= t < s + l:
                if i not in self.announced:
                    self.announced.add(i)
                    self.get_logger().info(f'hole {i + 1}: dropping {l:.1f} s from t+{t:.1f} s')
                return
        pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--every', type=float, default=0.0)
    ap.add_argument('--hole', type=float, default=3.4)
    ap.add_argument('--first', type=float, default=30.0)
    ap.add_argument('--at', default='')
    ap.add_argument('--until', type=float, default=3600.0)
    a = ap.parse_args()
    holes = []
    if a.at:
        holes = [(float(x.split(':')[0]), float(x.split(':')[1])) for x in a.at.split(',') if x]
    elif a.every > 0:
        s = a.first
        while s < a.until:
            holes.append((s, a.hole))
            s += a.every
    rclpy.init()
    node = HoleRelay(holes)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
