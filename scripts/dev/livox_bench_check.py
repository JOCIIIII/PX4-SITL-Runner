#!/usr/bin/env python3
"""Bench check of the vehicle's Mid-360 through livox_ros_driver2 (/livox/lidar), before the shadow test:
frame rate and spacing, points per frame, the per-point time field (span ~0.1 s, ns, increasing), returns
close to the sensor (vehicle parts in view: anything within --near m, by azimuth), elevation coverage.
usage (in the onboard image, with the driver up): livox_bench_check.py [SECONDS] [--near 1.0]"""
import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

ap = argparse.ArgumentParser()
ap.add_argument('seconds', nargs='?', type=float, default=10.0)
ap.add_argument('--near', type=float, default=1.0)
ap.add_argument('--topic', default='/livox/lidar')
a = ap.parse_args()


class Check(Node):
    def __init__(self):
        super().__init__('livox_bench_check')
        self.rx, self.n, self.span, self.mono, self.near, self.el, self.az_near = [], [], [], [], [], [], []
        self.fields = None
        self.create_subscription(PointCloud2, a.topic, self.cb, qos_profile_sensor_data)

    def cb(self, m):
        self.rx.append(time.monotonic())
        self.fields = [(f.name, f.datatype) for f in m.fields]
        arr = pc2.read_points(m, field_names=('x', 'y', 'z', 'timestamp'), skip_nans=False)
        xyz = np.column_stack([np.asarray(arr[c], dtype=np.float64) for c in ('x', 'y', 'z')])
        ts = np.asarray(arr['timestamp'], dtype=np.float64)
        r = np.linalg.norm(xyz, axis=1)
        ok = np.isfinite(r) & (r > 0.0)
        self.n.append(int(ok.sum()))
        self.span.append((ts.max() - ts.min()) * 1e-9 if ts.size else float('nan'))
        self.mono.append(float(np.mean(np.diff(ts) >= 0)) if ts.size > 1 else float('nan'))
        near = ok & (r < a.near)
        self.near.append(int(near.sum()))
        if near.any():
            self.az_near.append(np.degrees(np.arctan2(xyz[near, 1], xyz[near, 0])))
        self.el.append(np.degrees(np.arcsin(np.clip(xyz[ok, 2] / r[ok], -1, 1))))


rclpy.init()
c = Check()
t_end = time.monotonic() + a.seconds
while time.monotonic() < t_end:
    rclpy.spin_once(c, timeout_sec=0.1)
if len(c.rx) < 2:
    print(f'no frames on {a.topic} in {a.seconds:.0f} s (driver up? lidar at 192.168.1.100, this computer 192.168.1.50?)')
    raise SystemExit(1)
gap = np.diff(c.rx) * 1e3
el = np.concatenate(c.el)
print(f'{len(c.rx)} frames in {a.seconds:.0f} s = {len(c.rx) / a.seconds:.1f} Hz; spacing ms median {np.median(gap):.1f}, '
      f'p95 {np.percentile(gap, 95):.1f}, max {gap.max():.1f}')
print(f'fields: {c.fields}')
print(f'points per frame: median {int(np.median(c.n))} (min {min(c.n)}, max {max(c.n)})')
print(f'time field span per frame: median {np.nanmedian(c.span):.4f} s (as ns: 0.1 s expected), '
      f'increasing {np.nanmean(c.mono) * 100:.1f} % of steps')
print(f'elevation of returns: {np.percentile(el, 0.5):.1f} .. {np.percentile(el, 99.5):.1f} deg')
print(f'returns within {a.near} m: median {int(np.median(c.near))} per frame', end='')
if c.az_near:
    az = np.concatenate(c.az_near)
    h, e = np.histogram(az, bins=12, range=(-180, 180))
    print(' - by azimuth (deg, 0 = forward, + = left): ' +
          ', '.join(f'{e[i]:.0f}..{e[i + 1]:.0f}: {h[i] / len(c.rx):.0f}' for i in range(12) if h[i]))
else:
    print()
c.destroy_node()
rclpy.shutdown()
