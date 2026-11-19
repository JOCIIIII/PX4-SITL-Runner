#!/usr/bin/env python3
"""What the vehicle's Mid-360 sees around it, as objects: the frames of /livox/lidar over --seconds, put in the
vehicle body (FLU, the measured mount), heights levelled with PX4's attitude (the vehicle may stand tilted on the
ground; --no-level = body-frame heights), ground and the vehicle's own returns removed, grouped by position. Per
object: distance and bearing from the nose (+ = left), width, height span above the ground, returns per frame and
the share of frames that saw it - "is the 20 mm pole there, and how steadily?". Thin tall objects are marked POLE?.
Also the ground level per direction and range (low percentile of the returns' heights): about 0 all round = the
mount roll / pitch are right (on level ground); rising ahead and falling behind = a pitch error or a slope.

Runs where the Livox driver is up (scripts/dev/onboard_real.sh calib NAME, then
onboard_real.sh tool lidar_scene_check.py [--yaw 0 --roll R --pitch P --z Z --centre-height H] [--no-level]).
"""
import argparse
import math
import time

import numpy as np
import rclpy
from px4_msgs.msg import VehicleAttitude
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--seconds', type=float, default=5.0)
ap.add_argument('--yaw', type=float, default=0.0)
ap.add_argument('--roll', type=float, default=-0.07)
ap.add_argument('--pitch', type=float, default=-2.28)
ap.add_argument('--z', type=float, default=0.103, help='lidar above the vehicle centre [m]')
ap.add_argument('--centre-height', type=float, default=0.55, help='vehicle centre above the ground at rest [m]')
ap.add_argument('--self-radius', type=float, default=0.9)
ap.add_argument('--max-range', type=float, default=20.0)
ap.add_argument('--min-height', type=float, default=0.25, help='above the ground [m]: lower returns are ground')
ap.add_argument('--cell', type=float, default=0.25)
ap.add_argument('--no-level', action='store_true', help='heights in the body frame (no PX4 attitude)')
a = ap.parse_args()


def rpy(r, p, y):
    r, p, y = (math.radians(v) for v in (r, p, y))
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def quat_wxyz_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])
R = rpy(a.roll, a.pitch, a.yaw)
frames, att = [], []


class Grab(Node):
    def __init__(self):
        super().__init__('lidar_scene_check')
        self.create_subscription(PointCloud2, '/livox/lidar', self.cb, qos_profile_sensor_data)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude',
                                 lambda m: att.append(np.asarray(m.q, dtype=float)), qos_profile_sensor_data)

    def cb(self, m):
        r = pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)   # mixed field types: structured
        p = np.stack([r['x'], r['y'], r['z']], axis=1).astype(np.float64) if len(r) else np.zeros((0, 3))
        frames.append(p)


rclpy.init()
n = Grab()
t0 = time.time()
while time.time() - t0 < a.seconds:
    rclpy.spin_once(n, timeout_sec=0.1)
n.destroy_node()
rclpy.shutdown()
if not frames:
    raise SystemExit('no /livox/lidar frames - is the driver up (onboard_real.sh calib)?')

# up in the body (FLU): PX4's q = FRD body -> NED at rest; without it (or --no-level) the body z
up = np.array([0.0, 0.0, 1.0])
level = 'heights in the body frame (not levelled)'
if not a.no_level:
    if att:
        Q = np.array(att)
        Q[Q @ Q[0] < 0] *= -1.0                            # q and -q are the same attitude
        q = Q.mean(axis=0)
        Rq = quat_wxyz_to_R(q / np.linalg.norm(q))
        up = FRD_TO_FLU @ (Rq.T @ np.array([0.0, 0.0, -1.0]))
        roll = math.degrees(math.atan2(Rq[2, 1], Rq[2, 2]))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, -Rq[2, 0]))))
        level = (f'levelled with PX4 attitude ({len(att)} msgs): vehicle roll {roll:+.1f} deg (right side down +), '
                 f'pitch {pitch:+.1f} deg (nose up +)')
    else:
        level = 'NO /fmu/out/vehicle_attitude (agent / flight controller?) - heights in the body frame'

pts, fid, gnd = [], [], []
for i, p in enumerate(frames):
    if len(p) == 0:
        continue
    b = p @ R.T
    b[:, 2] += a.z                                         # from the vehicle centre
    h = b @ up + a.centre_height                           # height above the ground
    r = np.hypot(b[:, 0], b[:, 1])
    g = (r > 3.0) & (r < a.max_range)
    gnd.append(np.column_stack([b[g, 0], b[g, 1], h[g]]))
    keep = (r > a.self_radius) & (r < a.max_range) & (h > a.min_height) & (h < 3.5)
    pts.append(np.column_stack([b[keep, 0], b[keep, 1], h[keep]])); fid.append(np.full(int(keep.sum()), i))
P = np.concatenate(pts) if pts else np.zeros((0, 3))
F = np.concatenate(fid) if fid else np.zeros(0, int)
nf = len(frames)
print(f'{nf} frames in {a.seconds:.0f} s, mount yaw {a.yaw} roll {a.roll} pitch {a.pitch}, lidar {a.z + a.centre_height:.2f} m '
      f'above the ground; {len(P)} returns {a.min_height}-3.5 m high within {a.self_radius}-{a.max_range} m')
print(f'  {level}')

# the ground all round: the low percentile of the returns' heights per direction and range
G = np.concatenate(gnd) if gnd else np.zeros((0, 3))
if len(G):
    gb = np.degrees(np.arctan2(G[:, 1], G[:, 0]))
    gr = np.hypot(G[:, 0], G[:, 1])
    bins = ((5, 8), (8, 12), (12, 20))
    print('ground level [m] (10th percentile of the heights):  ' + '  '.join(f'{lo:>2}-{hi:<2} m' for lo, hi in bins))
    for name, m0 in (('ahead', np.abs(gb) < 20), ('left', (gb > 70) & (gb < 110)),
                     ('right', (gb < -70) & (gb > -110)), ('behind', np.abs(gb) > 160)):
        row = []
        for lo, hi in bins:
            m = m0 & (gr >= lo) & (gr < hi)
            row.append(f'{np.percentile(G[m, 2], 10):+7.2f}' if m.sum() > 30 else '      -')
        print(f'  {name:6s}' + ' ' * 37 + '  '.join(row))

# connected cells in the horizontal plane
cell = np.floor(P[:, :2] / a.cell).astype(int)
keys = {}
for j, c in enumerate(map(tuple, cell)):
    keys.setdefault(c, []).append(j)
parent = {c: c for c in keys}


def find(c):
    while parent[c] != c:
        parent[c] = parent[parent[c]]
        c = parent[c]
    return c


for (cx, cy) in keys:
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            nb = (cx + dx, cy + dy)
            if nb in keys:
                ra, rb = find((cx, cy)), find(nb)
                if ra != rb:
                    parent[ra] = rb
groups = {}
for c, idx in keys.items():
    groups.setdefault(find(c), []).extend(idx)

rows = []
for idx in groups.values():
    q = P[idx]
    if len(q) < 3:
        continue
    xy = q[:, :2]
    c = xy.mean(0)
    d = math.hypot(*c)
    brg = math.degrees(math.atan2(c[1], c[0]))
    # width: across the line of sight (the depth spread is range noise)
    u = np.array([-c[1], c[0]]) / max(d, 1e-6)
    width = float(np.percentile(xy @ u, 95) - np.percentile(xy @ u, 5))
    h0, h1 = float(np.percentile(q[:, 2], 2)), float(np.percentile(q[:, 2], 98))
    fr = np.unique(F[idx])
    rows.append((d, brg, width, h0, h1, len(q) / nf, 100.0 * len(fr) / nf))
rows.sort()
print(f'{"dist":>6} {"bearing":>8} {"width":>6} {"height":>11} {"pts/frame":>9} {"frames seen":>11}')
for d, brg, w, h0, h1, ppf, seen in rows[:40]:
    tag = 'POLE?' if w < 0.15 and h1 - h0 > 0.6 else ''
    side = 'ahead' if abs(brg) < 15 else ('left' if brg > 0 else 'right') if abs(brg) < 165 else 'behind'
    print(f'{d:5.1f}m {brg:+7.1f}d {w:5.2f}m {h0:4.1f}-{h1:3.1f} m {ppf:9.1f} {seen:10.0f}%  {side:6s} {tag}')
