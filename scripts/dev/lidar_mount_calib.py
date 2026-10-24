#!/usr/bin/env python3
"""Mid-360 mount on the vehicle, for the lidar adapter (scripts/ros2/apps/noclimb_ca.sh NOCLIMB_LIDAR_*): the
rotation lidar -> body (FLU; the adapter's lidar_roll / _pitch / _yaw, R = Rz(yaw) Ry(pitch) Rx(roll)), the
height, and the latency of its frames against PX4's data. Runs where the Livox driver and the PX4 agent are up
(scripts/dev/onboard_real.sh calib, then docker exec ros2-real ...). PROPS OFF.

  lidar_mount_calib.py rest  [--yaw 180] [--centre-height 0.23] [--seconds 10]
      The vehicle still on open ground. Roll / pitch: gravity seen by the Mid-360's own IMU (/livox/imu, axes =
      the point cloud's) against PX4's attitude - needs no level ground; yaw (from `shake` or the mount) is held.
      Cross-check: the ground plane fitted to the returns 2.5-20 m out (its tilt against the IMU's gravity = how
      level the ground is) and the lidar's height above it -> NOCLIMB_LIDAR_Z = height - centre height (the
      vehicle centre above the ground at rest, measured; X500 SITL model 0.23).
  lidar_mount_calib.py shake [--seconds 25]
      Rock the vehicle by hand about all three axes (roll, pitch AND yaw, ~1 Hz, +-20 deg) for the whole time.
      Latency: cross-correlation of the two gyros' rates over receipt time (/livox/imu vs PX4 sensor_combined) =
      how much later the lidar's data arrive than PX4's = NOCLIMB_LIDAR_LATENCY (the adapter matches each
      point to the PX4 pose by receipt time). Rotation: the gyro vectors aligned (Kabsch) after that lag - roll,
      pitch and yaw at once; the conditioning line says whether every axis was rocked enough.
  lidar_mount_calib.py target [--yaw 180] [--roll 0] [--pitch 0] [--seconds 5]
      Something narrow (a pole, a person) straight ahead of the nose 3-6 m out, nothing else within 8 m in front:
      its bearing in the body frame with the given mount; a yaw error shows as a bearing off 0.
Prints the NOCLIMB_LIDAR_* lines to put on the onboard_real.sh command line (they override the profile)."""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from px4_msgs.msg import SensorCombined, VehicleAttitude
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('mode', choices=('rest', 'shake', 'target'))
ap.add_argument('--seconds', type=float, default=None)
ap.add_argument('--yaw', type=float, default=180.0, help='mount yaw [deg] held in rest / used in target')
ap.add_argument('--roll', type=float, default=0.0)
ap.add_argument('--pitch', type=float, default=0.0)
ap.add_argument('--centre-height', type=float, default=0.23, help='vehicle centre above the ground at rest [m]')
ap.add_argument('--lidar', default='/livox/lidar')
ap.add_argument('--imu', default='/livox/imu')
a = ap.parse_args()
SECONDS = a.seconds or {'rest': 10.0, 'shake': 25.0, 'target': 5.0}[a.mode]


def rpy_to_R(r, p, y):
    """the adapter's quat_from_rpy_xyzw: R = Rz(y) Ry(p) Rx(r)"""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def R_to_rpy(R):
    return (math.atan2(R[2, 1], R[2, 2]), math.asin(max(-1.0, min(1.0, -R[2, 0]))), math.atan2(R[1, 0], R[0, 0]))


def quat_wxyz_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])


class Collect(Node):
    def __init__(self):
        super().__init__('lidar_mount_calib')
        self.imu, self.px4_gyro, self.att, self.clouds = [], [], [], []
        self.create_subscription(Imu, a.imu, self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.sc_cb, qos_profile_sensor_data)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.att_cb, qos_profile_sensor_data)
        if a.mode in ('rest', 'target'):
            self.create_subscription(PointCloud2, a.lidar, self.cloud_cb, qos_profile_sensor_data)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def imu_cb(self, m):
        self.imu.append((self.now(), m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                         m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))

    def sc_cb(self, m):
        g = FRD_TO_FLU @ np.asarray(m.gyro_rad, dtype=float)
        self.px4_gyro.append((self.now(), *g))

    def att_cb(self, m):
        self.att.append(np.asarray(m.q, dtype=float))

    def cloud_cb(self, m):
        if len(self.clouds) < 60:
            arr = pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
            xyz = np.column_stack([np.asarray(arr[c], dtype=np.float64) for c in ('x', 'y', 'z')])
            self.clouds.append(xyz[np.isfinite(xyz).all(axis=1)])


rclpy.init()
n = Collect()
print(f'{a.mode}: collecting {SECONDS:.0f} s ...', flush=True)
t_end = time.monotonic() + SECONDS
while time.monotonic() < t_end:
    rclpy.spin_once(n, timeout_sec=0.05)
imu = np.array(n.imu)
gyro = np.array(n.px4_gyro)
print(f'  /livox/imu {len(imu)} msgs, PX4 sensor_combined {len(gyro)}, attitude {len(n.att)}, lidar frames {len(n.clouds)}')
out = []

if a.mode == 'rest':
    if len(imu) < 20 or not n.att:
        sys.exit('need /livox/imu and /fmu/out/vehicle_attitude')
    f_s = imu[:, 4:7].mean(axis=0)
    up_s = f_s / np.linalg.norm(f_s)                      # specific force at rest points up (sensor frame)
    # PX4: q = FRD body -> NED; up in FLU body = FLU <- FRD <- NED (0, 0, -1)
    q = np.mean(n.att, axis=0)
    q /= np.linalg.norm(q)
    up_b = FRD_TO_FLU @ (quat_wxyz_to_R(q).T @ np.array([0.0, 0.0, -1.0]))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_b[2]))))
    yaw = math.radians(a.yaw)
    v = rpy_to_R(0.0, 0.0, yaw).T @ up_b                   # Ry(p) Rx(r) up_s = Rz(-yaw) up_b
    def fit(r0, p0, half, step):                           # Ry(p) Rx(r) up_s against v on a grid
        rr, pp = np.meshgrid(np.radians(np.arange(r0 - half, r0 + half + 1e-9, step)),
                             np.radians(np.arange(p0 - half, p0 + half + 1e-9, step)))
        x, y, z = up_s
        y1, z1 = y * np.cos(rr) - z * np.sin(rr), y * np.sin(rr) + z * np.cos(rr)
        c = (x * np.cos(pp) + z1 * np.sin(pp)) * v[0] + y1 * v[1] + (-x * np.sin(pp) + z1 * np.cos(pp)) * v[2]
        i = np.unravel_index(np.argmax(c), c.shape)
        return float(c[i]), float(rr[i]), float(pp[i])
    c0, r, p = fit(0.0, 0.0, 15.0, 0.5)
    best = fit(math.degrees(r), math.degrees(p), 0.6, 0.01)
    _, r, p = best
    print(f'  vehicle tilt on the ground (PX4) {tilt:.2f} deg; Mid-360 IMU |f| {np.linalg.norm(f_s):.3f} (g units)')
    print(f'  mount roll {math.degrees(r):+.2f} deg, pitch {math.degrees(p):+.2f} deg (yaw held at {a.yaw:.1f}), '
          f'residual {math.degrees(math.acos(min(1.0, best[0]))):.2f} deg')
    out += [f'NOCLIMB_LIDAR_ROLL_DEG={math.degrees(r):.2f}', f'NOCLIMB_LIDAR_PITCH_DEG={math.degrees(p):.2f}']
    if n.clouds:
        pts = np.concatenate(n.clouds)
        rng = np.linalg.norm(pts, axis=1)
        cand = pts[(rng > 2.5) & (rng < 20.0) & (pts @ up_s < -0.05)]
        rs = np.random.default_rng(0)
        best_pl = None
        for _ in range(300 if len(cand) > 50 else 0):
            s = cand[rs.choice(len(cand), 3, replace=False)]
            nn = np.cross(s[1] - s[0], s[2] - s[0])
            if np.linalg.norm(nn) < 1e-6:
                continue
            nn /= np.linalg.norm(nn)
            if nn @ up_s < 0:
                nn = -nn
            if nn @ up_s < math.cos(math.radians(20)):
                continue
            dist = (cand - s[0]) @ nn
            inl = np.abs(dist) < 0.05
            if best_pl is None or inl.sum() > best_pl[0]:
                best_pl = (inl.sum(), nn, s[0])
        if best_pl and best_pl[0] > 100:
            _, nn, p0 = best_pl
            inl = np.abs((cand - p0) @ nn) < 0.05
            c0 = cand[inl].mean(axis=0)
            u, sv, vt = np.linalg.svd(cand[inl] - c0, full_matrices=False)
            nn = vt[2] if vt[2] @ up_s > 0 else -vt[2]
            h = float(-(c0 @ nn))                           # sensor origin above the plane
            gtilt = math.degrees(math.acos(min(1.0, nn @ up_s)))
            print(f'  ground plane: {inl.sum()} returns, lidar {h:.3f} m above it, plane vs IMU gravity {gtilt:.2f} deg'
                  f'{"  (ground not level / not the vehicle plane - trust the IMU)" if gtilt > 1.0 else ""}')
            out.append(f'NOCLIMB_LIDAR_Z={h - a.centre_height:.3f}   # lidar {h:.3f} m up - centre {a.centre_height:.2f} m')
        else:
            print('  ground plane: not found (open ground 2.5-20 m around? the Mid-360 sees down only to -7 deg)')

elif a.mode == 'shake':
    if len(imu) < 200 or len(gyro) < 200:
        sys.exit('need /livox/imu and /fmu/out/sensor_combined for the whole shake')
    t0 = max(imu[0, 0], gyro[0, 0]) + 0.5
    t1 = min(imu[-1, 0], gyro[-1, 0]) - 0.5
    dt = 0.005
    tg = np.arange(t0, t1, dt)
    wl = np.linalg.norm(imu[:, 1:4], axis=1)
    wp = np.linalg.norm(gyro[:, 1:4], axis=1)
    sl = np.interp(tg, imu[:, 0], wl)
    sp = np.interp(tg, gyro[:, 0], wp)
    sl -= sl.mean()
    sp -= sp.mean()
    if sp.std() < 0.05:
        sys.exit(f'too little motion (PX4 gyro rate std {sp.std():.3f} rad/s) - rock it harder')
    lags = np.arange(-int(0.25 / dt), int(0.25 / dt) + 1)
    cc = [np.dot(sl[max(0, k):len(sl) + min(0, k)], sp[max(0, -k):len(sp) - max(0, k)]) for k in lags]
    k = lags[int(np.argmax(cc))]
    lag = k * dt                                           # lidar IMU sample i ~ PX4 sample i - k: lidar later by lag
    peak = max(cc) / (np.linalg.norm(sl) * np.linalg.norm(sp))
    print(f'  rates: lidar IMU arrives {lag * 1e3:+.0f} ms after PX4 (correlation {peak:.2f})')
    # rotation: lidar gyro vectors at t + lag against PX4's at t
    ws = np.column_stack([np.interp(tg + lag, imu[:, 0], imu[:, i]) for i in (1, 2, 3)])
    wb = np.column_stack([np.interp(tg, gyro[:, 0], gyro[:, i]) for i in (1, 2, 3)])
    keep = (tg + lag < t1)
    H = ws[keep].T @ wb[keep]
    U, S, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    r, p, y = R_to_rpy(R)
    res = np.degrees(np.median(np.arccos(np.clip(np.einsum('ij,ij->i', (R @ ws[keep].T).T, wb[keep]) /
                                                     (np.linalg.norm(ws[keep], axis=1) * np.linalg.norm(wb[keep], axis=1) + 1e-9), -1, 1))))
    cond = S / S[0]
    print(f'  rotation lidar -> body: roll {math.degrees(r):+.2f}, pitch {math.degrees(p):+.2f}, yaw {math.degrees(y):+.2f} deg; '
          f'median vector error {res:.1f} deg; excitation per axis {cond[0]:.2f} / {cond[1]:.2f} / {cond[2]:.2f}'
          f'{"  <- rock it about every axis (too low)" if cond[2] < 0.1 else ""}')
    out += [f'NOCLIMB_LIDAR_LATENCY={max(0.0, lag):.3f}',
            f'NOCLIMB_LIDAR_YAW_DEG={math.degrees(y) % 360.0:.1f}',
            f'NOCLIMB_LIDAR_ROLL_DEG={math.degrees(r):.2f}', f'NOCLIMB_LIDAR_PITCH_DEG={math.degrees(p):.2f}']

else:   # target
    if not n.clouds:
        sys.exit(f'no frames on {a.lidar}')
    R = rpy_to_R(math.radians(a.roll), math.radians(a.pitch), math.radians(a.yaw))
    pts = (R @ np.concatenate(n.clouds).T).T                  # body FLU, lidar origin
    rng = np.hypot(pts[:, 0], pts[:, 1])
    brg = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
    front = (np.abs(brg) < 40) & (rng > 1.5) & (rng < 8.0) & (pts[:, 2] > -0.3) & (pts[:, 2] < 1.5)
    if front.sum() < 20:
        sys.exit('nothing in front (+-40 deg, 1.5-8 m)')
    f = pts[front]
    d0 = np.hypot(f[:, 0], f[:, 1]).min()
    near = np.hypot(f[:, 0], f[:, 1]) < d0 + 0.4
    c = f[near]
    b = math.degrees(math.atan2(c[:, 1].mean(), c[:, 0].mean()))
    print(f'  nearest object ahead: {near.sum()} returns at {np.hypot(c[:, 0].mean(), c[:, 1].mean()):.2f} m, bearing {b:+.2f} deg '
          f'(+ = left) with mount yaw {a.yaw:.1f}')
    out.append(f'NOCLIMB_LIDAR_YAW_DEG={(a.yaw - b) % 360.0:.1f}   # if that object is exactly on the nose axis')

print('\n'.join(['', 'put on the onboard_real.sh command line:'] + ['  ' + o for o in out]))
n.destroy_node()
rclpy.try_shutdown()
