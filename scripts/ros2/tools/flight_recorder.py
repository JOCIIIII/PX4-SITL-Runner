#!/usr/bin/env python3
"""Flight recorder for SITL tuning runs (runs inside the ros2-env container).

Writes CSVs (receive-time stamped) into OUT_DIR:
  odom.csv         adapter /odom (ENU world)            t,x,y,z,vx,vy,vz
  px4.csv          /fmu/out/vehicle_local_position (NED) t,ts_us,x,y,z,vx,vy,vz
  status.csv       arming / nav state changes           t,arming,nav
  lidar.csv        raw AirSim lidar per message         t,stamp,npts,dup,minr
                   (minr = closest non-ground return, sensor frame, z > -1 m)
  airsim_odom.csv  AirSim odom_local_ned                t,stamp,x,y,z,qw,qx,qy,qz
  desired_speed.csv /desired_speed                      t,v
and on exit cloud_map.npz: voxel centres (0.2 m) of the world-frame /cloud
(real returns only, < 60 m from the vehicle) - used as the obstacle reference map -
plus t_first, the receive time each voxel was first seen (detection-latency checks), and
r_min, the closest vehicle distance it was ever observed from (registration error grows with
range, so clearance checks can keep only voxels confirmed up close).

usage: flight_recorder.py OUT_DIR [--no-map]
"""
import os
import signal
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
import sensor_msgs_py.point_cloud2 as pc2

try:
    from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
except ImportError:  # px4_msgs not sourced
    VehicleLocalPosition = VehicleStatus = None

VOX = 0.2


class Recorder(Node):
    def __init__(self, out, record_map=True):
        super().__init__('flight_recorder')
        os.makedirs(out, exist_ok=True)
        self.out = out
        self.f = {}
        for name, hdr in [('odom', 't,x,y,z,vx,vy,vz'),
                          ('px4', 't,ts_us,x,y,z,vx,vy,vz'),
                          ('status', 't,arming,nav'),
                          ('lidar', 't,stamp,npts,dup,minr'),
                          ('airsim_odom', 't,stamp,x,y,z,qw,qx,qy,qz'),
                          ('desired_speed', 't,v')]:
            self.f[name] = open(os.path.join(out, name + '.csv'), 'w', buffering=1)
            self.f[name].write(hdr + '\n')
        self.last_hash = None
        self.last_status = None
        self.pos = None
        self.record_map = record_map
        self.keys = []
        self.first_seen = {}
        self.r_min = {}
        self.cloud_n = 0
        q = qos_profile_sensor_data
        self.create_subscription(Odometry, '/odom', self.odom_cb, q)
        self.create_subscription(PointCloud2, '/airsim_node/SimpleFlight/lidar/points/RPLIDAR_A3', self.lidar_cb, q)
        self.create_subscription(Odometry, '/airsim_node/SimpleFlight/odom_local', self.aso_cb, q)
        self.create_subscription(Float32, '/desired_speed', self.ds_cb, 10)
        if record_map:
            self.create_subscription(PointCloud2, '/cloud', self.cloud_cb, q)
        if VehicleLocalPosition is not None:
            self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.px4_cb, q)
            self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, q)

    def odom_cb(self, m):
        p, v = m.pose.pose.position, m.twist.twist.linear
        self.pos = np.array([p.x, p.y, p.z])
        self.f['odom'].write(f'{time.time():.4f},{p.x:.3f},{p.y:.3f},{p.z:.3f},{v.x:.3f},{v.y:.3f},{v.z:.3f}\n')

    def px4_cb(self, m):
        self.f['px4'].write(f'{time.time():.4f},{m.timestamp},{m.x:.3f},{m.y:.3f},{m.z:.3f},{m.vx:.3f},{m.vy:.3f},{m.vz:.3f}\n')

    def status_cb(self, m):
        s = (int(m.arming_state), int(m.nav_state))
        if s != self.last_status:
            self.last_status = s
            self.f['status'].write(f'{time.time():.4f},{s[0]},{s[1]}\n')

    def ds_cb(self, m):
        self.f['desired_speed'].write(f'{time.time():.4f},{m.data:.3f}\n')

    def lidar_cb(self, m):
        t = time.time()
        h = hash(bytes(m.data[:8192]))
        dup = int(h == self.last_hash)
        self.last_hash = h
        minr = -1.0
        try:
            pts = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
            if pts.size:
                r = np.linalg.norm(pts, axis=1)
                sel = (pts[:, 2] > -1.0) & (r > 0.3)
                if sel.any():
                    minr = float(r[sel].min())
        except Exception:
            pass
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.f['lidar'].write(f'{t:.4f},{st:.4f},{m.width * m.height},{dup},{minr:.3f}\n')

    def aso_cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.f['airsim_odom'].write(f'{time.time():.4f},{st:.4f},{p.x:.3f},{p.y:.3f},{p.z:.3f},'
                                    f'{q.w:.5f},{q.x:.5f},{q.y:.5f},{q.z:.5f}\n')

    def cloud_cb(self, m):
        self.cloud_n += 1
        if self.cloud_n % 3 or self.pos is None:
            return
        pts = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
        if not pts.size:
            return
        d = np.linalg.norm(pts - self.pos, axis=1)
        # no-return points come out at the sensor origin; keep real returns only
        sel = (d > 1.0) & (d < 60.0)
        pts, d = pts[sel], d[sel]
        if not pts.size:
            return
        k = np.floor(pts / VOX).astype(np.int64) + (1 << 20)
        kk = (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]
        order = np.argsort(kk, kind='stable')
        kk, d = kk[order], d[order]
        uk, starts = np.unique(kk, return_index=True)
        dmin = np.minimum.reduceat(d, starts)
        self.keys.append(uk)
        now = time.time()
        for key, r in zip(uk.tolist(), dmin.tolist()):
            self.first_seen.setdefault(key, now)
            if r < self.r_min.get(key, 1e9):
                self.r_min[key] = r
        if len(self.keys) > 50:
            self.keys = [np.unique(np.concatenate(self.keys))]

    def finish(self):
        for fh in self.f.values():
            fh.close()
        if self.record_map and self.keys:
            k = np.unique(np.concatenate(self.keys))
            xyz = np.stack([(k >> 42) & 0x1FFFFF, (k >> 21) & 0x1FFFFF, k & 0x1FFFFF], 1) - (1 << 20)
            t_first = np.array([self.first_seen.get(int(x), np.nan) for x in k])
            r_min = np.array([self.r_min.get(int(x), np.nan) for x in k], dtype=np.float32)
            np.savez_compressed(os.path.join(self.out, 'cloud_map.npz'),
                                xyz=((xyz + 0.5) * VOX).astype(np.float32), t_first=t_first, r_min=r_min)


def main():
    out = sys.argv[1]
    rclpy.init()
    node = Recorder(out, record_map='--no-map' not in sys.argv)
    stop = {'v': False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__('v', True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__('v', True))
    try:
        while rclpy.ok() and not stop['v']:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.finish()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
