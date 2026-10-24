#!/usr/bin/env python3
"""Nose along the commanded velocity for T1000, without touching what its policy sees.

T1000's policy reads the lidar bins, velocity and goal in the body frame and was trained with
the nose on the path; turning the real nose with the command made it chase its own heading
(x8_t1000_d: nose to -140 deg, flew 120 m the wrong way). So the X8 version splits the two:
t1000_bridge.py (virtual_heading:=path) hands T1000 a pose and bins with the yaw of the path
segment, and this relay takes T1000's setpoints (its TOPIC_PX4_SETPOINT moved to
/t1000/trajectory_setpoint_raw) and forwards them to PX4 with an absolute yaw setpoint toward
the horizontal velocity command, rate-limited. The AirSim lidar covers 360 deg, so the real
heading does not change what is sensed.

  /t1000/trajectory_setpoint_raw (TrajectorySetpoint, NED velocity + yawspeed from T1000)
  + /fmu/out/vehicle_attitude    (real yaw, to start from)
  -> /fmu/in/trajectory_setpoint (velocity [safety-filtered], yaw = its direction, yawspeed NaN)

Safety velocity filter (safety_radius_m > 0): per 10 deg sector the horizontal velocity toward
the closest return may be at most the speed that still stops before the radius with
safety_brake_accel after safety_delay_s, v*delay + v^2/(2a) <= distance - radius (inside the
radius: back away at safety_gain * (radius - distance)), over the world-registered returns
(/cloud, /odom) of the last safety_memory_s within the vehicle's height band.
  - the memory matters: the lidar sits 0.3 m above the centre with a -7 deg lower limit, so a
    branch at flight height drops out of the scan inside ~1.6 m - a filter on T1000's current
    bins never saw it (x8_t1000_e: 0.44 m);
  - the braking law matters: the linear limit gain * (distance - radius) of noclimb_ca_node.py
    let the X8 come in at 3 m/s and stop too late (x8_t1000_g: 0.07 m).
"""
import math
from collections import deque

import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

import rclpy
from px4_msgs.msg import TrajectorySetpoint, VehicleAttitude
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)    # as T1000's commander


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class YawRelay(Node):
    def __init__(self):
        super().__init__('t1000_yaw_relay')
        self.rate = math.radians(float(self.declare_parameter('yaw_rate_limit_deg', 60.0).value))
        self.min_speed = float(self.declare_parameter('min_speed', 0.3).value)
        self.safe_r = float(self.declare_parameter('safety_radius_m', 0.0).value)
        self.safe_k = float(self.declare_parameter('safety_gain', 2.0).value)
        self.safe_mem = float(self.declare_parameter('safety_memory_s', 3.0).value)
        self.safe_a = float(self.declare_parameter('safety_brake_accel', 2.0).value)
        self.safe_tau = float(self.declare_parameter('safety_delay_s', 0.4).value)
        self.safe_dz = (float(self.declare_parameter('safety_dz_below_m', 0.5).value),
                        float(self.declare_parameter('safety_dz_above_m', 0.6).value))
        self.scans = deque()
        self.pos = None
        self.n_cut = 0
        if self.safe_r > 0.0:
            self.create_subscription(Odometry, '/odom', self.odom_cb, qos_profile_sensor_data)
            self.create_subscription(PointCloud2, '/cloud', self.cloud_cb, qos_profile_sensor_data)
        self.yaw_now = None
        self.yaw_sp = None
        self.t_last = None
        self.n = 0
        self.pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.create_subscription(TrajectorySetpoint, '/t1000/trajectory_setpoint_raw', self.sp_cb,
                                 QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                            history=HistoryPolicy.KEEP_LAST, depth=1))
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.att_cb,
                                 qos_profile_sensor_data)
        self.get_logger().info(f'[t1000_yaw_relay] nose along the command above {self.min_speed} m/s, '
                               f'<= {math.degrees(self.rate):.0f} deg/s; safety filter {self.safe_r} m, '
                               f'brake {self.safe_a} m/s^2 after {self.safe_tau} s, memory {self.safe_mem} s')

    def odom_cb(self, m):
        p = m.pose.pose.position
        self.pos = np.array([p.x, p.y, p.z])               # ENU

    def cloud_cb(self, m):
        if self.pos is None:
            return
        xyz = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        rel = xyz - self.pos
        xyz = xyz[np.einsum('ij,ij->i', rel, rel) <= (self.safe_r + 5.0) ** 2]
        now = self.get_clock().now().nanoseconds * 1e-9
        self.scans.append((now, xyz))
        while self.scans and now - self.scans[0][0] > self.safe_mem:
            self.scans.popleft()

    def safety_filter(self, v):
        """closest point to v (ENU, 2D) of {u : u.n_i <= k (d_i - r)}, one constraint per 10 deg
        sector (its closest return), by repeated projection onto the most violated one"""
        if not self.scans or self.pos is None:
            return v, float('inf')
        rel = np.concatenate([x for _, x in self.scans]) - self.pos
        rel = rel[(rel[:, 2] > -self.safe_dz[0]) & (rel[:, 2] < self.safe_dz[1]), :2]
        d = np.hypot(rel[:, 0], rel[:, 1])
        m = d < self.safe_r + 4.0                 # stopping from 3 m/s needs 3.45 m beyond the radius
        rel, d = rel[m], d[m]
        if len(d) == 0:
            return v, float('inf')
        sector = ((np.arctan2(rel[:, 1], rel[:, 0]) + math.pi) / (2.0 * math.pi) * 36.0).astype(int) % 36
        order = np.lexsort((d, sector))
        first = order[np.r_[True, sector[order][1:] != sector[order][:-1]]]
        nrm = rel[first] / np.maximum(d[first], 1e-3)[:, None]
        gap = d[first] - self.safe_r
        a, tau = self.safe_a, self.safe_tau
        lim = np.where(gap > 0.0, a * (np.sqrt(tau * tau + 2.0 * np.maximum(gap, 0.0) / a) - tau),
                       self.safe_k * gap)
        u = np.asarray(v, dtype=float).copy()
        for _ in range(30):
            viol = nrm @ u - lim
            i = int(np.argmax(viol))
            if viol[i] <= 1e-3:
                break
            u = u - viol[i] * nrm[i]
        return u, float(d.min())

    def att_cb(self, m):
        w, x, y, z = (float(v) for v in m.q)
        self.yaw_now = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))   # NED

    def sp_cb(self, m):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.yaw_sp is None:
            if self.yaw_now is None:
                self.pub.publish(m)                     # no attitude yet: pass through
                return
            self.yaw_sp = self.yaw_now
        vx, vy = float(m.velocity[0]), float(m.velocity[1])           # NED
        if self.safe_r > 0.0 and math.isfinite(vx) and math.isfinite(vy):
            u, dmin = self.safety_filter(np.array([vy, vx]))           # ENU = (east, north)
            if abs(u[0] - vy) + abs(u[1] - vx) > 1e-3:
                self.n_cut += 1
                if self.n_cut % 20 == 1:
                    self.get_logger().info(f'[t1000_yaw_relay] safety: {math.hypot(vx, vy):.2f} -> '
                                           f'{math.hypot(*u):.2f} m/s (nearest {dmin:.2f} m)')
            vx, vy = float(u[1]), float(u[0])
            m.velocity = [vx, vy, float(m.velocity[2])]
        target = math.atan2(vy, vx) if math.hypot(vx, vy) > self.min_speed else self.yaw_sp
        dt = 0.05 if self.t_last is None else min(max(now - self.t_last, 0.0), 0.2)
        self.t_last = now
        step = self.rate * dt
        self.yaw_sp = wrap(self.yaw_sp + max(min(wrap(target - self.yaw_sp), step), -step))
        m.yaw = float(self.yaw_sp)
        m.yawspeed = float('nan')
        self.pub.publish(m)
        self.n += 1
        if self.n == 1:
            self.get_logger().info('[t1000_yaw_relay] relaying T1000 setpoints')


def main():
    rclpy.init()
    n = YawRelay()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
