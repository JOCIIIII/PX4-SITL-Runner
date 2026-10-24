#!/usr/bin/env python3
"""T1000's velocity command flown by PathFollowing (PF) instead of PX4's velocity loop.

T1000 runs in its dry-run mode (t1000_pf.sh): it computes its full command (policy residual on its
path-following reference, avoidance phases, 1.5 m/s^2 rate limit of --corner-mode rounded, 0.18 s
low-pass) but sends nothing to PX4 - only /t1000/cmd_preview [vx, vy, vz, yawspeed] (NED). This
node turns that command into what PF takes from the in-house CA node (noclimb_ca_node.py), so
flight_driver + PF fly it exactly as they fly the in-house policy:
  direction of the horizontal command -> a waypoint lookahead_m ahead at the path height
  speed of the command                -> /desired_speed (capped for the goal approach, below)
  path [P0, WP, WP + k(WP-P0), WP + 2k(WP-P0)] (k = path_extension; see noclimb_ca.sh)
T1000's vz and yaw rate are dropped: PF holds the waypoint height and turns the nose; the bridge
hands T1000 the path's yaw (virtual_heading:=path), so the real nose does not reach its inputs.

Safety velocity filter (safety_radius_m > 0): the brake-law filter of t1000_yaw_relay.py (per 10 deg
sector v*delay + v^2/(2a) <= distance - radius, scans of the last safety_memory_s) on the command
before it becomes a waypoint - the X8 T1000 setup flew with it.

Goal: the flight driver's (/flight_driver/goal, ENU). Within goal_handover_m flight_driver stops
following PF and lands at the goal with PX4 position control; the speed is capped on the way
there as in noclimb_ca_node.py (sqrt(v_h^2 + 2 a d), v_h = 1 m/s at the handover radius).

T1000's path (/t1000/path, 2 Hz - its path-age failsafe holds after 1 s): a straight line from where the
vehicle is when the driver's goal arrives to the goal, in the same world ENU frame as /odom (on the vehicle PX4's
local origin is not the take-off point, so a fixed path cannot be given up front).

  in : /t1000/cmd_preview, /odom, /cloud, /fmu/out/vehicle_status_v1, /flight_driver/goal
  out: /local_waypoint_setpoint_to_PF, /desired_speed, /path_planning_heartbeat, /t1000/path,
       /collision_avoidance_heartbeat, /noclimb/ready (flight_driver arms only on it), log_csv
"""
import math
from collections import deque

import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from custom_msgs.msg import LocalWaypointSetpoint
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from px4_msgs.msg import VehicleStatus
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, Float32MultiArray

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data


class T1000PF(Node):
    def __init__(self):
        super().__init__('t1000_pf')
        p = lambda n, d: self.declare_parameter(n, d).value
        self.lookahead = float(p('lookahead_m', 3.0))
        self.path_ext = float(p('path_extension', 5.0))
        self.min_speed = float(p('min_speed', 0.1))       # below: keep the last direction
        self.handover = float(p('goal_handover_m', 3.0))
        self.handover_speed = float(p('handover_speed', 1.0))
        self.goal_brake = float(p('goal_brake_accel', 1.5))
        self.preview_timeout = float(p('preview_timeout_s', 0.5))
        self.safe_r = float(p('safety_radius_m', 0.0))
        self.safe_k = float(p('safety_gain', 2.0))
        self.safe_mem = float(p('safety_memory_s', 3.0))
        self.safe_a = float(p('safety_brake_accel', 2.0))
        self.safe_tau = float(p('safety_delay_s', 0.4))
        self.safe_dz = (float(p('safety_dz_below_m', 0.5)), float(p('safety_dz_above_m', 0.6)))
        self.log_path = str(p('log_csv', ''))
        self.path_frame = str(p('path_frame', 'map'))
        self.path_start = None

        self.pos = self.vel = None
        self.t_odom = self.t_cloud = self.t_cmd = None
        self.cmd = None
        self.scans = deque()
        self.goal = self.alt = None
        self.armed = False
        self.engaged = False
        self.done = False
        self.dir = None
        self.t0 = None
        self.log = open(self.log_path, 'w') if self.log_path else None
        if self.log:
            self.log.write('t,x,y,z,vx,vy,cmd_vx,cmd_vy,cmd_speed,safe_vx,safe_vy,safe_dmin,speed,wp_x,wp_y,wp_z,d_goal\n')

        wp_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.wp_pub = self.create_publisher(LocalWaypointSetpoint, '/local_waypoint_setpoint_to_PF', wp_qos)
        self.speed_pub = self.create_publisher(Float32, '/desired_speed', 10)
        self.pp_hb = self.create_publisher(Bool, '/path_planning_heartbeat', 10)
        self.ca_hb = self.create_publisher(Bool, '/collision_avoidance_heartbeat', 10)
        self.ready_pub = self.create_publisher(Bool, '/noclimb/ready', 10)
        self.path_pub = self.create_publisher(Path, '/t1000/path', 10)
        self.create_timer(0.5, self.publish_t1000_path)
        self.create_subscription(Float32MultiArray, '/t1000/cmd_preview', self.cmd_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/cloud', self.cloud_cb, qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, qos_profile_sensor_data)
        self.create_subscription(PointStamped, '/flight_driver/goal', self.goal_cb, 10)
        self.create_timer(1.0, self.ready)
        self.create_timer(0.1, self.heartbeat)
        self.get_logger().info(f'[t1000_pf] T1000 command -> PF: waypoint {self.lookahead} m ahead, path extension '
                               f'{self.path_ext}, safety filter {self.safe_r} m, goal handover {self.handover} m')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- inputs ----
    def odom_cb(self, m):
        q, v = m.pose.pose.position, m.twist.twist.linear
        self.pos = np.array([q.x, q.y, q.z])                 # ENU
        self.vel = np.array([v.x, v.y, v.z])
        self.t_odom = self.now()

    def cloud_cb(self, m):
        self.t_cloud = self.now()
        if self.pos is None or self.safe_r <= 0.0:
            return
        xyz = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        rel = xyz - self.pos
        xyz = xyz[np.einsum('ij,ij->i', rel, rel) <= (self.safe_r + 5.0) ** 2]
        self.scans.append((self.t_cloud, xyz))
        while self.scans and self.t_cloud - self.scans[0][0] > self.safe_mem:
            self.scans.popleft()

    def status_cb(self, m):
        armed = m.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if armed and not self.armed:
            self.get_logger().info('[t1000_pf] armed')
        if not armed and self.engaged and not self.done:
            self.done = True
            self.get_logger().info('[t1000_pf] disarmed - stopping')
            if self.log:
                self.log.flush()
        self.armed = armed

    def goal_cb(self, m):
        if self.engaged:
            return
        if self.goal is None:
            self.get_logger().info(f'[t1000_pf] goal from the flight driver ({m.point.x:.1f}, {m.point.y:.1f}) '
                                   f'at z {m.point.z:.2f}')
        self.goal = np.array([float(m.point.x), float(m.point.y)])
        self.alt = float(m.point.z)
        if self.path_start is None and self.pos is not None:
            self.path_start = self.pos[:2].copy()
            self.publish_t1000_path()

    def publish_t1000_path(self):
        if self.path_start is None or self.goal is None or self.done:
            return
        m = Path()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.path_frame
        for xy in (self.path_start, self.goal):
            ps = PoseStamped()
            ps.header = m.header
            ps.pose.position.x, ps.pose.position.y = float(xy[0]), float(xy[1])
            ps.pose.position.z = float(self.alt)
            ps.pose.orientation.w = 1.0
            m.poses.append(ps)
        self.path_pub.publish(m)

    # ---- the safety filter of t1000_yaw_relay.py ----
    def safety_filter(self, v):
        if not self.scans or self.pos is None:
            return v, float('inf')
        rel = np.concatenate([x for _, x in self.scans]) - self.pos
        rel = rel[(rel[:, 2] > -self.safe_dz[0]) & (rel[:, 2] < self.safe_dz[1]), :2]
        d = np.hypot(rel[:, 0], rel[:, 1])
        m = d < self.safe_r + 4.0
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

    # ---- outputs ----
    def ready(self):
        t = self.now()
        # pose and registered lidar live (T1000 computes only once it has its path, i.e. after the goal - the
        # engage waits for a fresh command instead)
        if self.done or self.pos is None or self.t_cloud is None:
            return
        if t - self.t_odom > 0.5 or t - self.t_cloud > 1.0:
            return
        self.ready_pub.publish(Bool(data=True))

    def heartbeat(self):
        if not self.engaged or self.done:
            return
        self.pp_hb.publish(Bool(data=True))
        self.ca_hb.publish(Bool(data=True))

    def goal_cap(self, d_goal):
        d = max(d_goal - self.handover, 0.0)
        return math.sqrt(self.handover_speed ** 2 + 2.0 * self.goal_brake * d)

    def cmd_cb(self, m):
        """one T1000 step (20 Hz): its command -> PF's path and speed"""
        self.t_cmd = t = self.now()
        if len(m.data) < 2 or self.done or self.pos is None:
            return
        vn, ve = float(m.data[0]), float(m.data[1])
        if not (math.isfinite(vn) and math.isfinite(ve)):
            return
        v = np.array([ve, vn])                                  # ENU
        if not self.engaged:
            if not self.armed or self.goal is None or self.path_start is None:
                return
            self.engaged = True
            self.t0 = t
            self.get_logger().info(f'[t1000_pf] engaged at ({self.pos[0]:.1f}, {self.pos[1]:.1f}, {self.pos[2]:.1f}), '
                                   f'{np.linalg.norm(self.goal - self.pos[:2]):.1f} m to the goal')
        vs, dmin = self.safety_filter(v) if self.safe_r > 0.0 else (v, float('inf'))
        d_goal = float(np.linalg.norm(self.goal - self.pos[:2]))
        sp = float(np.linalg.norm(vs))
        if sp > self.min_speed:
            self.dir = vs / sp
        elif self.dir is None:
            g = self.goal - self.pos[:2]
            self.dir = g / max(float(np.linalg.norm(g)), 1e-6)
        speed = min(sp, self.goal_cap(d_goal))
        p0 = np.r_[self.pos[:2], self.alt]
        wp = np.r_[self.pos[:2] + self.lookahead * self.dir, self.alt]
        step = wp - p0
        pts = [p0, wp, wp + self.path_ext * step, wp + 2.0 * self.path_ext * step]
        self.speed_pub.publish(Float32(data=float(speed)))       # speed first: PF's default is 1.5
        out = LocalWaypointSetpoint()
        out.path_planning_complete = True
        out.waypoint_x = [float(q[1]) for q in pts]             # North = ENU y
        out.waypoint_y = [float(q[0]) for q in pts]             # East  = ENU x
        out.waypoint_z = [float(q[2]) for q in pts]             # altitude (+)
        self.wp_pub.publish(out)
        if self.log:
            vel = self.vel if self.vel is not None else np.zeros(3)
            self.log.write(f'{t - self.t0:.3f},{self.pos[0]:.3f},{self.pos[1]:.3f},{self.pos[2]:.3f},'
                           f'{vel[0]:.3f},{vel[1]:.3f},{v[0]:.3f},{v[1]:.3f},{float(np.linalg.norm(v)):.3f},'
                           f'{vs[0]:.3f},{vs[1]:.3f},{min(dmin, 99.0):.2f},{speed:.3f},'
                           f'{wp[0]:.3f},{wp[1]:.3f},{wp[2]:.3f},{d_goal:.2f}\n')


def main():
    rclpy.init()
    node = T1000PF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node.log:
        node.log.close()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
