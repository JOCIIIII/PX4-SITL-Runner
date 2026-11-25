#!/usr/bin/env python3
"""Mission driver of a ground (UGV) / surface (USV) vehicle - the counterpart of flight_driver's waypoint
mission for a vehicle that does not take off. PathFollowing (vehicle_type 3 / 4) follows the waypoints and
publishes /pf_ground_cmd (speed + bearing); this node arms PX4, switches to offboard and relays the command
as PX4's offboard velocity setpoint (rover_differential: |velocity| = speed setpoint, direction = bearing).

    WAIT    PX4 estimate valid + pre-arm checks passed, then --settle seconds on the spot
    PRIME   hold setpoints streaming, offboard, arm
    FOLLOW  the waypoints to PathFollowing (current position first, as flight_driver), relay its command;
            a command older than 0.5 s -> hold (speed 0 at the present yaw)
    DONE    PathFollowing reports the end of the path (or --timeout): hold 2 s, disarm, exit

Waypoints: CSV rows "north,east[,...]" [m]. --frame start (default): relative to the position at the start of
FOLLOW; --frame local: PX4 local frame; --origin N,E: the rows are world coordinates and the PX4 local origin
(where the estimator initialised = the spawn point) is at that world position.
Log: OUT/ground_driver.csv at 50 Hz (state, estimate, command, the guidance PathFollowing used: 0 pursuit,
1-2 MPPI), OUT/wp_local.csv (the waypoints sent, PX4 local frame)."""
import argparse
import csv
import math
import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray
from custom_msgs.msg import LocalWaypointSetpoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ground_common import Px4Link  # noqa: E402


class GroundDriver(Node):
    def __init__(self, a):
        super().__init__('ground_driver')
        self.a = a
        self.px4 = Px4Link(self)
        self.rows = []
        with open(a.csv) as f:
            for line in f:
                p = line.replace(',', ' ').split()
                if len(p) >= 2:
                    try:
                        self.rows.append((float(p[0]), float(p[1])))
                    except ValueError:
                        pass
        if len(self.rows) < 1:
            raise RuntimeError(f'no waypoints in {a.csv}')
        self.origin = tuple(float(v) for v in a.origin.split(',')) if a.origin else None

        self.pub_wp = self.create_publisher(LocalWaypointSetpoint, '/local_waypoint_setpoint_to_PF', 10)
        self.pub_speed = self.create_publisher(Float32, '/desired_speed', 10)
        self.pub_hb = [self.create_publisher(Bool, t, 10) for t in
                       ('/controller_heartbeat', '/path_planning_heartbeat', '/collision_avoidance_heartbeat')]
        self.create_subscription(Float32MultiArray, '/pf_ground_cmd', self._cmd, 10)
        self.create_subscription(Bool, '/path_following_complete', self._complete, 10)
        self.create_subscription(Bool, '/path_following_heartbeat', self._pf_hb, 10)

        self.cmd = None
        self.t_cmd = -1.0
        self.pf_alive = False
        self.pf_complete = False
        self.state = 'WAIT'
        self.t_state = self.now()
        self.t_ready = None
        self.t_follow = None
        self.hold_bearing = 0.0
        self.wps = None
        self.t_wp_sent = -1.0
        self.done = False
        self.result = 'none'
        self.ticks = 0
        os.makedirs(a.out, exist_ok=True)
        self.f = open(os.path.join(a.out, 'ground_driver.csv'), 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'state', 'x', 'y', 'vx', 'vy', 'speed_fwd', 'yaw', 'yaw_rate', 'speed_cmd', 'bearing_cmd',
                         'yaw_rate_cmd', 'acc_t', 'acc_n', 'align', 'guid_used', 'armed', 'offboard', 'px4_us'])
        self.create_timer(0.02, self.tick)
        self.create_timer(0.2, self.slow)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _cmd(self, m):
        if len(m.data) >= 7:
            self.cmd = [float(v) for v in m.data[:7]]
            self.t_cmd = self.now()

    def _complete(self, m):
        if m.data and self.state == 'FOLLOW':
            self.pf_complete = True

    def _pf_hb(self, m):
        self.pf_alive = self.pf_alive or bool(m.data)

    def set_state(self, s):
        self.get_logger().info(f'{self.state} -> {s}')
        self.state, self.t_state = s, self.now()

    def slow(self):
        hb = Bool()
        hb.data = True
        for p in self.pub_hb:
            p.publish(hb)
        s = Float32()
        s.data = float(self.a.speed)
        self.pub_speed.publish(s)
        # the waypoints until PathFollowing answers with a command (it needs the heartbeats first)
        if self.state == 'FOLLOW' and self.wps is not None and self.cmd is None and self.now() - self.t_wp_sent > 1.0:
            m = LocalWaypointSetpoint()
            m.path_planning_complete = True
            m.waypoint_x = [w[0] for w in self.wps]
            m.waypoint_y = [w[1] for w in self.wps]
            m.waypoint_z = [0.0 for _ in self.wps]
            self.pub_wp.publish(m)
            self.t_wp_sent = self.now()

    def build_waypoints(self):
        p = self.px4.pos
        if self.a.frame == 'start':
            pts = [(p[0] + n, p[1] + e) for n, e in self.rows]
        elif self.origin is not None:
            pts = [(n - self.origin[0], e - self.origin[1]) for n, e in self.rows]
        else:
            pts = list(self.rows)
        if math.hypot(pts[0][0] - p[0], pts[0][1] - p[1]) > 0.3:
            pts.insert(0, (p[0], p[1]))
        with open(os.path.join(self.a.out, 'wp_local.csv'), 'w') as f:
            for n, e in pts:
                f.write(f'{n:.3f},{e:.3f}\n')
        return pts

    def tick(self):
        p = self.px4
        t = self.now()
        self.ticks += 1
        speed_cmd, bearing_cmd = 0.0, self.hold_bearing
        if self.state == 'WAIT':
            if p.ready() and p.preflight_ok and self.pf_alive and self.t_ready is None:
                self.t_ready = t
            if self.t_ready is not None and t - self.t_ready > self.a.settle:
                self.hold_bearing = p.yaw
                self.set_state('PRIME')
            elif self.ticks % 250 == 0:
                self.get_logger().info(f'waiting: estimate {p.ready()} pre-arm {p.preflight_ok} PathFollowing {self.pf_alive}')
            if t - self.t_state > 120.0:
                self.finish('not_ready')
            return
        if self.state == 'PRIME':
            p.hold(self.hold_bearing)
            if t - self.t_state > 1.0:
                if not p.offboard:
                    p.set_offboard()
                elif not p.armed:
                    p.arm()
                else:
                    self.wps = self.build_waypoints()
                    self.t_follow = t
                    self.set_state('FOLLOW')
            if t - self.t_state > 30.0:
                self.finish('no_arm')
            return
        if self.state == 'FOLLOW':
            if self.cmd is not None and t - self.t_cmd < 0.5:
                speed_cmd, bearing_cmd = self.cmd[0], self.cmd[1]
                self.hold_bearing = p.yaw
                p.send_velocity(speed_cmd, bearing_cmd)
            else:
                p.hold(self.hold_bearing)
            if self.pf_complete:
                self.result = 'complete'
                self.hold_bearing = p.yaw
                self.set_state('DONE')
            elif t - self.t_follow > self.a.timeout:
                self.result = 'timeout'
                self.hold_bearing = p.yaw
                self.set_state('DONE')
            elif not p.armed or not p.offboard:
                self.result = 'px4_left_offboard'
                self.set_state('DONE')
        elif self.state == 'DONE':
            p.hold(self.hold_bearing)
            if t - self.t_state > 2.0:
                p.disarm()
            if t - self.t_state > 4.0:
                self.finish(self.result)
                return
        c = self.cmd if (self.cmd is not None and self.state == 'FOLLOW') else [0.0] * 7
        self.w.writerow([f'{t - (self.t_follow or t):.3f}', self.state, f'{p.pos[0]:.3f}', f'{p.pos[1]:.3f}',
                         f'{p.vel[0]:.3f}', f'{p.vel[1]:.3f}', f'{p.forward_speed():.3f}', f'{p.yaw:.4f}',
                         f'{p.yaw_rate:.4f}', f'{speed_cmd:.3f}', f'{bearing_cmd:.4f}', f'{c[2]:.4f}', f'{c[3]:.3f}',
                         f'{c[4]:.3f}', int(c[5]), int(c[6]), int(p.armed), int(p.offboard), p.t_pos_us])

    def finish(self, result):
        self.result = result
        self.get_logger().info(f'result: {result}')
        with open(os.path.join(self.a.out, 'ground_driver_result.txt'), 'w') as f:
            f.write(result + '\n')
        self.f.close()
        self.done = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', required=True)
    ap.add_argument('--speed', type=float, default=1.5)
    ap.add_argument('--frame', choices=('start', 'local'), default='start')
    ap.add_argument('--origin', default='', help='N,E: world position of the PX4 local origin (--frame local)')
    ap.add_argument('--settle', type=float, default=8.0)
    ap.add_argument('--timeout', type=float, default=300.0)
    ap.add_argument('--out', default='/ground-run/out')
    a = ap.parse_args()
    rclpy.init()
    n = GroundDriver(a)
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
