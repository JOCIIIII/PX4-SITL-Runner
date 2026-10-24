#!/usr/bin/env python3
"""Closed-loop response test of vehicle + PathFollowing in the policy's command format.

Runs the test schedule of the CA training-dynamics note (dynamics.md 4.4) through the same
interface the noclimb CA node uses: every 50 ms a waypoint 3 m ahead of the vehicle in the
commanded heading at the commanded altitude, sent to PathFollowing as [P0, WP, WP + 2 (WP - P0), WP + 4 (WP - P0)]
plus /desired_speed and the heartbeats PF gates on. flight_manager arms, takes off and forwards
PF's attitude; the node starts after its hover phase and lands it at the end. Each segment
change is logged (logs/x8_step_test.csv) for scripts/dev/x8_step_analysis.py.

The command shaping of the noclimb CA node can be applied to the stepped command, so the test
measures the whole chain the policy drives: path_extension (k of [P0, WP, WP + k(WP-P0),
WP + 2k(WP-P0)]), cmd_tau_s (first-order smoothing of the horizontal velocity command) and
cmd_brake_immediate (speed decreases bypass the smoothing). Defaults = no shaping, k 2.

Schedule (heading h0 = east, altitude H = altitude at the start):
  hover 4 s | 3 m/s 5 s | 1 m/s 4 s | 0 m/s 4 s | 3 m/s 4 s | turn +90 deg, 3 m/s 5 s |
  0 m/s 4 s | altitude H+1 5 s | altitude H 6 s | 3 m/s, heading +-40 deg every 0.5 s 10 s |
  0 m/s 4 s | 5 m/s 5 s | reverse (+180 deg), 5 m/s 6 s | 0 m/s 4 s |
  3 m/s with altitude H+1 5 s | 3 m/s with altitude H 6 s | 0 m/s 4 s
"""
import csv
import math
import sys

import numpy as np
import rclpy
from custom_msgs.msg import LocalWaypointSetpoint
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, Float32

# (name, duration s, speed m/s, heading offset deg | 'zig', altitude offset m)
SCHEDULE = [
    ('hover', 4.0, 0.0, 0.0, 0.0),
    ('v0to3', 5.0, 3.0, 0.0, 0.0),
    ('v3to1', 4.0, 1.0, 0.0, 0.0),
    ('v1to0', 4.0, 0.0, 0.0, 0.0),
    ('v0to3b', 4.0, 3.0, 0.0, 0.0),
    ('turn90', 5.0, 3.0, 90.0, 0.0),
    ('stop', 4.0, 0.0, 90.0, 0.0),
    ('alt_up', 5.0, 0.0, 90.0, 1.0),
    ('alt_down', 6.0, 0.0, 90.0, 0.0),
    ('zigzag', 10.0, 3.0, 'zig', 0.0),
    ('stop2', 4.0, 0.0, 0.0, 0.0),
    ('v0to5', 5.0, 5.0, 0.0, 0.0),
    ('rev180', 6.0, 5.0, 180.0, 0.0),
    ('stop5', 4.0, 0.0, 180.0, 0.0),
    ('alt_up_mv', 5.0, 3.0, 180.0, 1.0),
    ('alt_down_mv', 6.0, 3.0, 180.0, 0.0),
    ('final', 4.0, 0.0, 180.0, 0.0),
]


class StepTest(Node):
    def __init__(self):
        super().__init__('x8_step_test')
        self.start_after_arm = float(self.declare_parameter('start_after_arm_s', 12.0).value)
        log_csv = self.declare_parameter('log_csv', '').value
        self.ext_factor = float(self.declare_parameter('path_extension', 2.0).value)
        self.cmd_tau = float(self.declare_parameter('cmd_tau_s', 0.0).value)
        self.brake_now = bool(self.declare_parameter('cmd_brake_immediate', False).value)
        self.v_s = np.zeros(2)
        self.pos = self.vel = None
        self.t_armed = None
        self.t0 = None
        self.done = False
        self.seg = -1
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.wp_pub = self.create_publisher(LocalWaypointSetpoint, '/local_waypoint_setpoint_to_PF', qos)
        self.speed_pub = self.create_publisher(Float32, '/desired_speed', 10)
        self.pp_hb = self.create_publisher(Bool, '/path_planning_heartbeat', 10)
        self.ca_hb = self.create_publisher(Bool, '/collision_avoidance_heartbeat', 10)
        self.done_pub = self.create_publisher(Bool, '/path_following_complete', 1)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.lp_cb, qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.st_cb, qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)
        self.create_timer(0.1, self.heartbeat)
        self.csv = None
        if log_csv:
            self.f = open(log_csv, 'w', buffering=1, newline='')
            self.csv = csv.writer(self.f)
            self.csv.writerow(['t', 'seg', 'speed_cmd', 'heading_cmd', 'alt_cmd', 'e', 'n', 'u', 've', 'vn', 'vu',
                               'speed_sent'])

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def lp_cb(self, m):                       # NED -> ENU
        self.pos = np.array([m.y, m.x, -m.z]); self.vel = np.array([m.vy, m.vx, -m.vz])

    def st_cb(self, m):
        if m.arming_state == VehicleStatus.ARMING_STATE_ARMED and self.t_armed is None:
            self.t_armed = self.now()

    def heartbeat(self):
        if self.t0 is None or self.done:
            return
        m = Bool(); m.data = True
        self.pp_hb.publish(m); self.ca_hb.publish(m)

    def tick(self):
        if self.pos is None or self.t_armed is None or self.done:
            return
        t = self.now()
        if self.t0 is None:
            if t - self.t_armed < self.start_after_arm:
                return
            self.t0, self.alt0 = t, self.pos[2]
            self.h0 = 0.0                      # east; the test is about responses, not direction
            self.get_logger().info(f'[x8_step_test] start at alt {self.alt0:.1f} m')
        tr = t - self.t0
        acc = 0.0
        for i, (name, dur, spd, hdg, dalt) in enumerate(SCHEDULE):
            if tr < acc + dur:
                break
            acc += dur
        else:
            self.done = True
            m = Bool(); m.data = True
            for _ in range(5):
                self.done_pub.publish(m)
            self.get_logger().info('[x8_step_test] done - landing')
            return
        if i != self.seg:
            self.seg = i
            self.get_logger().info(f'[x8_step_test] segment {name} at t+{tr:.1f}s')
        if hdg == 'zig':
            k = int((tr - acc) / 0.5)
            h = self.h0 + math.radians(40.0 if k % 2 == 0 else -40.0)
        else:
            h = self.h0 + math.radians(hdg)
        alt = self.alt0 + dalt
        u = np.array([math.cos(h), math.sin(h)])
        sent = float(spd)
        if self.cmd_tau > 0.0:                 # as noclimb_ca_node
            self.v_s = self.v_s + 0.05 / (self.cmd_tau + 0.05) * (spd * u - self.v_s)
            if self.brake_now and float(np.linalg.norm(self.v_s)) > spd:
                self.v_s = self.v_s * (spd / float(np.linalg.norm(self.v_s)))
            sent = float(np.linalg.norm(self.v_s))
            if sent > 0.05:
                u = self.v_s / sent
        wp = np.array([self.pos[0] + 3.0 * u[0], self.pos[1] + 3.0 * u[1], alt])
        p0 = np.array([self.pos[0], self.pos[1], alt])
        ext = wp + self.ext_factor * np.r_[wp[:2] - p0[:2], 0.0]
        ext2 = wp + 2.0 * self.ext_factor * np.r_[wp[:2] - p0[:2], 0.0]    # as noclimb_ca_node
        m = Float32(); m.data = sent
        self.speed_pub.publish(m)
        out = LocalWaypointSetpoint()
        out.path_planning_complete = True
        out.waypoint_x = [float(q[1]) for q in (p0, wp, ext, ext2)]
        out.waypoint_y = [float(q[0]) for q in (p0, wp, ext, ext2)]
        out.waypoint_z = [float(q[2]) for q in (p0, wp, ext, ext2)]
        self.wp_pub.publish(out)
        if self.csv:
            self.csv.writerow([f'{t:.3f}', name, spd, f'{math.degrees(h):.1f}', f'{alt:.3f}',
                               *[f'{x:.3f}' for x in (*self.pos, *self.vel)], f'{sent:.3f}'])


def main():
    rclpy.init(args=sys.argv)
    n = StepTest()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
