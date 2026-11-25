#!/usr/bin/env python3
"""Step 1 of the UGV/USV port: the vehicle alone under PX4 offboard velocity control (no PathFollowing).
Arms, switches to offboard and commands a sequence of (speed, bearing change) steps; logs the responses the
PathFollowing output stage and the MPPI rollout plant are sized from (speed lag, yaw-rate limit, yaw lag,
spot-turn behaviour, stopping).
    ground_step_test.py --out /ground-run/out/step.csv --steps "1.0:0:8,1.0:15:8,1.0:60:10,2.0:0:8,0:0:6"
steps: speed[m/s]:bearing change[deg, relative to the previous bearing]:duration[s]"""
import argparse
import csv
import math
import sys

import rclpy
from rclpy.node import Node

sys.path.insert(0, '/ground_scripts')
from ground_common import Px4Link, wrap  # noqa: E402


class StepTest(Node):
    def __init__(self, a):
        super().__init__('ground_step_test')
        self.a = a
        self.px4 = Px4Link(self)
        self.steps = [tuple(float(v) for v in s.split(':')) for s in a.steps.split(',')]
        self.f = open(a.out, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'phase', 'x', 'y', 'vx', 'vy', 'speed_fwd', 'yaw', 'yaw_rate', 'speed_cmd', 'bearing_cmd',
                         'nav_state', 'armed', 'offboard'])
        self.state = 'WAIT'
        self.t0 = None
        self.t_ready = None
        self.t_state = 0.0
        self.i = -1
        self.bearing = 0.0
        self.speed_cmd = 0.0
        self.ticks = 0
        self.done = False
        self.create_timer(0.02, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        p = self.px4
        t = self.now()
        self.ticks += 1
        if self.state == 'WAIT':
            if p.ready() and p.preflight_ok and self.t_ready is None:
                self.t_ready = t
            if self.t_ready is not None and t - self.t_ready > self.a.settle:
                self.bearing = p.yaw
                self.state, self.t_state = 'PRIME', t
                self.get_logger().info(f'ready: yaw {math.degrees(p.yaw):.1f} deg, pos {p.pos[0]:.2f} {p.pos[1]:.2f}')
            elif self.ticks % 250 == 0:
                self.get_logger().info(f'waiting: pos {p.pos is not None} yaw {p.yaw is not None} xy_valid {p.xy_valid} '
                                       f'preflight {p.preflight_ok}')
            return
        if self.state == 'PRIME':
            p.hold(self.bearing)
            if t - self.t_state > 1.0:
                if not p.offboard:
                    p.set_offboard()
                elif not p.armed:
                    p.arm()
                else:
                    self.state, self.t_state, self.t0 = 'RUN', t, t
                    self.i = 0
                    self.speed_cmd = self.steps[0][0]
                    self.bearing = wrap(self.bearing + math.radians(self.steps[0][1]))
                    self.get_logger().info('armed + offboard: steps start')
                if t - self.t_state > 30.0:
                    self.get_logger().error(f'no arm/offboard after 30 s (nav {p.nav_state}, armed {p.armed})')
                    self.finish()
            return
        if self.state == 'RUN':
            if t - self.t_state > self.steps[self.i][2]:
                self.i += 1
                self.t_state = t
                if self.i >= len(self.steps):
                    self.state = 'END'
                    return
                self.speed_cmd = self.steps[self.i][0]
                self.bearing = wrap(self.bearing + math.radians(self.steps[self.i][1]))
                self.get_logger().info(f'step {self.i}: speed {self.speed_cmd} bearing {math.degrees(self.bearing):.1f}')
            p.send_velocity(self.speed_cmd, self.bearing)
            self.w.writerow([f'{t - self.t0:.3f}', self.i, f'{p.pos[0]:.3f}', f'{p.pos[1]:.3f}', f'{p.vel[0]:.3f}',
                             f'{p.vel[1]:.3f}', f'{p.forward_speed():.3f}', f'{p.yaw:.4f}', f'{p.yaw_rate:.4f}',
                             f'{self.speed_cmd:.3f}', f'{self.bearing:.4f}', p.nav_state, int(p.armed), int(p.offboard)])
            return
        if self.state == 'END':
            p.hold(self.bearing)
            p.disarm()
            if not p.armed or t - self.t_state > 5.0:
                self.finish()

    def finish(self):
        self.f.close()
        self.done = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/ground-run/out/step.csv')
    ap.add_argument('--settle', type=float, default=8.0, help='[s] on the ground after PX4 is ready, before arming')
    ap.add_argument('--steps', default='1.0:0:8,1.0:15:8,1.0:60:10,2.0:0:8,0:0:6')
    a = ap.parse_args()
    rclpy.init()
    n = StepTest(a)
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
