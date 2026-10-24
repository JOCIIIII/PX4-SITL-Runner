#!/usr/bin/env python3
"""Replay a recorded attitude-setpoint sequence on the SITL vehicle (X8 model validation).

flight_manager arms, climbs and, once its hover phase is over, forwards /pf_att_2_control
(attitude + thrust) to PX4. This node feeds it the vehicle's recorded roll / pitch (window mean
removed) and yaw (relative to the start), adds a slow position hold (<= 3 deg) and an altitude
hold so the vehicle stays put, then publishes /path_following_complete so flight_manager lands.
Comparing the rate response of this flight with the vehicle's own (scripts/dev/x8_sysid.py)
tests the Gazebo model with the same excitation.

usage: x8_replay.py --ros-args -p csv:=<t,roll,pitch,yaw_rel> [-p hover_thrust:=0.44]
"""
import math
import sys

import numpy as np
import rclpy
from px4_msgs.msg import VehicleAttitude, VehicleAttitudeSetpoint, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

G = 9.80665


class Replay(Node):
    def __init__(self):
        super().__init__('x8_replay')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        d = np.genfromtxt(p('csv', ''), delimiter=',', names=True)
        self.rec_t, self.rec_r, self.rec_p, self.rec_y = d['t'], d['roll'], d['pitch'], d['yaw_rel']
        self.t_hover = float(p('hover_thrust', 0.44))
        self.start_after_arm = float(p('start_after_arm_s', 13.0))
        self.pos = self.vel = None
        self.yaw = 0.0
        self.t_armed = None
        self.t0 = None
        self.done = False
        self.integ = 0.0
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.lp_cb, qos_profile_sensor_data)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.att_cb, qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.st_cb, qos_profile_sensor_data)
        self.att_pub = self.create_publisher(VehicleAttitudeSetpoint, '/pf_att_2_control', 10)
        self.done_pub = self.create_publisher(Bool, '/path_following_complete', 1)
        self.create_timer(0.02, self.tick)
        self.get_logger().info(f'[x8_replay] {len(self.rec_t)} setpoints over {self.rec_t[-1]:.1f} s')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def lp_cb(self, m):
        self.pos = np.array([m.x, m.y, m.z]); self.vel = np.array([m.vx, m.vy, m.vz])

    def att_cb(self, m):
        w, x, y, z = m.q
        self.yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def st_cb(self, m):
        if m.arming_state == VehicleStatus.ARMING_STATE_ARMED and self.t_armed is None:
            self.t_armed = self.now()

    def tick(self):
        if self.pos is None or self.t_armed is None or self.done:
            return
        t = self.now()
        if self.t0 is None:
            if t - self.t_armed < self.start_after_arm:
                return
            self.t0, self.p0, self.yaw0, self.alt0 = t, self.pos[:2].copy(), self.yaw, -self.pos[2]
            self.get_logger().info(f'[x8_replay] start at alt {self.alt0:.1f} m')
        tr = t - self.t0
        if tr > self.rec_t[-1]:
            self.done = True
            m = Bool(); m.data = True
            for _ in range(5):
                self.done_pub.publish(m)
            self.get_logger().info('[x8_replay] done - landing')
            return
        r = float(np.interp(tr, self.rec_t, self.rec_r))
        pt = float(np.interp(tr, self.rec_t, self.rec_p))
        yw = self.yaw0 + float(np.interp(tr, self.rec_t, self.rec_y))
        # slow position hold (NED), as tilt in the heading frame, <= 3 deg
        a = -0.25 * (self.pos[:2] - self.p0) - 0.8 * self.vel[:2]
        a_f = math.cos(self.yaw) * a[0] + math.sin(self.yaw) * a[1]
        a_r = -math.sin(self.yaw) * a[0] + math.cos(self.yaw) * a[1]
        lim = math.radians(3)
        pt += max(-lim, min(lim, -math.atan(a_f / G)))
        r += max(-lim, min(lim, math.atan(a_r / G)))
        # altitude hold
        e = self.alt0 - (-self.pos[2])
        self.integ = max(-0.1, min(0.1, self.integ + 0.03 * e * 0.02))
        thr = (self.t_hover + 0.08 * e + 0.06 * self.vel[2] + self.integ) / max(math.cos(r) * math.cos(pt), 0.5)
        thr = max(0.1, min(0.9, thr))
        cr, sr, cp, sp, cy, sy = math.cos(r / 2), math.sin(r / 2), math.cos(pt / 2), math.sin(pt / 2), math.cos(yw / 2), math.sin(yw / 2)
        msg = VehicleAttitudeSetpoint()
        msg.q_d = [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]
        msg.thrust_body = [0.0, 0.0, -thr]
        self.att_pub.publish(msg)


def main():
    rclpy.init(args=sys.argv)
    n = Replay()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
