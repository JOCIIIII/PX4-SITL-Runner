#!/usr/bin/env python3
"""Integrated test (PP -> PF -> CA) for the avoidance chains without a prefix of their own (T1000, SUPER): the planned
prefix path is flown first, then PathFollowing is handed to the avoidance chain.

The chain's path and speed are remapped to /ca/local_waypoint_setpoint_to_PF and /ca/desired_speed. This node
publishes PathFollowing's inputs (/local_waypoint_setpoint_to_PF, /desired_speed) in two phases:
  - prefix: the planned path, fed the way the in-house policy node feeds its own prefix (noclimb_ca_node.py
    prefix_step), so the integrated test flies the same prefix whichever avoidance follows:
      - progress along the path, never backwards
      - the REMAINING path at 1 Hz, from the vehicle through the rest, 2 points past the end
        (a full path each time would restart PathFollowing at its first point)
      - the speed eased before each corner and braked toward the end speed
    prefix_path "east,north;..." = metres from the take-off point, PX4 local ENU, as scenario_super.sh's
    NOCLIMB_PREFIX_PATH, at the height altitude.
  - avoidance: within handover_radius_m of the prefix end (on its last segment), the chain's messages are forwarded
    as they come.
State on /prefix_relay/state: prefix | avoidance.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Float32, String
from px4_msgs.msg import VehicleLocalPosition
from custom_msgs.msg import LocalWaypointSetpoint


class PrefixRelay(Node):
    def __init__(self):
        super().__init__('prefix_relay')
        p = self.declare_parameter
        raw = p('prefix_path', '').value
        self.speed = float(p('prefix_speed', 5.0).value)
        self.alt = float(p('altitude', 1.5).value)
        self.radius = float(p('handover_radius_m', 3.0).value)
        self.end_speed = float(p('end_speed', -1.0).value)             # < 0: prefix_speed (the chain flies on)
        self.corner_a = float(p('corner_accel', 4.0).value)            # noclimb_ca_node.py prefix defaults
        self.corner_e = float(p('corner_err_m', 1.5).value)
        self.brake = float(p('brake_accel', 3.0).value)
        pts = [[float(v) for v in q.split(',')[:2]] for q in raw.split(';') if q.strip()]
        self.P = np.array(pts, dtype=float) if len(pts) >= 2 else None
        self.phase = 'prefix' if self.P is not None else 'avoidance'
        self.seg = 0
        self.steps = 0
        self.pos = None
        wp_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.wp_pub = self.create_publisher(LocalWaypointSetpoint, '/local_waypoint_setpoint_to_PF', wp_qos)
        self.speed_pub = self.create_publisher(Float32, '/desired_speed', 10)
        self.state_pub = self.create_publisher(String, '/prefix_relay/state', 10)
        self.create_subscription(LocalWaypointSetpoint, '/ca/local_waypoint_setpoint_to_PF', self.on_ca_path, 1)
        self.create_subscription(Float32, '/ca/desired_speed', self.on_ca_speed, 10)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_lpos,
                                 qos_profile_sensor_data)
        self.create_timer(0.05, self.step)
        if self.P is not None:
            L = float(np.sum(np.linalg.norm(np.diff(self.P, axis=0), axis=1)))
            self.get_logger().info(f'prefix {len(self.P)} points, {L:.0f} m at {self.speed} m/s, height {self.alt} m; '
                                   f'hand-over within {self.radius} m of its end')

    def on_lpos(self, m):
        if m.xy_valid:
            self.pos = np.array([float(m.y), float(m.x)])     # ENU east, north

    def on_ca_path(self, m):
        if self.phase == 'avoidance':
            self.wp_pub.publish(m)

    def on_ca_speed(self, m):
        if self.phase == 'avoidance':
            self.speed_pub.publish(m)

    def publish_path(self, enu):
        out = LocalWaypointSetpoint()
        out.path_planning_complete = True
        out.waypoint_x = [float(q[1]) for q in enu]        # north
        out.waypoint_y = [float(q[0]) for q in enu]        # east
        out.waypoint_z = [float(self.alt)] * len(enu)     # altitude (+)
        self.wp_pub.publish(out)

    def step(self):
        self.state_pub.publish(String(data=self.phase))
        if self.phase != 'prefix' or self.pos is None:
            return
        P, pos = self.P, self.pos
        best, seg = None, self.seg
        for i in range(self.seg, len(P) - 1):                # progress along the path, never backwards
            a, b = P[i], P[i + 1]
            ab = b - a
            L2 = float(ab @ ab)
            u = 0.0 if L2 < 1e-9 else float(np.clip((pos - a) @ ab / L2, 0.0, 1.0))
            d = float(np.linalg.norm(a + u * ab - pos))
            if best is None or d < best - 1e-6:
                best, seg = d, i
            elif d > best + 5.0:
                break
        self.seg = seg
        d_end = float(np.linalg.norm(P[-1] - pos))
        if d_end < self.radius and seg >= len(P) - 2:
            self.phase = 'avoidance'
            self.get_logger().info(f'prefix done ({d_end:.2f} m from its end): the avoidance chain flies on')
            return
        v = self.speed
        s = float(np.linalg.norm(P[seg + 1] - pos))          # along the path to vertex seg + 1
        to_end = s + float(np.sum(np.linalg.norm(np.diff(P[seg + 1:], axis=0), axis=1))) if seg + 2 < len(P) else s
        v_end = self.end_speed if self.end_speed >= 0.0 else self.speed
        v = min(v, math.sqrt(v_end ** 2 + 2.0 * self.brake * to_end))
        for j in range(seg + 1, len(P) - 1):
            a, b, c = P[j - 1], P[j], P[j + 1]
            l1, l2 = float(np.linalg.norm(b - a)), float(np.linalg.norm(c - b))
            if l1 < 1e-6 or l2 < 1e-6:
                continue
            dpsi = math.acos(float(np.clip(((b - a) / l1) @ ((c - b) / l2), -1.0, 1.0)))
            if dpsi > math.radians(5.0):
                # the arc that keeps within corner_err_m of the corner, cut to half of either leg
                d_turn = min(self.corner_e / max(1.0 - math.cos(dpsi / 2.0), 1e-6) * math.tan(dpsi / 2.0),
                             0.5 * min(l1, l2))
                R = d_turn / math.tan(dpsi / 2.0)
                v_c = math.sqrt(self.corner_a * R)
                v = min(v, math.sqrt(v_c ** 2 + 2.0 * self.brake * max(s - d_turn, 0.0)))
            s += l2
            if s > 80.0:
                break
        self.speed_pub.publish(Float32(data=float(v)))
        if self.steps % 20 == 0:                              # at the start, then 1 Hz (the driver times out at 2.5 s)
            u = (P[-1] - P[-2]) / max(float(np.linalg.norm(P[-1] - P[-2])), 1e-6)
            rest = [pos] + [P[k] for k in range(seg + 1, len(P))] + [P[-1] + 5.0 * u, P[-1] + 10.0 * u]
            self.publish_path(rest)
        self.steps += 1


def main():
    rclpy.init()
    node = PrefixRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
