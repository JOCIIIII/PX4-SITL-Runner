#!/usr/bin/env python3
"""PX4 link shared by the UGV/USV nodes of scripts/dev/ground: state from /fmu/out, the offboard velocity
setpoint PX4's rover_differential takes (TrajectorySetpoint.velocity in NED: its norm is the speed setpoint,
its direction the bearing = yaw setpoint), arming and the offboard switch.

Stopping: a zero velocity vector gives bearing atan2(0, 0) = 0, i.e. the rover would turn north while it
stops. hold() therefore sends a 1 mm/s vector along the bearing to keep (speed setpoint ~ 0)."""
import math

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition,
                          VehicleAttitude, VehicleOdometry, VehicleStatus, VehicleControlMode)

NAN = float('nan')


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Px4Link:
    def __init__(self, node: Node, ns: str = ''):
        self.n = node
        self.pos = None            # north, east, down [m]
        self.vel = None            # NED [m/s]
        self.yaw = None            # [rad], NED
        self.yaw_rate = 0.0        # [rad/s] body z (FRD)
        self.xy_valid = False
        self.nav_state = -1
        self.arming_state = -1
        self.preflight_ok = False
        self.offboard = False
        self.armed = False
        self.t_pos_us = 0
        out = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        node.create_subscription(VehicleLocalPosition, f'{ns}/fmu/out/vehicle_local_position', self._lp, qos_profile_sensor_data)
        node.create_subscription(VehicleAttitude, f'{ns}/fmu/out/vehicle_attitude', self._att, qos_profile_sensor_data)
        node.create_subscription(VehicleOdometry, f'{ns}/fmu/out/vehicle_odometry', self._odom, qos_profile_sensor_data)
        for topic in ('vehicle_status', 'vehicle_status_v1'):
            node.create_subscription(VehicleStatus, f'{ns}/fmu/out/{topic}', self._status, qos_profile_sensor_data)
        node.create_subscription(VehicleControlMode, f'{ns}/fmu/out/vehicle_control_mode', self._mode, qos_profile_sensor_data)
        self.pub_mode = node.create_publisher(OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', out)
        self.pub_sp = node.create_publisher(TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', out)
        self.pub_cmd = node.create_publisher(VehicleCommand, f'{ns}/fmu/in/vehicle_command', out)

    # ------------------------------------------------------------------ state
    def _lp(self, m):
        self.pos = (float(m.x), float(m.y), float(m.z))
        self.vel = (float(m.vx), float(m.vy), float(m.vz))
        self.xy_valid = bool(m.xy_valid) and bool(m.v_xy_valid)
        self.t_pos_us = int(m.timestamp)

    def _att(self, m):
        w, x, y, z = (float(v) for v in m.q)
        self.yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _odom(self, m):
        self.yaw_rate = float(m.angular_velocity[2])

    def _status(self, m):
        self.nav_state = int(m.nav_state)
        self.arming_state = int(m.arming_state)
        self.preflight_ok = bool(m.pre_flight_checks_pass)

    def _mode(self, m):
        self.offboard = bool(m.flag_control_offboard_enabled)
        self.armed = bool(m.flag_armed)

    def ready(self):
        return self.pos is not None and self.yaw is not None and self.xy_valid

    def speed(self):
        return math.hypot(self.vel[0], self.vel[1]) if self.vel else 0.0

    def forward_speed(self):
        if not self.vel or self.yaw is None:
            return 0.0
        return self.vel[0] * math.cos(self.yaw) + self.vel[1] * math.sin(self.yaw)

    def course(self):
        return math.atan2(self.vel[1], self.vel[0]) if self.vel else 0.0

    # ------------------------------------------------------------------ commands
    def _now_us(self):
        return int(self.n.get_clock().now().nanoseconds / 1000)

    def send_velocity(self, speed: float, bearing: float):
        """speed [m/s] >= 0 along bearing [rad, NED]"""
        m = OffboardControlMode()
        m.timestamp = self._now_us()
        m.position, m.velocity, m.acceleration, m.attitude, m.body_rate = False, True, False, False, False
        self.pub_mode.publish(m)
        s = TrajectorySetpoint()
        s.timestamp = m.timestamp
        v = max(float(speed), 1.0e-3)
        s.position = [NAN, NAN, NAN]
        s.velocity = [v * math.cos(bearing), v * math.sin(bearing), 0.0]
        s.acceleration = [NAN, NAN, NAN]
        s.yaw, s.yawspeed = NAN, NAN
        self.pub_sp.publish(s)

    def hold(self, bearing: float):
        self.send_velocity(0.0, bearing)

    def command(self, cmd: int, p1: float = 0.0, p2: float = 0.0):
        m = VehicleCommand()
        m.timestamp = self._now_us()
        m.command = int(cmd)
        m.param1, m.param2 = float(p1), float(p2)
        m.target_system, m.target_component, m.source_system, m.source_component = 1, 1, 1, 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def arm(self):
        self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def set_offboard(self):
        self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
