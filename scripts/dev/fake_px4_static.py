#!/usr/bin/env python3
"""Bench stand-in for PX4 when no flight controller is connected: publishes a resting, level vehicle on
/fmu/out/vehicle_local_position, /fmu/out/vehicle_attitude (100 Hz) and /fmu/out/vehicle_status_v1 (2 Hz,
disarmed), so the onboard chain (lidar adapter, policy node in shadow mode) runs on the real lidar.
The pose: NED (0, 0, Z) with Z = --z (default -13.2, like the X500 at rest in its logs), heading --yaw deg
(0 = north). usage: fake_px4_static.py [--z -13.2] [--yaw 0]"""
import argparse
import math

import rclpy
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

ap = argparse.ArgumentParser()
ap.add_argument('--z', type=float, default=-13.2)
ap.add_argument('--yaw', type=float, default=0.0)
a, _ = ap.parse_known_args()


class FakePx4(Node):
    def __init__(self):
        super().__init__('fake_px4_static')
        self.lp = self.create_publisher(VehicleLocalPosition, '/fmu/out/vehicle_local_position', qos_profile_sensor_data)
        self.att = self.create_publisher(VehicleAttitude, '/fmu/out/vehicle_attitude', qos_profile_sensor_data)
        self.st = self.create_publisher(VehicleStatus, '/fmu/out/vehicle_status_v1', qos_profile_sensor_data)
        self.create_timer(0.01, self.fast)
        self.create_timer(0.5, self.slow)
        self.get_logger().info(f'[fake_px4] resting at NED (0, 0, {a.z}), heading {a.yaw} deg - no flight controller')

    def us(self):
        return self.get_clock().now().nanoseconds // 1000

    def fast(self):
        t = self.us()
        q = VehicleAttitude()
        q.timestamp = t
        h = math.radians(a.yaw) / 2.0
        q.q = [math.cos(h), 0.0, 0.0, math.sin(h)]          # FRD -> NED, level, heading yaw
        self.att.publish(q)
        p = VehicleLocalPosition()
        p.timestamp = t
        p.xy_valid = p.z_valid = p.v_xy_valid = p.v_z_valid = True
        p.x, p.y, p.z = 0.0, 0.0, float(a.z)
        p.heading = math.radians(a.yaw)
        self.lp.publish(p)

    def slow(self):
        s = VehicleStatus()
        s.timestamp = self.us()
        s.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.st.publish(s)


rclpy.init()
rclpy.spin(FakePx4())
