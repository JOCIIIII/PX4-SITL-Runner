#!/usr/bin/env python3
"""THE VEHICLE: the goal DIST m ahead along where the vehicle points now, as the NOCLIMB_GOAL "east,north" line for
scripts/dev/onboard_real.sh fly (relative to the take-off point = where the vehicle stands when fly starts).
Averages PX4's heading (/fmu/out/vehicle_local_position) over --seconds. Runs in the onboard image, ROS sourced -
scripts/dev/heading_goal.sh starts it in a throwaway container (no app needs to be up, only the agent and PX4).

usage: heading_goal.py [DIST] [--seconds 2]
"""
import argparse
import math
import time

import rclpy
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import qos_profile_sensor_data

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('dist', type=float, nargs='?', default=40.0)
ap.add_argument('--seconds', type=float, default=2.0)
a = ap.parse_args()
rclpy.init()
n = rclpy.create_node('heading_goal')
msgs = []
n.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', msgs.append, qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < a.seconds + 10.0 and (not msgs or time.time() - t0 < a.seconds):
    rclpy.spin_once(n, timeout_sec=0.1)
if not msgs:
    raise SystemExit('no /fmu/out/vehicle_local_position: is the agent up and PX4 connected (reboot the Pixhawk '
                     'after the agent)?')
s = sum(math.sin(m.heading) for m in msgs)
c = sum(math.cos(m.heading) for m in msgs)
h = math.atan2(s, c)                                     # circular mean, PX4 heading: from north, clockwise
spread = max(abs(math.degrees(math.remainder(m.heading - h, 2 * math.pi))) for m in msgs)
last = msgs[-1]
print(f'heading {math.degrees(h):.1f} deg from north, clockwise ({len(msgs)} samples, spread {spread:.1f} deg); '
      f'xy_valid {last.xy_valid}, heading_good_for_control {last.heading_good_for_control}')
print(f'{a.dist:g} m ahead (east, north from where the vehicle stands):')
print(f'NOCLIMB_GOAL={a.dist * math.sin(h):.1f},{a.dist * math.cos(h):.1f}')
