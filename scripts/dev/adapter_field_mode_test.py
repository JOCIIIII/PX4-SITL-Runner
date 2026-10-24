#!/usr/bin/env python3
"""Check the lidar adapter's deskew_mode 'field' (the real Mid-360 through livox_ros_driver2): a wall
10 m north of the origin, shot over 0.1 s while the vehicle flies north at 5 m/s, each point
carrying its own time (float64 ns, as the Livox driver sends it). Registered with the PX4 pose at
each point's time, the wall stays at north 10 m; registered with the frame-end pose it smears up to
v * 0.1 s = 0.5 m. Also: a frame shot while the vehicle pitches faster than max_tilt_rate is
dropped, and a frame whose time field is unusable is still registered (without de-skew).
Exit status 1 on a failed check. Run it inside the ROS image:
  docker run --rm -v $HOME/Documents/A4VAI-SITL/ROS2:/home/user/workspace/ros2 \\
      -v $HOME/PX4-SITL-Runner:$HOME/PX4-SITL-Runner:ro --entrypoint bash jociiiii/a4vai:inhousesim-v0.9 -c \\
      "source /opt/ros/humble/setup.bash && source /home/user/workspace/ros2/ros2_ws/install/setup.bash && \\
      python3 $HOME/PX4-SITL-Runner/scripts/dev/adapter_field_mode_test.py"
"""
import math
import sys

import numpy as np
import rclpy
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

from realgazebo_fastplanner_adapters.px4_lidar_to_odom_cloud import Px4LidarToOdomCloud

V, T_END, SPAN, LIDAR_Z = 5.0, 1000.0, 0.1, 0.1
fails = 0


def check(ok, what):
    global fails
    print(f'  {"ok  " if ok else "FAIL"} {what}')
    fails += 0 if ok else 1


def make_node(deskew, tilt=0.0, self_r=0.0):
    args = ['--ros-args', '-p', 'deskew_mode:=field', '-p', 'use_ros_time:=false', '-p', "px4_ns:=''",
            '-p', f'deskew_rot_period_s:={0.1 if deskew else 0.0}', '-p', f'max_tilt_rate:={tilt}',
            '-p', f'lidar_offset_z:={LIDAR_Z}', '-p', 'publish_tf:=false', '-p', 'pose_buffer_sec:=2.0',
            '-p', 'min_range:=0.2', '-p', f'self_radius_m:={self_r}']
    rclpy.init(args=args)
    n = Px4LidarToOdomCloud()
    n.out = []
    n.cloud_pub.publish = n.out.append
    return n


def feed_poses(n, pitch_rate=0.0):
    """level (or pitching nose-down at pitch_rate) and heading north, flying north at V, 1.5 m up"""
    for t in np.arange(T_END - 1.0, T_END + 0.05, 0.004):
        a = VehicleAttitude()
        th = pitch_rate * (t - (T_END - 1.0))       # pitch about the body y (FRD) axis
        a.q = [math.cos(th / 2), 0.0, math.sin(-th / 2), 0.0]
        n.att_cb(a)
        p = VehicleLocalPosition()
        p.timestamp = int(round(t * 1e6))
        p.xy_valid = p.z_valid = True
        p.x, p.y, p.z = float(V * (t - T_END)), 0.0, -1.5
        p.vx = V
        n.pos_cb(p)


def wall_cloud(time_scale=1e-9, zero_times=False, own=0):
    """Livox-style PointCloud2 (x y z intensity tag line timestamp) of the wall at north 10 m"""
    rng = np.random.default_rng(1)
    k = 4000 + own
    ts = np.sort(rng.uniform(T_END - SPAN, T_END, k))
    east = rng.uniform(-2.0, 2.0, k)
    up = rng.uniform(0.5, 2.5, k)
    north_vehicle = V * (ts - T_END)                # vehicle north at each shot
    x = 10.0 - north_vehicle                         # FLU forward = north
    y = -east                                        # FLU left = west
    z = up - 1.5 - LIDAR_Z
    if own:                                          # the vehicle's own parts: 0.3 m out, at the lidar's height
        ang = rng.uniform(-math.pi, math.pi, own)
        idx = rng.choice(k, own, replace=False)
        x[idx], y[idx], z[idx] = 0.3 * np.cos(ang), 0.3 * np.sin(ang), rng.uniform(-0.1, 0.1, own)
    dt = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32),
                   ('tag', np.uint8), ('line', np.uint8), ('timestamp', np.float64)])
    arr = np.zeros(k, dtype=dt)
    arr['x'], arr['y'], arr['z'] = x, y, z
    arr['timestamp'] = 0.0 if zero_times else ts / time_scale
    fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
              PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
              PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
              PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
              PointField(name='tag', offset=16, datatype=PointField.UINT8, count=1),
              PointField(name='line', offset=17, datatype=PointField.UINT8, count=1),
              PointField(name='timestamp', offset=18, datatype=PointField.FLOAT64, count=1)]
    h = Header()
    h.frame_id = 'livox_frame'
    h.stamp.sec, h.stamp.nanosec = int(T_END), int(round((T_END % 1) * 1e9))
    msg = PointCloud2(header=h, height=1, width=k, fields=fields, is_bigendian=False,
                      point_step=dt.itemsize, row_step=dt.itemsize * k, is_dense=True,
                      data=arr.tobytes())
    return msg


def north_of(n):
    out = n.out[-1]
    return pc2.read_points_numpy(out, field_names=('x', 'y', 'z'))[:, 1]


print('1. wall 10 m north, 0.1 s frame at 5 m/s')
for deskew in (True, False):
    n = make_node(deskew)
    feed_poses(n)
    n.cloud_cb(wall_cloud())
    nn = north_of(n) if n.out else np.array([np.nan])
    err = np.abs(nn - 10.0)
    if deskew:
        check(err.max() < 0.05, f'de-skewed: wall at north {nn.mean():.3f} m, max error {err.max():.3f} m (< 0.05)')
    else:
        check(0.4 < err.max() < 0.55, f'end pose only: smeared up to {err.max():.3f} m (v x span = {V * SPAN:.2f})')
    n.destroy_node()
    rclpy.shutdown()

print('2. tilt gate (max_tilt_rate 0.8 rad/s) with PX4 attitude')
for rate, dropped in ((1.5, True), (0.3, False)):
    n = make_node(True, tilt=0.8)
    feed_poses(n, pitch_rate=rate)
    n.cloud_cb(wall_cloud())
    check((len(n.out) == 0) == dropped, f'pitching {rate} rad/s: frame {"dropped" if not n.out else "kept"}')
    n.destroy_node()
    rclpy.shutdown()

print('3. no usable point times (all zero) -> registered without de-skew, not lost')
n = make_node(True)
feed_poses(n)
n.cloud_cb(wall_cloud(zero_times=True))
check(len(n.out) == 1 and n._warned_span, f'frame published ({len(n.out)}), warning {"given" if n._warned_span else "missing"}')
n.destroy_node()
rclpy.shutdown()

print('4. self filter (self_radius_m 0.55) on the vehicle')
for self_r, expect in ((0.55, 4000 - 0), (0.0, 4300)):
    n = make_node(True, self_r=self_r)
    feed_poses(n)
    n.cloud_cb(wall_cloud(own=300))
    got = len(north_of(n)) if n.out else 0
    check(got == expect if self_r else got == 4300,
          f'self_radius {self_r}: {got} of 4300 points kept (300 own parts 0.3 m out {"removed" if self_r else "kept"})')
    n.destroy_node()
    rclpy.shutdown()

print('FAILED' if fails else 'ALL OK')
sys.exit(1 if fails else 0)
