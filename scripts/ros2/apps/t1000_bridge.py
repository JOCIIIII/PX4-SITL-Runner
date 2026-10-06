#!/usr/bin/env python3
"""SITL inputs for T1000 Control 445 in its --isaac-sim mode.

T1000's autonomous SITL lifecycle (--sim-auto-start/--land-on-goal) only exists in
--isaac-sim mode, which reads the simulator through three topics instead of PX4 + a
Livox cloud. This node produces them from this SITL (Gazebo/PX4 + AirSim lidar):

  /odom (adapter, ENU world = PX4 origin, FLU body)  ->  /t1000/pose      PoseStamped
                                                    ->  /t1000/velocity  TwistStamped (world ENU)
  AirSim lidar (sensor frame FLU, one copy per scan) ->  /t1000/lidar_bins Float32MultiArray[432]

Bins use the arithmetic of t1000_control.sensors.binning.xyz_to_432bins (points relative to
the vehicle, yaw-only levelled body frame, 72 x 6 layers, range / LIDAR_MAX, 1 = clear) but
number the azimuth counter-clockwise from the nose, as the policy expects (see bin_azimuth).
Each scan is levelled with the vehicle attitude, so no world registration (and no PX4/AirSim
offset) is involved.
"""
import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
import sensor_msgs_py.point_cloud2 as pc2

from t1000_control import settings as CFG

H_BINS = CFG.LIDAR_H_BINS
V_BOUNDARIES = np.asarray(CFG.LIDAR_V_BOUNDARIES, dtype=np.float64)
TOTAL_BINS = CFG.LIDAR_TOTAL_BINS


def bins_432(rel, yaw, ccw=False):
    """Vectorised t1000_control.sensors.binning.xyz_to_432bins (same arithmetic).

    ccw=True numbers the azimuth bins counter-clockwise (left of the nose first), as the
    real-flight path does (sensor_xyz_to_432bins -> _bin_body_frame: atan2(y, x) on the
    Livox FLU cloud); xyz_to_432bins uses atan2(-y, x) - the mirror image.
    """
    bins = np.ones(TOTAL_BINS, dtype=np.float32)
    if len(rel) == 0:
        return bins
    cy, sy = np.cos(-yaw), np.sin(-yaw)
    xb = cy * rel[:, 0] + sy * rel[:, 1]
    yb = -sy * rel[:, 0] + cy * rel[:, 1]
    zb = rel[:, 2]
    dist = np.sqrt(xb ** 2 + yb ** 2 + zb ** 2)
    m = (dist >= CFG.LIDAR_MIN) & (dist <= CFG.LIDAR_MAX)
    xb, yb, zb, dist = xb[m], yb[m], zb[m], dist[m]
    if len(dist) == 0:
        return bins
    az = np.degrees(np.arctan2(yb if ccw else -yb, xb)) % 360.0
    el = np.degrees(np.arcsin(np.clip(zb / np.maximum(dist, 1e-6), -1.0, 1.0)))
    h_idx = (az / 360.0 * H_BINS).astype(int) % H_BINS
    v_idx = np.searchsorted(V_BOUNDARIES[1:-1], el)
    flat = (v_idx * H_BINS + h_idx).clip(0, TOTAL_BINS - 1)
    norm = (dist / CFG.LIDAR_MAX).clip(0.0, 1.0).astype(np.float32)
    np.minimum.at(bins, flat, norm)
    return bins


def quat_to_rot(qw, qx, qy, qz):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


class T1000Bridge(Node):
    def __init__(self):
        super().__init__('t1000_bridge')
        self.frame = self.declare_parameter('frame_id', 'map').value
        lidar_topic = self.declare_parameter(
            'lidar_topic', '/airsim_node/SimpleFlight/lidar/points/RPLIDAR_A3').value
        # sensor origin in the FLU body frame (AirSim settings: Z = -0.3 NED, no tilt)
        self.lidar_off = np.array([
            float(self.declare_parameter('lidar_offset_x', 0.0).value),
            float(self.declare_parameter('lidar_offset_y', 0.0).value),
            float(self.declare_parameter('lidar_offset_z', 0.3).value)])
        # Azimuth numbering of the bins. The policy reads LOW indices as LEFT of the nose
        # (counter-clockwise): offline probe of policy_445.onnx, an obstacle patch at bins
        # 1-10 -> lateral action -1 (go right), its mirror at 62-71 -> +1, 16/16 cases. That
        # matches T1000's real-flight path (atan2(y, x) on the Livox FLU cloud) and its own
        # recovery-corridor / roll geometry. xyz_to_432bins (named in its README for SITL)
        # numbers clockwise - the mirror image; flown that way the policy steered toward
        # obstacles (75% of sampled actions) and only the speed caps kept it clear.
        self.ccw = self.declare_parameter('bin_azimuth', 'ccw').value == 'ccw'
        self.q = None
        self.last_stamp = None
        self.n_scans = 0
        self.pose_pub = self.create_publisher(PoseStamped, CFG.TOPIC_ISAAC_POSE, 10)
        self.vel_pub = self.create_publisher(TwistStamped, CFG.TOPIC_ISAAC_VELOCITY, 10)
        self.bins_pub = self.create_publisher(Float32MultiArray, CFG.TOPIC_ISAAC_LIDAR_BINS, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, lidar_topic, self.lidar_cb, qos_profile_sensor_data)
        self.get_logger().info(f'[t1000_bridge] /odom + {lidar_topic} -> /t1000/pose, '
                               f'/t1000/velocity, /t1000/lidar_bins ({TOTAL_BINS}, azimuth '
                               f'{"ccw" if self.ccw else "cw"})')

    def odom_cb(self, m):
        o = m.pose.pose.orientation
        self.q = (o.w, o.x, o.y, o.z)
        ps = PoseStamped()
        ps.header.stamp = m.header.stamp
        ps.header.frame_id = self.frame
        ps.pose = m.pose.pose
        self.pose_pub.publish(ps)
        tw = TwistStamped()
        tw.header = ps.header
        tw.twist.linear = m.twist.twist.linear   # adapter /odom twist is world ENU
        self.vel_pub.publish(tw)

    def lidar_cb(self, m):
        # AirSim's ROS bridge republishes each scan ~100 Hz; bin each scan once
        stamp = (m.header.stamp.sec, m.header.stamp.nanosec)
        if stamp == self.last_stamp or self.q is None:
            return
        self.last_stamp = stamp
        xyz = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        qw, qx, qy, qz = self.q
        # sensor -> body -> world-aligned (rotation only, vehicle at the origin)
        rel = (xyz + self.lidar_off) @ quat_to_rot(qw, qx, qy, qz).T
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        out = Float32MultiArray()
        out.data = bins_432(rel, yaw, self.ccw).tolist()
        self.bins_pub.publish(out)
        self.n_scans += 1
        if self.n_scans == 1:
            self.get_logger().info(f'[t1000_bridge] first scan binned: {xyz.shape[0]} points')


def main():
    rclpy.init(args=sys.argv)
    node = T1000Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
