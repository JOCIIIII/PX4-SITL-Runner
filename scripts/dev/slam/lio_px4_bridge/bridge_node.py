"""lio_px4_bridge: FAST-LIO2 odometry -> PX4 external vision (px4_msgs/VehicleOdometry).

FAST-LIO's world is the IMU frame at the moment it started: not gravity-aligned, arbitrary heading. While the
vehicle rests, the bridge pairs FAST-LIO's pose with PX4's attitude (gravity-referenced, heading from mag / GPS)
once to get a fixed W -> NED transform and starts the body where PX4's local position is. From then on it sends
the vehicle body (FRD, at its centre) pose, velocity and quality at the scan rate, and stops sending when FAST-LIO
goes stale, jumps or turns unhealthy, so PX4's own EV timeout and failsafes take over.

send_to_px4 is false by default: the output goes to ~/vehicle_odometry (shadow) and PX4 is not touched. Only with
send_to_px4 true does it publish {px4_ns}/fmu/in/vehicle_visual_odometry.

Timestamps are this computer's ROS time in microseconds, like flight_driver's now_us(); uXRCE-DDS converts them.
The sensor clock (the lidar's, or the simulator's) is mapped to it with the smallest receipt delay of the IMU stream.
"""
import json
import math
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleOdometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, String

from lio_px4_bridge.frames import (Alignment, ClockOffset, DegradedLatch, Extrinsics, angle_between, body_state,
                                   quality_from_health, quat_to_rot, rot_to_quat, rot_to_rpy_deg, yaw_of)

WAIT, REST, RUN = 'WAIT', 'REST', 'RUN'     # waiting for inputs / collecting rest samples / sending
CSV_HEADER = ('t_rx,t_sample,sent,quality,eff,residual,x,y,z,vx,vy,vz,roll,pitch,yaw,'
              'px4_dt,px4_x,px4_y,px4_z,px4_vx,px4_vy,px4_vz,px4_xy_valid,px4_z_valid,px4_roll,px4_pitch,px4_yaw,down,coast')


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def nearest(history, t):
    """(t_rx, value) of history closest to t, or None."""
    return min(history, key=lambda h: abs(h[0] - t)) if history else None


class LioPx4Bridge(Node):
    def __init__(self):
        super().__init__('lio_px4_bridge')
        p = self.declare_parameter
        ns = str(p('px4_ns', '').value)
        odom_topic = str(p('odom_topic', '/Odometry').value)
        health_topic = str(p('health_topic', '/fast_lio/health').value)
        imu_topic = str(p('imu_topic', '/livox/imu').value)          # clock reference: FAST-LIO's own IMU stream
        lidar_rpy = list(p('lidar_rpy_deg', [0.0, 0.0, 0.0]).value)  # NOCLIMB_LIDAR_ROLL_DEG / _PITCH_DEG / _YAW_DEG
        lidar_xyz = list(p('lidar_xyz', [0.0, 0.0, 0.0]).value)      # NOCLIMB_LIDAR_X / _Y / _Z (FLU, m)
        self.ext = Extrinsics(lidar_rpy, lidar_xyz,
                              list(p('imu_R_lidar', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]).value),
                              list(p('imu_p_lidar', [0.0, 0.0, 0.0]).value))
        self.send_to_px4 = bool(p('send_to_px4', False).value)
        self.heading_mode = str(p('heading_mode', 'px4').value)      # px4 | fixed | initial
        self.heading = math.radians(float(p('heading_deg', 0.0).value))
        self.align_to_px4_pos = bool(p('align_to_px4_position', True).value)
        self.rest_window = float(p('rest_window_s', 2.0).value)
        self.rest_gyro = float(p('rest_gyro_max', 0.3).value)        # rad/s, coarse: armed props shake the IMU
        self.rest_speed = float(p('rest_speed_max', 0.2).value)      # m/s; rest is decided by the spread below
        self.rest_spread_m = float(p('rest_spread_m', 0.05).value)
        self.rest_spread_rad = math.radians(float(p('rest_spread_deg', 0.5).value))
        self.stamp_mode = str(p('stamp_mode', 'auto').value)         # auto | header | receipt
        self.receipt_latency = float(p('receipt_latency_s', 0.05).value)
        self.clock = ClockOffset(float(p('clock_window_s', 2.0).value))
        self.var_pos = float(p('pos_std', 0.10).value) ** 2
        self.var_vel = float(p('vel_std', 0.20).value) ** 2
        self.var_att = math.radians(float(p('att_std_deg', 3.0).value)) ** 2
        self.send_velocity = bool(p('send_velocity', True).value)
        self.health_required = bool(p('health_required', True).value)
        self.eff_min = float(p('eff_min', 100.0).value)
        self.eff_good = float(p('eff_good', 500.0).value)
        self.residual_max = float(p('residual_max', 0.2).value)
        # health_sparse (2026-10-27, default off = eff_min / residual_max alone): a scan below eff_min is still sent,
        # at quality 1, when FAST-LIO matched >= sparse_ratio_min of its input points with a residual <= sparse_residual_max
        # and >= sparse_eff_min points (frames.quality_from_health)
        self.sparse = ((float(p('sparse_ratio_min', 0.3).value), float(p('sparse_residual_max', 0.1).value),
                        float(p('sparse_eff_min', 40.0).value)) if bool(p('health_sparse', False).value) else None)
        self.max_speed = float(p('max_speed', 15.0).value)           # m/s between scans; faster = FAST-LIO jumped
        self.lost_after = float(p('lost_after_s', 1.0).value)        # FAST-LIO silent this long: its world is gone
        # degraded_latch_s (2026-10-31, default off): unusable longer than this -> no sending until FAST-LIO restarts
        # (its first output back at its origin, restart_radius_m) - frames.DegradedLatch
        self.latch = DegradedLatch(float(p('degraded_latch_s', 0.0).value), float(p('restart_radius_m', 0.5).value))
        log_csv = str(p('log_csv', '').value)
        # holes (2026-10-31, jetson2 flights m1/m2, h1, bench6): FAST-LIO's output reached the bridge with gaps of
        # 3.2-3.5 s (sometimes longer) while FAST-LIO kept its map and state (the recorder in the same container
        # got every message, header stamps continuous). Each gap > lost_after_s threw the alignment away and PX4
        # lost its only aiding -> local_position_invalid failsafes. Now a gap up to hole_max_s is bridged: the
        # last pose is coasted (plus velocity, covariance growing) for up to coast_max_s so PX4's EV aiding stays
        # alive, and the alignment is kept when the output resumes where it should. A restart (back at the
        # origin / header time back), a jump or a longer silence: WAIT and align again at rest, as before.
        self.hole_max = float(p('hole_max_s', 8.0).value)
        self.coast_max = float(p('coast_max_s', 4.0).value)                  # 0 = never coast
        self.coast_after = float(p('coast_after_s', 0.3).value)
        self.coast_pos_growth = float(p('coast_pos_growth', 0.5).value)       # position std growth [m per s coasted]
        self.coast_vel_std = float(p('coast_vel_std', 0.5).value)
        self.intake_reliable = bool(p('intake_reliable', True).value)         # /Odometry, /fast_lio/health: reliable, deep

        frd = self.heading_mode == 'initial'
        self.pose_frame = VehicleOdometry.POSE_FRAME_FRD if frd else VehicleOdometry.POSE_FRAME_NED
        self.vel_frame = VehicleOdometry.VELOCITY_FRAME_FRD if frd else VehicleOdometry.VELOCITY_FRAME_NED
        self.state = WAIT
        self.align = None
        self.aligned_once = False
        self.reset_counter = 0
        self.rest = deque()             # (t_rx, R_N_BD, R_W_I, p_W_I) while resting
        self.R_N_BD, self.att_rx = None, -1.0
        self.lpos, self.lpos_rx = None, -1.0
        self.health, self.health_rx = None, -1.0
        self.att_hist = deque(maxlen=200)    # (t_rx, R_N_BD), (t_rx, lpos): PX4 at the scan time, for the log
        self.lpos_hist = deque(maxlen=200)
        self.last = None                # (stamp, p_W_I, t_rx) of the previous FAST-LIO sample
        self.quality = 0
        self.sent = self.dropped = 0
        self.last_out = None            # (t_rx, t_sample, p_O, R_O_BD, v_O, w_BD, quality) of the last real sample
        self.coasting, self.coast_n, self.coasted, self.holes = False, 0, 0, 0

        out_topic = f'{ns}/fmu/in/vehicle_visual_odometry' if self.send_to_px4 else '~/vehicle_odometry'
        self.pub = self.create_publisher(VehicleOdometry, out_topic, 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        qos = qos_profile_sensor_data
        # FAST-LIO publishes reliable (depth 20); a best-effort intake lost whole bursts on the Jetson (10-31)
        qos_lio = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE) if self.intake_reliable else qos
        self.create_subscription(Odometry, odom_topic, self.on_odom, qos_lio)
        self.create_subscription(Float32MultiArray, health_topic, self.on_health, qos_lio)
        self.create_subscription(Imu, imu_topic, self.on_imu, qos)
        self.create_subscription(VehicleAttitude, f'{ns}/fmu/out/vehicle_attitude', self.on_att, qos)
        self.create_subscription(VehicleLocalPosition, f'{ns}/fmu/out/vehicle_local_position', self.on_lpos, qos)
        self.create_timer(1.0, self.on_status)
        self.create_timer(0.1, self.on_coast)
        self.csv = open(log_csv, 'w', buffering=1) if log_csv else None
        if self.csv:
            self.csv.write(CSV_HEADER + '\n')
        self.get_logger().info(
            f'{odom_topic} -> {self.pub.topic_name} ({"to PX4" if self.send_to_px4 else "shadow, PX4 untouched"}); '
            f'heading {self.heading_mode}, stamps {self.stamp_mode}; IMU in body: '
            f'rpy {np.round(rot_to_rpy_deg(self.ext.R_BF_I), 2).tolist()} deg, '
            f'xyz {np.round(self.ext.p_BF_I, 3).tolist()} m')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- inputs ----
    def on_imu(self, msg):
        self.clock.add(self.now_s(), stamp_s(msg.header.stamp))

    def on_att(self, msg):
        self.R_N_BD = quat_to_rot(*[float(v) for v in msg.q])
        self.att_rx = self.now_s()
        self.att_hist.append((self.att_rx, self.R_N_BD))

    def on_lpos(self, msg):
        self.lpos = msg
        self.lpos_rx = self.now_s()
        self.lpos_hist.append((self.lpos_rx, msg))

    def on_health(self, msg):
        if len(msg.data) >= 3:
            # effective (matched) points, mean residual, downsampled input points
            self.health = (float(msg.data[0]), float(msg.data[2]), float(msg.data[1]))
            self.health_rx = self.now_s()

    def on_odom(self, msg):
        now = self.now_s()
        t = stamp_s(msg.header.stamp)
        q, pos = msg.pose.pose.orientation, msg.pose.pose.position
        lin, ang = msg.twist.twist.linear, msg.twist.twist.angular
        R_W_I = quat_to_rot(q.w, q.x, q.y, q.z)
        p_W_I = np.array([pos.x, pos.y, pos.z])
        v_I = np.array([lin.x, lin.y, lin.z])
        w_I = np.array([ang.x, ang.y, ang.z])

        if self.last is not None:
            dt = t - self.last[0]
            gap = now - self.last[2]
            back = dt < -0.5
            jump = dt > 0 and np.linalg.norm(p_W_I - self.last[1]) / dt > self.max_speed
            # back at its origin after a silence while it had been away: FAST-LIO restarted
            restarted = (gap > self.lost_after and np.linalg.norm(p_W_I) <= self.latch.restart_radius
                         and np.linalg.norm(self.last[1]) > 2.0 * self.latch.restart_radius)
            if back or jump or restarted or gap > self.hole_max:
                why = ('restarted' if back or restarted else 'jumped' if jump else f'silent {gap:.1f} s (over hole_max_s)')
                self.get_logger().warn(f'FAST-LIO {why} - stop sending, align again at rest')
                self.state, self.align = WAIT, None
                self.rest.clear()
                self.coasting, self.last_out = False, None     # the held sample belongs to the old alignment
                if not jump and self.latch.output_resumed(p_W_I):
                    self.get_logger().info('FAST-LIO back at its origin (restarted) - degraded latch cleared')
            elif gap > self.lost_after:
                # a hole in the output, not a restart: its world is the one we aligned to - carry on
                self.holes += 1
                self.latch.bad_since = None
                self.get_logger().warn(f'FAST-LIO output resumed after a {gap:.1f} s hole (coasted {self.coast_n} samples)'
                                       f' - alignment kept')
        self.coasting, self.coast_n = False, 0
        self.last = (t, p_W_I, now)

        if self.state != RUN:
            self.collect_rest(now, R_W_I, p_W_I, v_I, w_I)
            return
        self.send(now, t, R_W_I, p_W_I, v_I, w_I)

    # ---- alignment at rest ----
    def collect_rest(self, now, R_W_I, p_W_I, v_I, w_I):
        why = []
        if self.R_N_BD is None or now - self.att_rx > 0.2:
            why.append('no PX4 attitude')
        if np.linalg.norm(w_I) > self.rest_gyro:
            why.append(f'gyro {np.linalg.norm(w_I):.3f} rad/s')
        if np.linalg.norm(v_I) > self.rest_speed:
            why.append(f'speed {np.linalg.norm(v_I):.3f} m/s')
        if self.rest and not why:
            _, R0, Rw0, p0 = self.rest[0]
            moved = (np.linalg.norm(p_W_I - p0), angle_between(Rw0, R_W_I), angle_between(R0, self.R_N_BD))
            if moved[0] > self.rest_spread_m or max(moved[1:]) > self.rest_spread_rad:
                why.append(f'moved {moved[0]:.3f} m / {math.degrees(moved[1]):.2f} deg (FAST-LIO), '
                           f'{math.degrees(moved[2]):.2f} deg (PX4)')
        if why:
            if self.rest:
                self.get_logger().info(f'alignment restarted: {", ".join(why)}', throttle_duration_sec=1.0)
            self.rest.clear()
            self.state = WAIT
            self.last_out = None
            return
        self.state = REST
        self.rest.append((now, self.R_N_BD.copy(), R_W_I, p_W_I))
        if now - self.rest[0][0] < self.rest_window or len(self.rest) < 5:
            return

        p0, p0_src = np.zeros(3), 'origin'
        lp = self.lpos
        if self.align_to_px4_pos and lp is not None and now - self.lpos_rx < 0.5 and lp.xy_valid and lp.z_valid:
            p0, p0_src = np.array([lp.x, lp.y, lp.z]), 'PX4 local position'
        _, R_N_BD, R_W_I_rest, p_W_I_rest = zip(*self.rest)
        self.align = Alignment.solve(self.ext, R_N_BD, R_W_I_rest, p_W_I_rest, p0, self.heading_mode, self.heading)
        if self.aligned_once:
            self.reset_counter = (self.reset_counter + 1) % 256
        self.aligned_once = True
        self.state = RUN
        self.rest.clear()
        self.last_out = None
        self.get_logger().info(
            f'aligned ({len(R_N_BD)} samples at rest): PX4 rpy {np.round(self.align.rest_rpy_deg, 1).tolist()} deg, '
            f'start {np.round(p0, 2).tolist()} m ({p0_src}), reset_counter {self.reset_counter}')

    # ---- output ----
    def sample_time(self, t, now):
        if self.stamp_mode == 'header':
            return t
        off = self.clock.offset() if self.stamp_mode == 'auto' else None
        if off is None:
            if self.stamp_mode == 'auto':
                self.get_logger().warn('no IMU stamps for the clock offset yet - using receipt time',
                                       throttle_duration_sec=5.0)
            return now - self.receipt_latency
        return t + off

    def health_quality(self, now):
        if self.health is None or now - self.health_rx > 0.5:
            return (0, None, None, None) if self.health_required else (50, None, None, None)
        eff, res, down = self.health
        q = quality_from_health(eff, res, self.eff_min, self.eff_good, self.residual_max, down, self.sparse)
        return q, eff, res, down

    def send(self, now, t, R_W_I, p_W_I, v_I, w_I):
        p_O, R_O_BD, v_O, w_BD = body_state(self.align, self.ext, R_W_I, p_W_I, v_I, w_I)
        self.quality, eff, res, down = self.health_quality(now)
        t_sample = min(self.sample_time(t, now), now)
        held = self.latch.update(now, self.quality)
        ok = self.quality > 0 and not held
        if ok:
            nan3 = [math.nan] * 3
            m = VehicleOdometry()
            m.timestamp = int(now * 1e6)
            m.timestamp_sample = int(t_sample * 1e6)
            m.pose_frame = self.pose_frame
            m.position = np.asarray(p_O, dtype=np.float32)
            m.q = np.asarray(rot_to_quat(R_O_BD), dtype=np.float32)
            m.velocity_frame = self.vel_frame
            m.velocity = np.asarray(v_O if self.send_velocity and v_O is not None else nan3, dtype=np.float32)
            m.angular_velocity = np.asarray(w_BD if w_BD is not None else nan3, dtype=np.float32)
            m.position_variance = np.full(3, self.var_pos, dtype=np.float32)
            m.orientation_variance = np.full(3, self.var_att, dtype=np.float32)
            m.velocity_variance = np.full(3, self.var_vel, dtype=np.float32)
            m.reset_counter = self.reset_counter
            m.quality = self.quality
            self.pub.publish(m)
            self.sent += 1
        else:
            self.dropped += 1
            if held:
                self.get_logger().warn(f'FAST-LIO was unusable over {self.latch.hold_s} s - not sending until it '
                                       f'restarts (effective points {eff})', throttle_duration_sec=2.0)
            else:
                self.get_logger().warn(f'FAST-LIO unhealthy (effective points {eff}, residual {res}) - not sending',
                                       throttle_duration_sec=2.0)
        self.last_out = (now, t_sample, p_O, R_O_BD, v_O, w_BD, self.quality if ok else 0)
        if self.csv:
            self.log_row(now, t_sample, ok, eff, res, p_O, v_O, R_O_BD, down)

    def log_row(self, now, t_sample, ok, eff, res, p_O, v_O, R_O_BD, down=None, coast=0):
        lp, att = nearest(self.lpos_hist, t_sample), nearest(self.att_hist, t_sample)
        px4 = ([lp[0] - t_sample, lp[1].x, lp[1].y, lp[1].z, lp[1].vx, lp[1].vy, lp[1].vz,
                int(lp[1].xy_valid), int(lp[1].z_valid)] if lp else [math.nan] * 9)
        px4_rpy = rot_to_rpy_deg(att[1]) if att else [math.nan] * 3
        v = v_O if v_O is not None else [math.nan] * 3
        row = [now, t_sample, int(ok), self.quality, eff, res, *p_O, *v, *rot_to_rpy_deg(R_O_BD), *px4, *px4_rpy, down, coast]
        self.csv.write(','.join('' if x is None else f'{x:.6f}' if isinstance(x, float) else str(x) for x in row)
                       + '\n')

    def on_coast(self):
        """10 Hz: while FAST-LIO's output is missing, keep PX4 aided with the last pose moved by its velocity and a
        covariance that grows with the silence (coast_pos_growth), up to coast_max_s."""
        if self.state != RUN or self.last_out is None or self.coast_max <= 0:
            return
        now = self.now_s()
        t_rx, t_s, p_O, R_O_BD, v_O, w_BD, q = self.last_out
        age = now - t_rx
        if age < self.coast_after or q <= 0 or self.latch.latched:
            return
        if age > self.coast_max:
            if self.coasting:
                self.coasting = False
                self.get_logger().warn(f'FAST-LIO silent {age:.1f} s - coasting stopped, PX4 EV times out now')
            return
        if not self.coasting:
            self.coasting = True
            self.get_logger().warn(f'FAST-LIO silent {age:.2f} s - coasting the last pose (velocity {np.round(v_O, 2).tolist() if v_O is not None else None} m/s)')
        v = np.asarray(v_O if v_O is not None else np.zeros(3), float)
        p = np.asarray(p_O, float) + v * age
        sd_pos = math.sqrt(self.var_pos) + self.coast_pos_growth * age
        nan3 = [math.nan] * 3
        m = VehicleOdometry()
        m.timestamp = int(now * 1e6)
        m.timestamp_sample = int(min(t_s + age, now) * 1e6)
        m.pose_frame = self.pose_frame
        m.position = np.asarray(p, dtype=np.float32)
        m.q = np.asarray(rot_to_quat(R_O_BD), dtype=np.float32)
        m.velocity_frame = self.vel_frame
        m.velocity = np.asarray(v if self.send_velocity and v_O is not None else nan3, dtype=np.float32)
        m.angular_velocity = np.asarray(nan3, dtype=np.float32)
        m.position_variance = np.full(3, sd_pos ** 2, dtype=np.float32)
        m.orientation_variance = np.full(3, 4.0 * self.var_att, dtype=np.float32)
        m.velocity_variance = np.full(3, self.coast_vel_std ** 2, dtype=np.float32)
        m.reset_counter = self.reset_counter
        m.quality = max(1, q // 2)
        self.pub.publish(m)
        self.coasted += 1
        self.coast_n += 1
        if self.csv:
            self.log_row(now, t_s + age, 1, None, None, p, v if v_O is not None else None, R_O_BD, None, coast=1)

    def on_status(self):
        off = self.clock.offset()
        status = {'state': self.state, 'to_px4': self.send_to_px4, 'sent': self.sent, 'dropped': self.dropped,
                  'holes': self.holes, 'coasting': self.coasting, 'coasted': self.coasted,
                  'quality': self.quality, 'reset_counter': self.reset_counter, 'latched': self.latch.latched,
                  'clock_offset_s': None if off is None else round(off, 4),
                  'px4_attitude': self.R_N_BD is not None and self.now_s() - self.att_rx < 0.5}
        if self.R_N_BD is not None:
            status['px4_yaw_deg'] = round(math.degrees(yaw_of(self.R_N_BD)), 1)
        self.status_pub.publish(String(data=json.dumps(status)))

    def destroy_node(self):
        if self.csv:
            self.csv.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = LioPx4Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
