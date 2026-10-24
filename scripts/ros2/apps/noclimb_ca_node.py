#!/usr/bin/env python3
"""No-climb collision-avoidance policy -> PathFollowing (CA-PF) for this SITL.

Runs the learned avoidance policy of ros2_ws/src/Inhouse-collision-avoidance/noclimb_onnx
through its reference implementation (ca_policy_ref.NoClimbPolicy, unmodified: observation,
output -> WP/speed, speed governor, policy state) at 20 Hz and hands every step to
PathFollowing as a short waypoint path plus a speed command:

  /odom  (adapter: world ENU = PX4 origin, FLU body)        -> pos, vel, R_wb
  /cloud (adapter: world ENU, AirSim-registered, de-skewed)  -> latest scan (or the last
          scan_memory_s seconds) - pos (world axes)
      NoClimbPolicy.step -> WP 3 m ahead (never above the path altitude) + speed
  -> /local_waypoint_setpoint_to_PF  [P0, WP, WP + 2 (WP - P0), WP + 4 (WP - P0)], P0 = (x, y, WP z)
     /desired_speed                   policy speed, capped near the goal

cmd_tau_s > 0 smooths the horizontal command (speed * unit(WP - pos)) with that first-order lag
before it goes to PF. The policy's actions jump from step to step; in training the follower's
own lag (0.5-1.2 s) absorbed that, while PF and the iris follow within ~0.2 s and passed the
jumps on as wobble (roll-setpoint rate ~100 deg/s rms vs ~35 under SUPER).
     /path_planning_heartbeat, /collision_avoidance_heartbeat (10 Hz; PF gates on both)

The policy was trained with a follower that flies speed * unit(WP - pos) horizontally and
3 * (WP z - z) vertically. PathFollowing (guid_type 0 speed mode) is driven to do the same:
P0 sits at the vehicle's xy but at the WP altitude, so an altitude error is a cross-track
error PF closes; the extension past the WP keeps PF's own end-of-path brake (sqrt(2 a d))
from clipping the policy's 5 m/s and keeps PF from declaring done mid-flight.

Global path: straight line from the vehicle position when CA engages to the goal, at the
path altitude. CA engages once flight_manager hands control to PF (arm + 10 s: climb and
hover phases) and the vehicle is at the path altitude - before that nothing is published,
so PF stays idle and the policy's stall timer does not run while the vehicle is held.
With flight_driver (goal_topic, the vehicle) the driver's goal message is the signal instead,
and /noclimb/ready (pose, lidar driver and /cloud live) is what lets the driver arm.
Within final_switch_m of the goal the policy stops (training ended 0.5 m from the goal,
agent.md section 8) and PF gets a fixed path to the goal, flown like the SUPER bridge's
final approach (goal brake sqrt(2 * 2 * d) >= 0.5 m/s, 0.5 m/s inside 1.5 m); PF declares
done there and flight_manager lands.

Obstacle density (observation 207) was simulator truth in training ("obstacles within 4 m
of the surface / 4"). density:=cluster estimates it from the scan (connected occupied
cells of a 0.5 m horizontal grid among returns within 4 m); a number uses that constant.

Lidar watchdog: no new scan from the lidar driver (lidar_topic, by header stamp) for
scan_timeout_s or no /cloud for cloud_timeout_s -> hold (speed 0, policy not stepped, mode
BLIND); blind for blind_land_s -> /path_following_complete, i.e. flight_manager lands in place
(or finishes at the goal when within its goal_pf_done_max_m).
"""
import csv
import math
import os
import struct
import sys
import time
from collections import deque

import numpy as np
import rclpy
from custom_msgs.msg import LocalWaypointSetpoint
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from px4_msgs.msg import VehicleStatus
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32
import sensor_msgs_py.point_cloud2 as pc2


def quat_to_rot(qw, qx, qy, qz):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def count_obstacles(rel, z_world, radius=4.0, cell=0.5, ground_z=0.3):
    """Objects within `radius` of the vehicle: 8-connected occupied cells of a horizontal
    grid over the returns closer than `radius` (ground returns dropped)."""
    m = (np.einsum('ij,ij->i', rel, rel) <= radius * radius) & (z_world > ground_z)
    if not m.any():
        return 0
    n = int(math.ceil(2 * radius / cell))
    ij = np.floor((rel[m, :2] + radius) / cell).astype(int).clip(0, n - 1)
    occ = np.zeros((n, n), dtype=bool)
    occ[ij[:, 0], ij[:, 1]] = True
    seen = np.zeros_like(occ)
    count = 0
    for i0, j0 in zip(*np.nonzero(occ)):
        if seen[i0, j0]:
            continue
        count += 1
        seen[i0, j0] = True
        stack = [(i0, j0)]
        while stack:
            i, j = stack.pop()
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    a, b = i + di, j + dj
                    if 0 <= a < n and 0 <= b < n and occ[a, b] and not seen[a, b]:
                        seen[a, b] = True
                        stack.append((a, b))
    return count


class NoClimbCA(Node):
    def __init__(self):
        super().__init__('noclimb_ca')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        policy_dir = p('policy_dir', '')
        policy = p('policy', 'ca_noclimb_tau05-12')
        self.goal = np.array([float(p('goal_x', 100.0)), float(p('goal_y', 0.0))])
        self.alt = float(p('altitude', 4.0))
        self.engage_after_arm = float(p('engage_after_arm_s', 11.0))
        # shadow: the whole onboard chain on the vehicle without flying it (carried towards real
        # obstacles before the first avoidance flight). Engages without arming, at the current height,
        # with the goal shadow_goal_ahead_m along the current heading (0 = goal_x/goal_y); every output
        # goes to /shadow/... so PathFollowing, flight_manager and PX4 never act on it. A Bool on
        # /shadow/reset starts a new run (point the nose at the course first). /noclimb/path (always)
        # shows the commanded path in the world frame.
        self.shadow = bool(p('shadow', False))
        self.shadow_ahead = float(p('shadow_goal_ahead_m', 30.0))
        # shadow_mask_rear_m: whoever carries the vehicle walks right behind it - returns behind the nose
        # closer than this (horizontally) are left out, or the policy would steer away from the carrier
        self.shadow_mask = float(p('shadow_mask_rear_m', 1.5))
        # shadow_carry_height_m: at each engagement the ground is taken this far below the carried vehicle's
        # centre (and the policy's heights from it) - PX4's GPS-referenced z drifted 11 m in minutes after a
        # power-up on the X500 and was reset 8 times, which moved every return below the start's ground. 0 = off.
        self.shadow_carry = float(p('shadow_carry_height_m', 1.2))
        # voxel_m: keep the return nearest to the vehicle in each voxel_m cube, per scan and over the scan memory.
        # The real Mid-360 indoors left 57k points in memory and a 72 ms step (13 Hz) on the Jetson; the image
        # bins are >= 0.2 m wide past 1 m, so 0.1 m changes no bin's nearest return by more than 0.09 m. 0 = off.
        self.voxel = float(p('voxel_m', 0.0))
        self.final_switch = float(p('final_switch_m', 3.0))
        # pf_direct: PathFollowing flies the straight line to the goal alone (mode PFDIRECT; the policy keeps
        # stepping so its inputs stay current, its output is not used) once that line is clear - no return within
        # pf_direct_corridor_m of it at the vehicle's height (+-pf_direct_dz_m) for pf_direct_on_m or up to the
        # goal, for pf_direct_hold_s on end - and the vehicle already flies along it (velocity within
        # pf_direct_align_deg of it). A return within pf_direct_off_m of the line ahead hands back to the policy
        # (the on/off gap keeps it from flapping). Where the policy jitters for nothing (open stretches) PF's
        # pursuit holds a straight line within 0.1 m. The path is two points [here, goal] on purpose: PF flies its
        # last segment with pursuit, not MPPI (see the ext/ext2 note in tick), so a corner before the goal means a
        # mid-air controller switch, and a line that does not match the velocity at the switch rolls the vehicle
        # (C-Track 10-22: 12 deg roll swings from a 10 m rejoin leg and a 21 deg misaligned start).
        self.pf_direct = bool(p('pf_direct', False))
        self.pfd_w = float(p('pf_direct_corridor_m', 1.8))
        self.pfd_on = float(p('pf_direct_on_m', 25.0))
        self.pfd_off = float(p('pf_direct_off_m', 18.0))
        self.pfd_hold = float(p('pf_direct_hold_s', 1.0))
        self.pfd_align = math.radians(float(p('pf_direct_align_deg', 2.5)))
        self.pfd_dz = float(p('pf_direct_dz_m', 1.0))
        self.t_pfd_clear = None
        self.pfd_path = None
        self.pfd_steps = 0
        self.goal_brake = float(p('goal_brake_accel', 2.0))
        # flight_manager's goal handover radius (PX4 position control inside it; 0 = none): brake
        # to handover_speed at that circle, not at the goal - at 5 m/s the goal-referenced brake
        # (3.6 m/s cap at 3.2 m, tracking lag on top) reached it at 4.8 m/s and PX4 overshot the
        # goal by 1.7 m with 24 deg of roll and a 0.5 m altitude bump.
        self.handover = float(p('goal_handover_m', 0.0))
        self.handover_speed = float(p('handover_speed', 1.0))
        self.arrive_radius = float(p('arrive_radius_m', 1.5))
        self.arrive_speed = float(p('arrive_speed', 0.5))
        self.ext_factor = float(p('path_extension', 2.0))
        self.cmd_tau = float(p('cmd_tau_s', 0.0))
        # with cmd_tau_s: pass speed decreases through at once (direction and speed-ups stay
        # smoothed). x8_nc_mppi_c: the policy stopped (0 m/s) 3.2 m before the canopy, the 0.3 s
        # smoothing turned it into 2.7 -> 1.9 m/s and the 10 kg X8 braked a second late.
        self.brake_now = bool(p('cmd_brake_immediate', False))
        # Command shaping rule on top of cmd_tau_s (0 = off): the change the smoothed command asks for
        # in one step is an acceleration, capped at cmd_accel_max [m/s^2] (tilt ~ atan(a/g)), and it may
        # change at most cmd_jerk_max [m/s^3] per second (tilt rate ~ jerk / g). A longer cmd_tau_s
        # delayed every turn alike (0.5 s passed C2 at +0.07 m) and left the trained 0.3 s; the caps
        # only clip the abrupt part. Speed decreases still bypass it (cmd_brake_immediate) and the
        # safety filter acts after it.
        self.cmd_accel_max = float(p('cmd_accel_max', 0.0))
        self.cmd_jerk_max = float(p('cmd_jerk_max', 0.0))
        self.a_s = np.zeros(2)          # acceleration of the shaped command
        # ...applied by obstacle distance (cmd_fade_near_m < cmd_fade_far_m, 0 = everywhere): none while the
        # policy image's nearest return is closer than near, all of them beyond far, in between they
        # widen (cap / fraction). Everywhere, a4 j8 slowed the turn at C2 into a contact (ctrack_a4j8_1);
        # 56 % of the fast tilting happened with the nearest obstacle 4-10 m away. Faded 4/6 m, a4 j8
        # kept the C2 turn (5 of 5) but hardly engaged: fast tilting 15.4 -> 14.0 %.
        self.cmd_fade_near = float(p('cmd_fade_near_m', 0.0))
        self.cmd_fade_far = float(p('cmd_fade_far_m', 0.0))
        # cmd_tau_near_s (< 0 = cmd_tau_s everywhere): the smoothing inside cmd_fade_near_m, blending to
        # cmd_tau_s at cmd_fade_far_m. 0.7 s everywhere halved the fast tilting (15.4 -> 6.8 %) but, as
        # cmd_brake_immediate passes speed decreases only, the command heading trailed the policy's by
        # > 30 deg for 26.8 % of the time within 4 m of a return (0.3 s: 9.6 %) and a swerve around C2
        # came a second late (ctrack_tau7_7, prop tips 0.35 m into the pole; 1 of 8).
        self.cmd_tau_near = float(p('cmd_tau_near_s', -1.0))
        # Contact: the nearest return at the vehicle's height (the safety filter's, 3 s memory) closer
        # than contact_dist_m (the prop tips; 0 = off) for contact_time_s -> hold and land in place.
        # ctrack_a4j8_1 touched C2 at 1.6 m/s - under the impact check's 30 m/s^2 - and stayed wedged
        # 0.4 m from it for 390 s.
        self.contact_dist = float(p('contact_dist_m', 0.0))
        self.contact_time = float(p('contact_time_s', 1.0))
        self.t_contact = None
        self.hold_reason = None
        # Vehicle larger than the one the policy was trained for (d_crash 0.6 m, ~0.35 m class):
        # every occupied range-image cell is moved this much closer [m], so the policy keeps its
        # learned clearance from the bigger vehicle's outline (the speed governor, which reads
        # the image, slows down for it too). X8: prop tips at 0.77 m. 0 = off.
        self.inflate = float(p('range_inflate_m', 0.0))
        # Safety velocity filter outside the policy (0 = off): the commanded horizontal velocity
        # may approach a lidar point at most at gain * (distance - radius) [m/s] (inside the
        # radius: back away), per 10 deg bearing sector, over the returns of the last
        # safety_memory_s within the vehicle's height band. Sideways/away motion is untouched.
        # For a vehicle bigger than the policy's: at a tree gap too narrow for the X8 the
        # policy, already at 'crash' image distances, crept on at its 0.6 m/s floor into the
        # canopy (x8_nc_mppi_b); the memory covers returns the lidar (0.3 m up, -7 deg) loses
        # at close range.
        self.safe_r = float(p('safety_radius_m', 0.0))
        self.safe_k = float(p('safety_gain', 1.0))
        self.safe_mem = float(p('safety_memory_s', 3.0))
        self.safe_dz = (float(p('safety_dz_below_m', 0.5)), float(p('safety_dz_above_m', 0.6)))
        # Height of the WP sent to PF moved at most this fast toward the policy's [m/s] (0 = off;
        # < 0 = the policy's ctrl_vz_max if it may climb (cfg wp_alt_up > 0), else off). The
        # height action is nearly bang-bang (|a1| < 0.3 in 1 % of the steps): with climbing allowed
        # (ca_big, +6 m) a +1 for a step or two put the WP up to 6 m higher and PF chased it - in
        # the forest 0.6 WP height jumps > 1 m per second, pitch-rate jitter 26 deg/s and thrust
        # jitter 0.10 (the no-climb policy: 8-10 deg/s, 0.02); on C-Track at 1.5 m a climb /
        # descend cycle with no horizontal progress until the stall input saturated and the
        # policy stopped (ctrack_big_a). The policy was trained with a follower whose vertical
        # speed is capped at ctrl_vz_max (1.5 m/s).
        self.wp_z_rate = float(p('wp_z_rate', -1.0))
        self.wp_z = None                # rate-limited WP height (from the vehicle's at engagement)
        # Lidar watchdog (a timeout of 0 = that check off). ctrack_big_e: Unreal crashed 4.8 s in,
        # the lidar stopped and the policy kept flying the last scan - cropped to 10.5 m around
        # where it was taken, so soon an empty image - straight into the wall at 5 m/s. Normal
        # gaps: new driver scans every 0.1 s (0.21 s max), /cloud up to 1.3 s (the adapter's tilt
        # gate drops scans while braking hard), hence the driver's stamps for a quick verdict.
        # Speed governor sector [deg either side of the commanded direction]: the reference
        # governor (training rule) caps the policy's speed by the closest cell of the whole 360 deg
        # image, so obstacles beside and behind - already passed - keep the vehicle crawling: in
        # ctrack_big_c/i the governor held the X8 (image 0.8 m closer) below the policy's 5 m/s for
        # 7.5 s of 23, the closest cell behind (> 100 deg) for 4 s of it (C1: from -25 to -152 deg,
        # 2.8 s near 0.8 m/s). < 180: the same law over the cells within this sector only (the
        # safety filter still limits closing in from the side). 180 = the reference governor;
        # 0 = no governor (the policy's own speed - it learned with the governor, test only).
        self.gov_sector = float(p('governor_sector_deg', 180.0))
        # Corridor governor [m] (> 0: replaces the sector): the law on the along-track distance
        # to the closest return within this half-width of the commanded line or of the current
        # velocity line (at the vehicle's height, safety_dz), less range_inflate_m. ctrack_big_j:
        # the policy steered past C1 / C5 with the pole 1.2-2.0 m off its line and kept 5 m/s,
        # yet the +-90 deg governor braked 5 -> 0.6 m/s as the pole came alongside and let go at
        # 90 deg - dodge, brake, dodge, brake. A pole off the line does not cap the speed; the
        # safety filter (safety_radius_m) still bounds closing in from the side.
        self.gov_corridor = float(p('governor_corridor_m', 0.0))
        # Braking-distance law for the corridor governor (governor_brake_decel > 0 [m/s^2]): a
        # return in the path is a circle of radius governor_corridor_m, s = distance along the line
        # to where it enters that circle, and the speed is capped so that v * delay + v^2 / (2 a)
        # <= s (super_speed_guard.py's law). The reference law (floor at d_img 0.8 = 1.6 m with the
        # X8's 0.8 m) only starts below 3.6 m and drops 5 -> 0.6 m/s within ~2 m: the hard brakes
        # of ctrack_big_j, and once side poles no longer slowed it, too late (big_k, C5 +0.07 m).
        self.brake_decel = float(p('governor_brake_decel', 0.0))
        self.brake_delay = float(p('governor_brake_delay_s', 0.6))
        # Ground and height reference (bundles trained on height above ground, ground removed from
        # the image - x8_onnx_r2 agent.md 5 and 6): ground_z_m = the ground's z in this ENU frame
        # (NaN = keep every return; returns below ground_z_m + ground_clear_m leave the policy's
        # points), agl_offset_m = height of this frame's origin above the ground (PX4's origin is
        # the vehicle centre at rest: X8 0.44 m) - added to z going into the policy, taken off
        # the WP coming out.
        self.ground_z = float(p('ground_z_m', float('nan')))
        self.ground_clear = float(p('ground_clear_m', 0.25))
        self.agl_off = float(p('agl_offset_m', 0.0))
        # ref_from_start: the heights and the goal are relative to where the vehicle rests when this node
        # comes up (on the ground) - on the vehicle PX4's local origin is not there: the X500's logs put
        # the vehicle at rest 13-45 m off z 0 (EKF2_HGT_REF GPS). Off in SITL (origin = the spawn).
        self.ref_from_start = bool(p('ref_from_start', False))
        self.z0 = None
        # goal_topic (flight_driver - the vehicle, and SILS / PILS with NOCLIMB_DRIVER=flight_driver): the driver
        # sends the goal once the vehicle hovers in OFFBOARD - PointStamped, PX4 local ENU, z = the path height -
        # and that is the signal to engage (no engage_after_arm_s, no height check). The heights are referenced
        # to the driver's take-off point from it (rest height = z - altitude) instead of ref_from_start's first
        # pose: PX4's z drifted 11 m in minutes on the ground on the X500. '' = goal_x / goal_y / altitude.
        self.goal_topic = str(p('goal_topic', ''))
        self.goal_rx = False
        self.alt_param, self.ground_z_param, self.agl_param = self.alt, self.ground_z, self.agl_off
        self.lidar_topic = str(p('lidar_topic', ''))
        self.scan_timeout = float(p('scan_timeout_s', 0.5))
        self.cloud_timeout = float(p('cloud_timeout_s', 2.0))
        self.blind_land = float(p('blind_land_s', 5.0))
        self.raw_stamp = None
        self.t_raw = self.t_cloud = self.t_engage = self.t_blind = None
        self.land_requested = False
        self.v_s = np.zeros(2)          # smoothed horizontal velocity command (cmd_tau_s)
        # Scan memory [s]: build the image from the world-registered returns of the last
        # memory_s seconds instead of the latest scan only (0, the default: the policy learned
        # on an instantaneous analytic sensor). Meant for the lidar sitting 0.3 m above the
        # body centre - its -7 deg lower FOV edge hides anything at body height closer than
        # ~2.4 m - and for thin branches only some scans hit. Flown with 2 s it showed no
        # benefit (2 of 4 runs reversed into the tree gap at x 19-22 vs 0 of 4 without; a
        # replay of the same states with memory never flipped the policy's turn direction).
        self.memory_s = float(p('scan_memory_s', 0.0))
        # ...but always keep the last scan_memory_min_scans scans up to scan_memory_max_s old: the
        # window counts from the newest scan, so a gap (the tilt gate dropped 0.72 s while the X8
        # turned, ctrack_mem_spd_on_3) wiped it and the next, tilted scans without the pole made it
        # vanish (contact at C1). 0 = the time window alone.
        self.memory_min_scans = int(p('scan_memory_min_scans', 0))
        self.memory_max_s = float(p('scan_memory_max_s', 1.5))
        density = str(p('density', 'cluster'))
        self.density_const = None if density == 'cluster' else float(density)
        log_csv = p('log_csv', '')
        # replay dump (written at disarm): every scan as used (receive time, world points
        # within range) and every policy step (state, observation, action) - lets the image
        # construction be varied offline on the exact flown states
        self.dump_path = p('dump_npz', '')
        self.dump_scans, self.dump_steps = [], []

        # ca_policy_ref opens a plain CPU session; keep onnxruntime to one non-spinning
        # thread (a 216-512-256-128-3 MLP at 20 Hz) so it does not compete with Gazebo.
        import onnxruntime as ort
        _session = ort.InferenceSession

        def _small_session(path, sess_options=None, providers=None, **kw):
            so = sess_options or ort.SessionOptions()
            so.intra_op_num_threads = 1
            so.inter_op_num_threads = 1
            so.add_session_config_entry('session.intra_op.allow_spinning', '0')
            return _session(path, sess_options=so, providers=providers, **kw)

        ort.InferenceSession = _small_session
        sys.path.insert(0, policy_dir)
        from ca_policy_ref import NoClimbPolicy, pointcloud_to_image
        self.p2i = pointcloud_to_image
        self.onnx = os.path.join(policy_dir, policy + '.onnx')
        self.cfg_json = os.path.join(policy_dir, 'cfg.json')
        # placeholder path until engagement (reset() there with the real start point)
        self.pol = NoClimbPolicy(self.onnx, self.cfg_json, [[0.0, 0.0], self.goal.tolist()],
                                 [self.alt, self.alt])
        self.d_max = float(self.pol.c['d_sensor_max'])
        if self.wp_z_rate < 0.0:
            c = self.pol.c
            self.wp_z_rate = float(c.get('ctrl_vz_max', 1.5)) if float(c.get('wp_alt_up', 0.0)) > 0.0 else 0.0

        self.pos = self.vel = self.R = None
        self.yaw = 0.0
        self.cloud = None
        self.cloud_n = 0
        self.scans = deque()
        self.safe_scans = deque()
        self.safe_dmin = float('inf')
        self.safe_cut = 0.0
        self.t_armed = None
        self.disarmed_after_flight = False
        self.mode = 'WAIT'          # WAIT -> CA (<-> PFDIRECT with pf_direct) -> FINAL
        self.final_path = None
        self.last_speed = 0.0

        wp_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        pre = '/shadow' if self.shadow else ''
        self.wp_pub = self.create_publisher(LocalWaypointSetpoint, pre + '/local_waypoint_setpoint_to_PF', wp_qos)
        self.speed_pub = self.create_publisher(Float32, pre + '/desired_speed', 10)
        self.pp_hb = self.create_publisher(Bool, pre + '/path_planning_heartbeat', 10)
        self.ca_hb = self.create_publisher(Bool, pre + '/collision_avoidance_heartbeat', 10)
        self.path_vis = self.create_publisher(Path, '/noclimb/path', 1)
        if self.shadow:
            self.create_subscription(Bool, '/shadow/reset', self.shadow_reset, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/cloud', self.cloud_cb, qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb,
                                 qos_profile_sensor_data)
        if self.lidar_topic:            # serialized: only the header stamp is read (100 Hz, 6k points)
            self.create_subscription(PointCloud2, self.lidar_topic, self.raw_cb, qos_profile_sensor_data,
                                     raw=True)
        self.done_pub = self.create_publisher(Bool, pre + '/path_following_complete', 10)
        if self.goal_topic:
            self.create_subscription(PointStamped, self.goal_topic, self.goal_cb, 10)
        # /noclimb/ready (1 Hz): pose, lidar driver and registered returns all live - flight_driver arms only on it
        self.ready_pub = self.create_publisher(Bool, '/noclimb/ready', 10)
        self.create_timer(1.0, self.ready)
        self.create_timer(self.pol.dt, self.tick)              # 20 Hz (policy_hz)
        self.create_timer(0.1, self.heartbeat)
        self.create_timer(1.0, self.report)
        self.recent = deque(maxlen=20)

        self.csv = None
        if log_csv:
            self.csv_f = open(log_csv, 'w', buffering=1, newline='')
            self.csv = csv.writer(self.csv_f)
            self.csv.writerow(['t', 'mode', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'yaw', 'ref_bearing',
                               's', 'cross', 'remaining', 'vdev', 'd_img_min', 'density', 'n_obj',
                               'a0', 'a1', 'a2', 'wp_x', 'wp_y', 'wp_z', 'speed_policy',
                               'speed_cmd', 'stall_t', 'n_pts', 'd_goal', 'wp_cmd_x', 'wp_cmd_y',
                               'safe_dmin', 'safe_cut', 'wp_cmd_z', 'd_front', 'speed_front', 'tick_ms'])
        self.get_logger().info(
            f'[noclimb_ca] policy {policy} ({self.onnx}), goal ({self.goal[0]:.1f}, '
            f'{self.goal[1]:.1f}) at {self.alt:.1f} m, density '
            f'{"cluster estimate" if self.density_const is None else self.density_const}, WP height rate '
            f'{f"{self.wp_z_rate:.2f} m/s" if self.wp_z_rate > 0.0 else "unlimited"}')

    # ---- inputs ----
    def odom_cb(self, m):
        p, o, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        self.pos = np.array([p.x, p.y, p.z])
        if self.ref_from_start and self.z0 is None and not self.goal_topic:
            self.z0 = float(p.z)
            if np.isfinite(self.ground_z):
                self.ground_z += self.z0
            self.agl_off -= self.z0
            self.alt += self.z0
            self.goal = self.goal + self.pos[:2]
            self.get_logger().info(
                f'[noclimb_ca] reference = the start ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}): ground at z {self.ground_z:.2f}, '
                f'path at z {self.alt:.2f}, goal ({self.goal[0]:.1f}, {self.goal[1]:.1f})')
        self.vel = np.array([v.x, v.y, v.z])                   # adapter twist: world ENU
        self.R = quat_to_rot(o.w, o.x, o.y, o.z)               # FLU body -> ENU world
        self.yaw = math.atan2(self.R[1, 0], self.R[0, 0])

    def voxel_down(self, xyz):
        """the return nearest to the vehicle in each voxel_m cube"""
        if self.voxel <= 0.0 or len(xyz) < 2:
            return xyz
        if self.pos is not None:
            rel = xyz - self.pos
            xyz = xyz[np.argsort(np.einsum('ij,ij->i', rel, rel), kind='stable')]
        k = np.floor(xyz / self.voxel).astype(np.int64) + (1 << 20)      # 21 bits a coordinate: +-100 km
        _, first = np.unique((k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2], return_index=True)
        return xyz[first]

    def cloud_cb(self, m):
        xyz = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = xyz[np.isfinite(xyz).all(axis=1)].astype(np.float64)
        self.cloud_n += 1
        if self.pos is not None and len(xyz):
            # only what can enter the image soon (registration error grows with range)
            rel = xyz - self.pos
            xyz = xyz[np.einsum('ij,ij->i', rel, rel) <= (self.d_max + 0.5) ** 2]
            if self.shadow and self.shadow_mask > 0.0 and len(xyz):
                rel = xyz[:, :2] - self.pos[:2]
                back = rel @ np.array([math.cos(self.yaw), math.sin(self.yaw)]) < 0.0
                xyz = xyz[~(back & (np.einsum('ij,ij->i', rel, rel) < self.shadow_mask ** 2))]
        xyz = self.voxel_down(xyz)
        now = self.now()
        self.t_cloud = now
        if self.dump_path:
            self.dump_scans.append((now, xyz.astype(np.float32)))
        self.scans.append((now, xyz))
        while self.scans and (now - self.scans[0][0] > self.memory_max_s
                              or (now - self.scans[0][0] > self.memory_s and len(self.scans) > self.memory_min_scans)):
            self.scans.popleft()
        if self.safe_r > 0.0:
            self.safe_scans.append((now, xyz))
            while self.safe_scans and now - self.safe_scans[0][0] > self.safe_mem:
                self.safe_scans.popleft()
        self.cloud = self.voxel_down(np.concatenate([s for _, s in self.scans])) if len(self.scans) > 1 else xyz

    def raw_cb(self, data):
        """Lidar driver message, serialized (CDR): 4-byte encapsulation header (byte 1: 1 =
        little endian), then header.stamp (int32 sec, uint32 nanosec). The AirSim bridge
        republishes the latest scan at ~100 Hz; only a new stamp is a new scan."""
        if len(data) < 12:
            return
        stamp = struct.unpack_from('<iI' if data[1] == 1 else '>iI', data, 4)
        if stamp != self.raw_stamp:
            self.raw_stamp = stamp
            self.t_raw = self.now()

    def status_cb(self, m):
        armed = m.arming_state == VehicleStatus.ARMING_STATE_ARMED
        now = self.now()
        if armed and self.t_armed is None:
            self.t_armed = now
            self.get_logger().info('[noclimb_ca] armed - CA engages after the take-off/hover phases')
        elif not armed and self.mode != 'WAIT' and not self.disarmed_after_flight and not self.shadow:
            # (shadow: never armed - a disarmed vehicle is the normal state, not the end of a flight)
            self.disarmed_after_flight = True
            self.get_logger().info('[noclimb_ca] disarmed - stopping')
            self.write_dump()

    def goal_cb(self, m):
        """the flight driver's goal (goal_topic): sets the goal and the height reference until engaged"""
        if self.mode != 'WAIT':
            return
        z0 = float(m.point.z) - self.alt_param           # the vehicle's rest height: the driver's take-off point
        self.goal = np.array([float(m.point.x), float(m.point.y)])
        self.alt = float(m.point.z)
        self.ground_z = self.ground_z_param + z0          # (NaN stays NaN: ground kept)
        self.agl_off = self.agl_param - z0
        if not self.goal_rx:
            self.get_logger().info(
                f'[noclimb_ca] goal from the flight driver ({self.goal[0]:.1f}, {self.goal[1]:.1f}), path at z '
                f'{self.alt:.2f}, ground at z {self.ground_z:.2f} (rest height {z0:.2f})')
        self.goal_rx = True

    def ready(self):
        if self.shadow or self.disarmed_after_flight or self.pos is None or self.t_cloud is None:
            return
        now = self.now()
        if now - self.t_cloud > 1.0 or (self.lidar_topic and (self.t_raw is None or now - self.t_raw > 0.5)):
            return
        m = Bool()
        m.data = True
        self.ready_pub.publish(m)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- outputs ----
    def publish_path(self, pts_enu):
        out = LocalWaypointSetpoint()
        out.path_planning_complete = True
        out.waypoint_x = [float(q[1]) for q in pts_enu]     # North = ENU y
        out.waypoint_y = [float(q[0]) for q in pts_enu]     # East  = ENU x
        out.waypoint_z = [float(q[2]) for q in pts_enu]     # altitude (+)
        self.wp_pub.publish(out)
        vis = Path()
        vis.header.frame_id = 'world'
        vis.header.stamp = self.get_clock().now().to_msg()
        for q in pts_enu:
            ps = PoseStamped()
            ps.header = vis.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(q[0]), float(q[1]), float(q[2])
            ps.pose.orientation.w = 1.0
            vis.poses.append(ps)
        self.path_vis.publish(vis)

    def shadow_reset(self, m):
        if not m.data:
            return
        self.mode, self.final_path, self.t_blind = 'WAIT', None, None
        self.land_requested, self.hold_reason, self.t_contact = False, None, None
        self.get_logger().info('[noclimb_ca] shadow: reset - engaging again, goal ahead of the nose')

    def publish_speed(self, v):
        self.last_speed = float(v)
        m = Float32()
        m.data = float(v)
        self.speed_pub.publish(m)

    def heartbeat(self):
        if self.mode == 'WAIT' or self.disarmed_after_flight:
            return
        m = Bool()
        m.data = True
        self.pp_hb.publish(m)
        self.ca_hb.publish(m)

    def goal_cap(self, d_goal):
        if self.goal_brake <= 0.0:
            return float('inf')
        if self.handover > 0.0:
            d = max(d_goal - self.handover, 0.0)
            return max(math.sqrt(self.handover_speed ** 2 + 2.0 * self.goal_brake * d), self.arrive_speed)
        return max(math.sqrt(2.0 * self.goal_brake * d_goal), self.arrive_speed)

    # ---- 20 Hz step ----
    def tick(self):
        # tick_ms (CSV): this step's compute time - on the onboard computer (PILS / vehicle) it shows
        # whether the 20 Hz loop keeps up; the t column gives the step spacing
        self._tick_t0 = time.perf_counter()
        if self.disarmed_after_flight or self.pos is None:
            return
        t = self.now()
        d_goal = float(np.linalg.norm(self.goal - self.pos[:2]))
        if self.mode == 'WAIT':
            if self.shadow:
                if self.cloud is None:
                    return
                if self.shadow_ahead > 0.0:
                    self.goal = self.pos[:2] + self.shadow_ahead * np.array([math.cos(self.yaw), math.sin(self.yaw)])
                    d_goal = self.shadow_ahead
                self.alt = float(self.pos[2])            # carried: the policy holds the current height
                if self.shadow_carry > 0.0:
                    self.ground_z = self.alt - self.shadow_carry
                    self.agl_off = -self.ground_z
            elif self.goal_topic:
                if self.t_armed is None or not self.goal_rx or self.cloud is None:
                    return
            elif (self.t_armed is None or t - self.t_armed < self.engage_after_arm
                    or self.cloud is None or self.pos[2] < self.alt - 0.5):
                return
            start = self.pos[:2].copy()
            self.pol.reset([start.tolist(), self.goal.tolist()], [self.alt + self.agl_off] * 2)
            self.wp_z = float(self.pos[2])
            self.t_engage = t
            self.mode = 'CA'
            self.v_s = np.asarray(self.vel[:2], dtype=float).copy()
            self.a_s = np.zeros(2)
            self.get_logger().info(
                f'[noclimb_ca] engaged at ({start[0]:.1f}, {start[1]:.1f}, {self.pos[2]:.1f}), '
                f'path {np.linalg.norm(self.goal - start):.1f} m to the goal')
        if self.mode in ('CA', 'PFDIRECT') and d_goal < self.final_switch:
            self.mode = 'FINAL'
            self.final_steps = 0
            self.final_path = [np.r_[self.pos[:2], self.alt], np.r_[self.goal, self.alt]]
            self.get_logger().info(f'[noclimb_ca] final approach: {d_goal:.2f} m to the goal, '
                                   f'fixed path to it (policy stopped)')
        if self.mode == 'FINAL':
            # never faster than the last command: sqrt(2 a d) down to arrive_speed
            v = self.arrive_speed if d_goal < self.arrive_radius else min(self.last_speed, self.goal_cap(d_goal))
            self.publish_speed(v)
            if self.final_steps % 20 == 0:      # at the switch, then 1 Hz re-publish
                self.publish_path(self.final_path)
            self.final_steps += 1
            self.log_row(t, None, d_goal)
            return

        blind = self.blind_reason(t)
        if blind or self.land_requested:
            self.hold_blind(t, blind or self.hold_reason, d_goal)
            return
        if self.mode == 'BLIND':
            self.get_logger().info(f'[noclimb_ca] lidar back after {t - self.t_blind:.1f} s - resuming')
            self.mode, self.t_blind = 'CA', None

        rel = (self.cloud - self.pos) if self.cloud is not None and len(self.cloud) else np.zeros((0, 3))
        if len(rel):
            near = np.einsum('ij,ij->i', rel, rel) <= (self.d_max + 0.5) ** 2
            rel, z_world = rel[near], self.cloud[near, 2]
            if np.isfinite(self.ground_z):
                above = z_world >= self.ground_z + self.ground_clear
                rel, z_world = rel[above], z_world[above]
        else:
            z_world = np.zeros(0)
        n_obj = count_obstacles(rel, z_world)
        density = min(n_obj / 4.0, 1.0) if self.density_const is None else self.density_const
        pos_agl = self.pos + np.array([0.0, 0.0, self.agl_off])
        if self.inflate > 0.0:
            def image(ref_bearing, pts=rel):
                img = self.p2i(pts, ref_bearing, self.pol.c)
                occ = img < self.d_max
                img[occ] = np.maximum(img[occ] - self.inflate, 0.05)
                return img
            out = self.pol.step(pos_agl, self.vel, self.R, image=image, density=density)
        else:
            out = self.pol.step(pos_agl, self.vel, self.R, points=rel, density=density)
        out['wp'] = np.asarray(out['wp'], dtype=float) - np.array([0.0, 0.0, self.agl_off])   # back to this frame
        wp = np.array(out['wp'], dtype=float)           # a copy: out['wp'] is logged as the policy's
        if self.pf_direct and self.pfd_step(t, out, rel, n_obj, density, d_goal):
            return
        if self.wp_z_rate > 0.0:
            step = self.wp_z_rate * self.pol.dt
            self.wp_z += float(np.clip(wp[2] - self.wp_z, -step, step))
            wp[2] = self.wp_z
        out['d_front'] = out['d_img_min']
        out['speed_front'] = float(out['speed'])
        if self.gov_corridor > 0.0:
            out['speed_front'], out['d_front'] = self.corridor_governor(out, rel)
        elif self.gov_sector < 180.0:
            out['speed_front'], out['d_front'] = self.front_governor(out)
        speed = min(out['speed_front'], self.goal_cap(d_goal))
        if self.cmd_tau > 0.0:
            d = wp[:2] - self.pos[:2]
            n = float(np.linalg.norm(d))
            v_cmd = speed * d / n if n > 1e-6 else np.zeros(2)
            dt = self.pol.dt
            frac = 1.0                                   # 1 clear of obstacles .. 0 inside cmd_fade_near_m
            if self.cmd_fade_far > self.cmd_fade_near > 0.0:
                frac = float(np.clip((float(out['d_img_min']) - self.cmd_fade_near)
                                     / (self.cmd_fade_far - self.cmd_fade_near), 0.0, 1.0))
            tau = self.cmd_tau if self.cmd_tau_near < 0.0 else \
                self.cmd_tau_near + frac * (self.cmd_tau - self.cmd_tau_near)
            v_lp = self.v_s + dt / (tau + dt) * (v_cmd - self.v_s)
            if (self.cmd_accel_max > 0.0 or self.cmd_jerk_max > 0.0) and frac > 1e-3:
                a = (v_lp - self.v_s) / dt
                m = float(np.linalg.norm(a))
                a_cap, j_cap = self.cmd_accel_max / frac, self.cmd_jerk_max / frac
                if self.cmd_accel_max > 0.0 and m > a_cap:
                    a = a * (a_cap / m)
                if self.cmd_jerk_max > 0.0:
                    da = a - self.a_s
                    m = float(np.linalg.norm(da))
                    if m > j_cap * dt:
                        a = self.a_s + da * (j_cap * dt / m)
                self.a_s = a
                self.v_s = self.v_s + a * dt
            elif self.cmd_accel_max > 0.0 or self.cmd_jerk_max > 0.0:
                self.a_s = (v_lp - self.v_s) / dt         # caps off near obstacles: track the lag
                self.v_s = v_lp
            else:
                self.v_s = v_lp
            if self.brake_now and float(np.linalg.norm(self.v_s)) > speed:
                self.v_s = self.v_s * (speed / float(np.linalg.norm(self.v_s)))
            sp = float(np.linalg.norm(self.v_s))
            u = self.v_s / sp if sp > 0.05 else (d / n if n > 1e-6 else np.array([1.0, 0.0]))
            wp = np.r_[self.pos[:2] + n * u, wp[2]]
            speed = sp
        if self.safe_r > 0.0:
            d = wp[:2] - self.pos[:2]
            n = float(np.linalg.norm(d))
            v = speed * d / n if n > 1e-6 else np.zeros(2)
            vs = self.safety_filter(v)
            sp = float(np.linalg.norm(vs))
            self.safe_cut = float(np.linalg.norm(v) - sp)
            if sp > 0.05:
                wp = np.r_[self.pos[:2] + max(n, 1.0) * vs / sp, wp[2]]
            speed = sp
            if self.contact_dist > 0.0:
                if self.safe_dmin < self.contact_dist:
                    if self.t_contact is None:
                        self.t_contact = t
                    elif t - self.t_contact >= self.contact_time and not self.land_requested:
                        self.land_requested = True
                        self.hold_reason = (f'contact: a return {self.safe_dmin:.2f} m from the centre '
                                            f'(< {self.contact_dist:.2f}) for {t - self.t_contact:.1f} s')
                        self.get_logger().error(f'[noclimb_ca] {self.hold_reason} - landing in place '
                                                f'(path_following_complete)')
                else:
                    self.t_contact = None
        p0 = np.array([self.pos[0], self.pos[1], wp[2]])
        ext = wp + self.ext_factor * np.r_[wp[:2] - p0[:2], 0.0]
        # a 4th point: PF falls back from MPPI to pursuit while heading for the last waypoint,
        # and its 4.5 m switch distance makes the 3 m WP 'passed' at once
        ext2 = wp + 2.0 * self.ext_factor * np.r_[wp[:2] - p0[:2], 0.0]
        self.publish_speed(speed)                       # speed first: PF's default is 1.5
        self.publish_path([p0, wp, ext, ext2])
        self.last_wp_cmd = wp
        out['n_obj'] = n_obj
        out['density'] = density
        out['n_pts'] = len(rel)
        self.recent.append(out)
        if self.dump_path:
            self.dump_steps.append((t, self.pos.copy(), self.vel.copy(), self.R.copy(), out['obs'].copy(),
                                    np.asarray(out['action'], dtype=np.float32).copy(),
                                    out['ref_bearing'], density, out['speed']))
        self.log_row(t, out, d_goal)

    def front_governor(self, out):
        """The reference governor's law (ca_policy_ref.step) on the policy's own speed output, over
        the image cells within gov_sector of the commanded direction. Image azimuth bin j is
        centred on ref_bearing + j * 360 / n_az (counter-clockwise), the WP on ref_bearing +
        a0 * wp_azim_max_deg. Returns (speed, closest distance in the sector)."""
        c = self.pol.c
        a = out['action']
        speed = min(max(0.5 * (float(a[2]) + 1.0), 0.0), 1.0) * c['v_max']
        if self.gov_sector <= 0.0:                      # governor off: the policy's own speed
            return max(speed, c['speed_floor']), float(c['d_sensor_max'])
        img = np.asarray(out['image'])
        n_az = img.shape[1]
        rel = (np.arange(n_az) * 360.0 / n_az - float(a[0]) * c['wp_azim_max_deg'] + 180.0) % 360.0 - 180.0
        front = np.abs(rel) <= self.gov_sector
        d = float(img[:, front].min()) if front.any() else float(c['d_sensor_max'])
        if c['gov_k'] > 0:
            speed = min(speed, min(max(c['gov_k'] * (d - c['gov_d0']) + c['gov_vmin'], c['gov_vmin']), c['v_max']))
        if c['speed_floor'] > 0:
            speed = max(speed, c['speed_floor'])
        return speed, d

    def pf_direct_clearance(self, rel):
        """distance along the straight line from the vehicle to the goal to the first return within
        pf_direct_corridor_m of it at the vehicle's height (+-pf_direct_dz_m); returns beside or just behind the
        vehicle count as 0, past the goal as nothing; inf: nothing in it"""
        seg = np.asarray(self.goal, dtype=float) - np.asarray(self.pos[:2], dtype=float)
        length = float(np.linalg.norm(seg))
        p = np.asarray(rel, dtype=float).reshape(-1, 3)
        p = p[np.abs(p[:, 2]) < self.pfd_dz, :2]
        if length < 1e-6 or not len(p):
            return float('inf')
        e = seg / length
        along = p @ e
        lat = np.abs(p[:, 0] * e[1] - p[:, 1] * e[0])
        sel = (along >= -self.pfd_w) & (along <= length) & (lat < self.pfd_w)
        return max(float(along[sel].min()), 0.0) if sel.any() else float('inf')

    def pfd_step(self, t, out, rel, n_obj, density, d_goal):
        """the pf_direct gate; True when this step left the flying to PathFollowing on the line to the goal"""
        d_corr = self.pf_direct_clearance(rel)
        out['d_front'], out['speed_front'] = min(d_corr, 99.0), float(out['speed'])
        if self.mode == 'CA':
            if d_corr >= self.pfd_on:
                if self.t_pfd_clear is None:
                    self.t_pfd_clear = t
                v = np.asarray(self.vel[:2], dtype=float)
                to_goal = np.asarray(self.goal, dtype=float) - self.pos[:2]
                miss = abs(math.remainder(math.atan2(v[1], v[0]) - math.atan2(to_goal[1], to_goal[0]), 2 * math.pi))
                if t - self.t_pfd_clear >= self.pfd_hold and np.linalg.norm(v) > 1.0 and miss < self.pfd_align:
                    self.mode = 'PFDIRECT'
                    # at the height it flies now: the policy cruises ~0.2 m off self.alt, and PF climbed that
                    # step at every switch (10-22)
                    z = float(self.pos[2])
                    self.pfd_path = [np.r_[self.pos[:2], z], np.r_[self.goal, z]]
                    self.pfd_steps = 0
                    tilt = math.degrees(math.acos(float(np.clip(self.R[2, 2], -1.0, 1.0))))
                    self.get_logger().info(
                        f'[noclimb_ca] the line to the goal ({d_goal:.1f} m) is clear '
                        f'{"all the way" if math.isinf(d_corr) else f"for {d_corr:.0f} m"} - PathFollowing flies it '
                        f'(policy output unused); at the switch: velocity '
                        f'{math.degrees(miss):.1f} deg off it, {np.linalg.norm(v):.2f} m/s, {out["cross"]:+.2f} m off '
                        f'the global line, tilt {tilt:.1f} deg')
            else:
                self.t_pfd_clear = None
        elif self.mode == 'PFDIRECT' and d_corr < self.pfd_off:
            self.mode, self.t_pfd_clear = 'CA', None
            self.v_s = np.asarray(self.vel[:2], dtype=float).copy()
            self.a_s = np.zeros(2)
            self.get_logger().info(f'[noclimb_ca] a return {d_corr:.1f} m ahead on the line - back to the policy')
        if self.mode != 'PFDIRECT':
            return False
        self.v_s = np.asarray(self.vel[:2], dtype=float).copy()   # back in CA the command starts from here
        self.a_s = np.zeros(2)
        self.publish_speed(min(float(self.pol.c['v_max']), self.goal_cap(d_goal)))
        if self.pfd_steps % 20 == 0:                             # at the switch, then 1 Hz (driver times out at 2.5 s)
            self.publish_path(self.pfd_path)
        self.pfd_steps += 1
        self.last_wp_cmd = self.pfd_path[1]
        out['n_obj'], out['density'], out['n_pts'] = n_obj, density, len(rel)
        self.recent.append(out)
        if self.dump_path:
            self.dump_steps.append((t, self.pos.copy(), self.vel.copy(), self.R.copy(), out['obs'].copy(),
                                    np.asarray(out['action'], dtype=np.float32).copy(),
                                    out['ref_bearing'], density, out['speed']))
        self.log_row(t, out, d_goal)
        return True

    def corridor_governor(self, out, rel):
        """Speed cap from the returns in the path: within gov_corridor of the commanded line
        (ref_bearing + a0 * wp_azim_max_deg) or of the current velocity line, ahead, at the
        vehicle's height (safety_dz). brake_decel > 0: braking-distance law on the distance to where
        the line enters the corridor-radius circle of a return (returns (speed, that distance));
        else the reference governor's law on the along-track distance less range_inflate_m
        (returns (speed, along-track distance)). d_sensor_max: nothing in the path."""
        c = self.pol.c
        a = out['action']
        speed = min(max(0.5 * (float(a[2]) + 1.0), 0.0), 1.0) * c['v_max']
        d = s_free = float(c['d_sensor_max'])
        w = self.gov_corridor
        p = np.asarray(rel, dtype=float).reshape(-1, 3)
        p = p[(p[:, 2] > -self.safe_dz[0]) & (p[:, 2] < self.safe_dz[1]), :2]
        if len(p):
            az = float(out['ref_bearing']) + float(a[0]) * math.radians(c['wp_azim_max_deg'])
            dirs = [np.array([math.cos(az), math.sin(az)])]
            vh = np.asarray(self.vel[:2], dtype=float)
            if float(np.linalg.norm(vh)) > 0.5:
                dirs.append(vh / float(np.linalg.norm(vh)))
            for u in dirs:
                along = p @ u
                lat = np.abs(p[:, 1] * u[0] - p[:, 0] * u[1])
                sel = (along > 0.0) & (lat < w)
                if sel.any():
                    d = min(d, float(along[sel].min()))
                    s_free = min(s_free, float((along[sel] - np.sqrt(w * w - lat[sel] ** 2)).min()))
        if self.brake_decel > 0.0:
            acc, lag = self.brake_decel, self.brake_delay
            cap = acc * (-lag + math.sqrt(lag * lag + 2.0 * max(s_free, 0.0) / acc))
            return min(speed, cap), s_free
        if c['gov_k'] > 0:
            speed = min(speed, min(max(c['gov_k'] * (d - self.inflate - c['gov_d0']) + c['gov_vmin'], c['gov_vmin']),
                                   c['v_max']))
        if c['speed_floor'] > 0:
            speed = max(speed, c['speed_floor'])
        return speed, d

    def blind_reason(self, t):
        """Why the lidar counts as lost (None: it does not). Silence counts from engagement
        when nothing came since."""
        if self.scan_timeout > 0.0 and self.lidar_topic:
            silent = t - (self.t_raw if self.t_raw is not None else self.t_engage)
            if silent > self.scan_timeout:
                return f'no new lidar scan for {silent:.1f} s'
        if self.cloud_timeout > 0.0:
            silent = t - (self.t_cloud if self.t_cloud is not None else self.t_engage)
            if silent > self.cloud_timeout:
                return f'no /cloud for {silent:.1f} s'
        return None

    def hold_blind(self, t, reason, d_goal):
        """Lidar lost: hold (speed 0 on the last path, the policy not stepped - its stall timer
        stays put); lost for blind_land_s: request landing in place, for good."""
        if self.mode != 'BLIND':
            self.mode, self.t_blind = 'BLIND', t
            self.get_logger().warn(f'[noclimb_ca] {reason} - holding (speed 0)')
        if not self.land_requested and self.blind_land > 0.0 and t - self.t_blind > self.blind_land:
            self.land_requested = True
            self.get_logger().error(f'[noclimb_ca] lidar lost for {t - self.t_blind:.1f} s ({reason}) - '
                                    f'landing in place (path_following_complete)')
        if self.land_requested:
            self.done_pub.publish(Bool(data=True))
        self.publish_speed(0.0)
        self.log_row(t, None, d_goal)

    def safety_filter(self, v):
        """Closest point to v of {u : u.n_i <= k (d_i - r)} over one constraint per 10 deg
        sector (its closest return), by repeated projection onto the most violated one."""
        self.safe_dmin = float('inf')
        if not self.safe_scans:
            return v
        rel = np.concatenate([x for _, x in self.safe_scans]) - self.pos
        rel = rel[(rel[:, 2] > -self.safe_dz[0]) & (rel[:, 2] < self.safe_dz[1]), :2]
        d = np.hypot(rel[:, 0], rel[:, 1])
        m = d < self.safe_r + 3.0
        rel, d = rel[m], d[m]
        if len(d) == 0:
            return v
        self.safe_dmin = float(d.min())
        sector = ((np.arctan2(rel[:, 1], rel[:, 0]) + math.pi) / (2.0 * math.pi) * 36.0).astype(int) % 36
        order = np.lexsort((d, sector))
        first = order[np.r_[True, sector[order][1:] != sector[order][:-1]]]
        nrm = rel[first] / np.maximum(d[first], 1e-3)[:, None]
        lim = self.safe_k * (d[first] - self.safe_r)
        u = np.asarray(v, dtype=float).copy()
        for _ in range(30):
            viol = nrm @ u - lim
            i = int(np.argmax(viol))
            if viol[i] <= 1e-3:
                break
            u = u - viol[i] * nrm[i]
        return u

    def log_row(self, t, out, d_goal):
        if self.csv is None:
            return
        base = [f'{t:.3f}', self.mode] + [f'{v:.3f}' for v in (*self.pos, *self.vel)] + [f'{self.yaw:.4f}']
        if out is None:
            rest = [''] * 15 + [f'{self.last_speed:.3f}', '']
        else:
            a = out['action']
            rest = [f'{out["ref_bearing"]:.4f}', f'{out["s"]:.2f}', f'{out["cross"]:.3f}',
                    f'{out["remaining"]:.2f}', f'{out["vdev"]:.3f}', f'{out["d_img_min"]:.2f}',
                    f'{out["density"]:.2f}', out['n_obj'], f'{a[0]:.4f}', f'{a[1]:.4f}', f'{a[2]:.4f}',
                    f'{out["wp"][0]:.3f}', f'{out["wp"][1]:.3f}', f'{out["wp"][2]:.3f}',
                    f'{out["speed"]:.3f}', f'{self.last_speed:.3f}', f'{self.pol.stall_t:.2f}']
        tail = [out['n_pts'] if out is not None else '', f'{d_goal:.2f}']
        wc = getattr(self, 'last_wp_cmd', None)
        tail += [f'{wc[0]:.3f}', f'{wc[1]:.3f}'] if (out is not None and wc is not None) else ['', '']
        tail += [f'{min(self.safe_dmin, 99.0):.2f}', f'{self.safe_cut:.2f}'] if out is not None else ['', '']
        tail += [f'{wc[2]:.3f}'] if (out is not None and wc is not None) else ['']
        tail += [f'{out["d_front"]:.2f}', f'{out["speed_front"]:.3f}'] if (out is not None and 'd_front' in out) else ['', '']
        tail += [f'{(time.perf_counter() - self._tick_t0) * 1e3:.2f}']
        self.csv.writerow(base + rest + tail)

    def report(self):
        if self.mode == 'WAIT':
            return
        if self.mode == 'FINAL' or not self.recent:
            self.get_logger().info(f'[noclimb_ca] {self.mode} pos ({self.pos[0]:.1f}, {self.pos[1]:.1f}, '
                                   f'{self.pos[2]:.1f}) speed cmd {self.last_speed:.2f} '
                                   f'|v| {np.linalg.norm(self.vel[:2]):.2f}')
            return
        o = self.recent[-1]
        a = np.array([r['action'] for r in self.recent])
        self.get_logger().info(
            f'[noclimb_ca] {self.mode} pos ({self.pos[0]:.1f}, {self.pos[1]:.1f}, {self.pos[2]:.1f}) '
            f'rem {o["remaining"]:.1f} xte {o["cross"]:+.2f} dmin {min(r["d_img_min"] for r in self.recent):.2f} '
            f'dens {o["density"]:.2f} a0 [{a[:, 0].min():+.2f},{a[:, 0].max():+.2f}] '
            f'v cmd {self.last_speed:.2f} |v| {np.linalg.norm(self.vel[:2]):.2f} '
            f'stall {self.pol.stall_t:.1f}s scans {self.cloud_n}')

    def write_dump(self):
        if not self.dump_path or not self.dump_steps:
            return
        st = self.dump_steps
        np.savez_compressed(
            self.dump_path,
            scan_t=np.array([t for t, _ in self.dump_scans]),
            scan_n=np.array([len(x) for _, x in self.dump_scans]),
            scan_xyz=(np.concatenate([x for _, x in self.dump_scans]) if self.dump_scans
                      else np.zeros((0, 3), np.float32)),
            t=np.array([r[0] for r in st]), pos=np.array([r[1] for r in st]),
            vel=np.array([r[2] for r in st]), R=np.array([r[3] for r in st]),
            obs=np.array([r[4] for r in st]), action=np.array([r[5] for r in st]),
            ref_bearing=np.array([r[6] for r in st]), density=np.array([r[7] for r in st]),
            speed=np.array([r[8] for r in st]), memory_s=self.memory_s)
        self.get_logger().info(f'[noclimb_ca] replay dump: {self.dump_path} ({len(st)} steps, '
                               f'{len(self.dump_scans)} scans)')
        self.dump_path = ''


def main():
    rclpy.init(args=sys.argv)
    node = NoClimbCA()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_dump()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
