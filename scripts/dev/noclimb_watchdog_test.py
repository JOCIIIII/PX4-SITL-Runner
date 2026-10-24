#!/usr/bin/env python3
"""Drive scripts/ros2/apps/noclimb_ca_node.py's lidar watchdog and front-sector speed governor
through its callbacks (no simulator, fake clock, the ca_big policy): the driver topic arrives at
~100 Hz with a new header stamp every 0.1 s (the AirSim bridge republishes the latest scan),
/cloud at 10 Hz.
  1. normal flight engages and commands a speed
  2. driver silent 1 s: hold (speed 0) within scan_timeout_s 0.5 + one step, back to CA after
  3. driver fine but /cloud silent 2.5 s: hold only after cloud_timeout_s 2.0
  4. /cloud gaps of 1.3 s (the adapter's tilt gate while braking) with the driver fine: no hold;
     empty clouds (publish_empty_scans: a 10 m lidar sees nothing in the open) keep it alive and
     clear the image
  5. everything silent 7 s: hold, then path_following_complete after blind_land_s 5; the lidar
     coming back does not resume (landing requested for good)
  6. governor_sector_deg 90: the governor's law over the front half of the image only - a cell
     behind or beside the commanded direction does not cap the speed, one ahead does; 180 is
     the reference governor; in the node, an obstacle 1.6 m behind no longer caps it at 0.6 m/s
  7. governor_corridor_m 1.2: only returns within 1.2 m of the commanded or the velocity line,
     ahead and at the vehicle's height, cap the speed (by their along-track distance - 0.8 m);
     governor_brake_decel 2.5: by braking distance to where the line enters the 1.2 m circle
  8. cmd_tau_s 0.7 with cmd_tau_near_s 0.3 over cmd_fade_near_m 4 / _far_m 6: a 90 deg turn of the
     policy's WP is followed with the 0.3 s lag inside 4 m of a return, 0.5 s at 5 m, 0.7 s beyond
     6 m, and 0.7 s everywhere with cmd_tau_near_s -1
  9-11. shadow mode, reference from the start, voxel filter
  12. goal_topic (flight_driver): no engagement before the driver's goal, heights referenced from it, later
     goals ignored once engaged; /noclimb/ready only with a pose and a fresh /cloud
Exit status 1 on a failed check. Run it inside the ROS image:
  docker run --rm -v $HOME/Documents/A4VAI-SITL/ROS2:/home/user/workspace/ros2 \
      -v $HOME/PX4-SITL-Runner:$HOME/PX4-SITL-Runner:ro --entrypoint bash jociiiii/a4vai:inhousesim-v0.9 -c \
      "source /opt/ros/humble/setup.bash && source /home/user/workspace/ros2/ros2_ws/install/setup.bash && \
      python3 $HOME/PX4-SITL-Runner/scripts/dev/noclimb_watchdog_test.py"
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ros2', 'apps'))
import rclpy  # noqa: E402
import sensor_msgs_py.point_cloud2 as pc2  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from px4_msgs.msg import VehicleStatus  # noqa: E402
from rclpy.serialization import serialize_message  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

import noclimb_ca_node as nc  # noqa: E402

POLICY_DIR = '/home/user/workspace/ros2/ros2_ws/src/Inhouse-collision-avoidance/ca_big_onnx'


def cloud(t, empty=False):
    h = Header()
    h.stamp.sec, h.stamp.nanosec = int(t), int((t % 1) * 1e9)
    pts = np.array([[6.0, 2.0, z] for z in np.linspace(3.0, 5.0, 8)], dtype=np.float32)   # a pole beside the path
    return pc2.create_cloud_xyz32(h, pts[:0] if empty else pts)


def main():
    rclpy.init(args=['--ros-args', '-p', f'policy_dir:={POLICY_DIR}', '-p', 'policy:=ca_big',
                     '-p', 'lidar_topic:=/lidar_raw'])
    n = nc.NoClimbCA()
    clock = [100.0]
    n.now = lambda: clock[0]
    speeds, done = [], []
    n.speed_pub.publish = lambda m: speeds.append((clock[0], m.data))
    n.done_pub.publish = lambda m: done.append((clock[0], m.data))
    n.wp_pub.publish = lambda m: None

    o = Odometry()
    o.pose.pose.position.z = 4.0
    o.pose.pose.orientation.w = 1.0
    st = VehicleStatus()
    st.arming_state = VehicleStatus.ARMING_STATE_ARMED
    n.odom_cb(o)
    n.status_cb(st)

    raw_last = [None]
    modes = []

    def run(t_end, driver=True, cloud_on=True, cloud_gap=None, empty=False):
        """20 Hz ticks to t_end; driver: new stamps every 0.1 s (duplicates in between);
        cloud_gap: (period, length) of /cloud silences"""
        while clock[0] < t_end - 1e-9:
            clock[0] = round(clock[0] + 0.05, 3)
            t = clock[0]
            k = int(round(t * 20))
            n.odom_cb(o)
            if driver and k % 2 == 0:
                raw_last[0] = serialize_message(cloud(t))
            if raw_last[0] is not None:
                n.raw_cb(raw_last[0])           # ~100 Hz in reality: new or duplicate stamp
            in_gap = cloud_gap is not None and (t % cloud_gap[0]) < cloud_gap[1]
            if cloud_on and not in_gap and k % 2 == 0:
                n.cloud_cb(cloud(t, empty))
            n.tick()
            modes.append((t, n.mode))

    ok = True

    def check(name, good):
        nonlocal ok
        print(f'  {"ok  " if good else "FAIL"} {name}')
        ok &= bool(good)

    # 1. normal: engages 11 s after arming, commands a speed
    run(112.0)
    check('engaged (CA) and commanding a speed', n.mode == 'CA' and speeds and speeds[-1][1] > 0.5)
    # 2. driver silent 1 s
    t0 = clock[0]
    run(113.0, driver=False, cloud_on=False)
    blind = [t for t, m in modes if m == 'BLIND' and t > t0]
    check(f'driver silent: hold after {blind[0] - t0:.2f} s (0.5 .. 0.6)' if blind else 'driver silent: hold',
          blind and 0.5 < blind[0] - t0 <= 0.6)
    check('speed 0 while holding', all(v == 0.0 for t, v in speeds if blind and t >= blind[0]))
    run(113.5)
    check('driver back: CA again', n.mode == 'CA' and speeds[-1][1] > 0.5)
    # 3. driver fine, /cloud silent 2.5 s
    t0 = clock[0]
    run(116.0, cloud_on=False)
    blind = [t for t, m in modes if m == 'BLIND' and t > t0]
    check(f'/cloud silent: hold after {blind[0] - t0:.2f} s (2.0 .. 2.1)' if blind else '/cloud silent: hold',
          blind and 2.0 < blind[0] - t0 <= 2.1)
    run(116.5)
    check('/cloud back: CA again', n.mode == 'CA')
    # 4. tilt-gate gaps of 1.3 s every 3 s, driver fine: never holds
    t0 = clock[0]
    run(126.5, cloud_gap=(3.0, 1.3))
    check('1.3 s /cloud gaps with the driver fine: no hold', all(m == 'CA' for t, m in modes if t > t0))
    t0 = clock[0]
    run(129.5, empty=True)
    check('3 s of empty clouds: no hold, image cleared',
          all(m == 'CA' for t, m in modes if t > t0) and n.cloud is not None and len(n.cloud) == 0)
    run(130.0)
    check('no landing request so far', not done)
    # 5. everything silent 7 s, then back
    t0 = clock[0]
    run(137.0, driver=False, cloud_on=False)
    blind = [t for t, m in modes if m == 'BLIND' and t > t0]
    check(f'all silent: landing requested {done[0][0] - blind[0]:.2f} s after the hold (5.0 .. 5.1)' if done and blind
          else 'all silent: landing requested', done and blind and 5.0 < done[0][0] - blind[0] <= 5.1)
    run(138.0)
    check('lidar back after the landing request: stays holding, keeps requesting',
          n.mode == 'BLIND' and done[-1][0] > 137.5 and speeds[-1][1] == 0.0)
    n.destroy_node()

    # 6. front-sector governor: the law on synthetic images (inflated distances, as the policy sees)
    g = nc.NoClimbCA()
    c = g.pol.c
    n_el, n_az, dmax = int(c['lidar_n_el']), int(c['lidar_n_az']), float(c['d_sensor_max'])

    def gov(cells, a0=0.0, sector=90.0):
        img = np.full((n_el, n_az), dmax, dtype=np.float32)
        for j, dist in cells:
            img[1, j % n_az] = dist
        g.gov_sector = sector
        return g.front_governor({'action': np.array([a0, 0.0, 1.0]), 'image': img})

    law = lambda d: min(max(c['gov_k'] * (d - c['gov_d0']) + c['gov_vmin'], c['gov_vmin']), c['v_max'])  # noqa: E731
    q = n_az // 4                                  # 90 deg in bins (bin j at +j * 360 / n_az, counter-clockwise)
    check('cell behind (180 deg) at 0.8: no cap with +-90', abs(gov([(2 * q, 0.8)])[0] - c['v_max']) < 1e-6)
    check('cell behind (180 deg) at 0.8: capped to the floor with 180 (reference)',
          abs(gov([(2 * q, 0.8)], sector=180.0)[0] - law(0.8)) < 1e-6)
    check('cell ahead at 2.2: capped by the law', abs(gov([(0, 2.2)])[0] - law(2.2)) < 1e-6)
    check('turning left (a0 = +1): cell at +90 deg is ahead -> capped', abs(gov([(q, 1.0)], a0=1.0)[0] - law(1.0)) < 1e-6)
    check('turning left (a0 = +1): cell at -90 deg is behind -> not capped', abs(gov([(-q, 1.0)], a0=1.0)[0] - c['v_max']) < 1e-6)
    check('sector 0 (governor off): cell ahead at 0.8 does not cap', abs(gov([(0, 0.8)], sector=0.0)[0] - c['v_max']) < 1e-6)

    # 7. corridor governor (X8: inflate 0.8, half-width 1.2 m), vehicle flying +x at 5 m/s
    g.gov_corridor, g.inflate = 1.2, 0.8
    g.vel = np.array([5.0, 0.0, 0.0])

    def cor(pts, a0=0.0):
        pts = np.array([[x, y, z] for x, y in pts for z in (-0.3, 0.0, 0.3)])
        return g.corridor_governor({'action': np.array([a0, 0.0, 1.0]), 'ref_bearing': 0.0}, pts)[0]

    check('corridor: pole beside (1.5 m off the line, 1 m ahead) does not cap', abs(cor([(1.0, -1.5)]) - c['v_max']) < 1e-6)
    check('corridor: pole on the line 2 m ahead caps (law of 2.0 - 0.8)', abs(cor([(2.0, 0.0)]) - law(1.2)) < 1e-6)
    check('corridor: pole 1.0 m off the line, 2.5 m ahead caps (law of 2.5 - 0.8)', abs(cor([(2.5, 1.0)]) - law(1.7)) < 1e-6)
    check('corridor: pole on the line behind does not cap', abs(cor([(-1.0, 0.0)]) - c['v_max']) < 1e-6)
    check('corridor: commanded left but still moving +x - a pole ahead on the velocity line caps',
          abs(cor([(2.0, 0.2)], a0=1.0) - law(1.2)) < 1e-6)
    above = g.corridor_governor({'action': np.array([0.0, 0.0, 1.0]), 'ref_bearing': 0.0}, np.array([[2.0, 0.0, 1.5]]))[0]
    check('corridor: return 1.5 m above the vehicle does not cap', abs(above - c['v_max']) < 1e-6)
    g.brake_decel, g.brake_delay = 2.5, 0.6
    brake = lambda s_: min(c['v_max'], 2.5 * (-0.6 + math.sqrt(0.36 + 2.0 * max(s_, 0.0) / 2.5)))  # noqa: E731
    check(f'brake law: pole on the line 8 m ahead -> {cor([(8.0, 0.0)]):.2f} m/s (stops by 1.2 m from it)',
          abs(cor([(8.0, 0.0)]) - brake(8.0 - 1.2)) < 1e-6)
    check(f'brake law: tight pass (1.1 m off the line) 1 m ahead -> {cor([(1.0, 1.1)]):.2f} m/s',
          abs(cor([(1.0, 1.1)]) - brake(1.0 - math.sqrt(1.2 ** 2 - 1.1 ** 2))) < 1e-6)
    check('brake law: pole 1.5 m off the line does not cap', abs(cor([(2.0, 1.5)]) - c['v_max']) < 1e-6)
    check('brake law: pole inside the 1.2 m circle ahead -> stop', abs(cor([(0.5, 0.3)])) < 1e-6)
    check('brake law: pole on the line behind does not cap', abs(cor([(-1.0, 0.0)]) - c['v_max']) < 1e-6)
    g.gov_corridor, g.brake_decel = 0.0, 0.0

    #    in the node: an obstacle 1.6 m behind (image 0.8 m with the X8 inflation)
    g.inflate = 0.8
    caps = {}
    for sector in (90.0, 180.0):
        g.gov_sector = sector
        g.mode, g.t_armed, g.cloud = 'WAIT', None, None
        clock[0] = 300.0
        g.now = lambda: clock[0]
        got = []
        g.speed_pub.publish = lambda m: got.append(m.data)
        g.wp_pub.publish = lambda m: None
        g.done_pub.publish = lambda m: None
        g.odom_cb(o)
        g.status_cb(st)
        g.t_armed = clock[0] - 12.0
        for k in range(10):
            clock[0] = round(clock[0] + 0.05, 3)
            h = Header()
            h.stamp.sec, h.stamp.nanosec = int(clock[0]), int((clock[0] % 1) * 1e9)
            pts = np.array([[-1.6, 0.0, z] for z in np.linspace(3.5, 4.5, 6)], dtype=np.float32)
            g.raw_cb(serialize_message(pc2.create_cloud_xyz32(h, pts)))
            g.cloud_cb(pc2.create_cloud_xyz32(h, pts))
            g.odom_cb(o)
            g.tick()
        caps[sector] = got[-1] if got else None
    check(f'node, obstacle 1.6 m behind: 180 caps at {caps[180.0]:.2f} m/s, +-90 lets {caps[90.0]:.2f} m/s',
          caps[180.0] is not None and caps[90.0] is not None and caps[180.0] <= law(0.8) + 0.05 and caps[90.0] > caps[180.0] + 1.0)

    # 8. command smoothing faded by obstacle distance (cmd_tau_s 0.7 beyond 6 m, cmd_tau_near_s 0.3
    #    inside 4 m): a synthetic policy flies +x at 2 m/s, then turns its WP 90 deg left; the time
    #    for the smoothed command to come within 10 deg of it ((1 - a)^k <= 0.15, a = dt / (tau + dt))
    g.gov_sector, g.inflate, g.safe_r, g.brake_now = 180.0, 0.0, 0.0, False
    g.cmd_tau, g.cmd_fade_near, g.cmd_fade_far = 0.7, 4.0, 6.0
    dt = g.pol.dt
    expect = lambda tau: math.ceil(math.log(0.15) / math.log(1.0 - dt / (tau + dt))) * dt  # noqa: E731

    def turn_time(d_img, tau_near):
        g.cmd_tau_near = tau_near
        head, t_turn = [0.0], [None]

        def fake_step(pos, vel, R, **kw):
            hd = head[0]
            return {'wp': np.asarray(pos, dtype=float) + 3.0 * np.array([math.cos(hd), math.sin(hd), 0.0]),
                    'speed': 2.0, 'd_img_min': d_img, 'action': np.zeros(3), 'ref_bearing': 0.0, 's': 0.0,
                    'cross': 0.0, 'remaining': 50.0, 'vdev': 0.0, 'obs': np.zeros(1)}
        g.pol.step = fake_step
        g.mode, g.cloud = 'WAIT', None
        g.t_armed = clock[0] - 12.0
        g.goal = np.array([500.0, 0.0])
        for k in range(160):
            clock[0] = round(clock[0] + dt, 3)
            if k == 80:
                head[0], t_turn[0] = math.pi / 2.0, clock[0]
            if k % 2 == 0:
                h = Header()
                h.stamp.sec, h.stamp.nanosec = int(clock[0]), int((clock[0] % 1) * 1e9)
                pts = np.array([[30.0, 30.0, z] for z in np.linspace(3.5, 4.5, 6)], dtype=np.float32)
                g.raw_cb(serialize_message(pc2.create_cloud_xyz32(h, pts)))
                g.cloud_cb(pc2.create_cloud_xyz32(h, pts))
            g.odom_cb(o)
            g.tick()
            if t_turn[0] is not None and abs(math.degrees(math.atan2(g.v_s[1], g.v_s[0])) - 90.0) <= 10.0:
                return clock[0] - t_turn[0] + dt          # the turn's WP is first used on its own tick
        return float('inf')

    for d_img, tau_near, tau in ((3.0, 0.3, 0.3), (5.0, 0.3, 0.5), (10.0, 0.3, 0.7), (3.0, -1.0, 0.7)):
        lag = turn_time(d_img, tau_near)
        check(f'smoothing: nearest return {d_img:.0f} m, cmd_tau_near_s {tau_near:+.1f} -> 90 deg turn within '
              f'10 deg after {lag:.2f} s (tau {tau}: {expect(tau):.2f} s)', abs(lag - expect(tau)) <= dt + 1e-6)
    g.destroy_node()
    rclpy.try_shutdown()

    # 9. shadow (carried, not flown): engages unarmed with the goal ahead of the nose at the current height,
    #    every output on /shadow/..., the path on /noclimb/path; /shadow/reset starts a new run
    print('9. shadow mode')
    rclpy.init(args=['--ros-args', '-p', f'policy_dir:={POLICY_DIR}', '-p', 'policy:=ca_big',
                     '-p', 'lidar_topic:=/lidar_raw', '-p', 'shadow:=true', '-p', 'shadow_goal_ahead_m:=30.0'])
    s = nc.NoClimbCA()
    clk = [200.0]
    s.now = lambda: clk[0]
    vis, wps = [], []
    s.path_vis.publish = vis.append
    s.wp_pub.publish = wps.append
    s.speed_pub.publish = lambda m: None
    names = {p.topic_name for p in (s.wp_pub, s.speed_pub, s.pp_hb, s.ca_hb, s.done_pub)}
    check(f'shadow: every command topic under /shadow ({len(names)} topics)', all(nm.startswith('/shadow/') for nm in names))

    def carry(x, y, z, yaw_deg, secs):
        so = Odometry()
        so.pose.pose.position.x, so.pose.pose.position.y, so.pose.pose.position.z = x, y, z
        so.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
        so.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
        t_end = clk[0] + secs
        while clk[0] < t_end - 1e-9:
            clk[0] = round(clk[0] + 0.05, 3)
            s.odom_cb(so)
            if int(round(clk[0] * 20)) % 2 == 0:
                s.raw_cb(serialize_message(cloud(clk[0])))
                s.cloud_cb(cloud(clk[0]))
            s.tick()

    carry(2.0, 3.0, 1.2, 90.0, 1.0)                  # held 1.2 m up, nose north, never armed
    sd = VehicleStatus()
    sd.arming_state = VehicleStatus.ARMING_STATE_DISARMED
    s.status_cb(sd)                                  # PX4 keeps reporting disarmed: not the end of a flight
    n_vis = len(vis)
    carry(2.0, 3.0, 1.2, 90.0, 0.5)
    check(f'shadow: a disarmed status does not stop it ({len(vis) - n_vis} paths in 0.5 s after it)',
          not s.disarmed_after_flight and len(vis) - n_vis >= 8)
    check(f'shadow: ground re-taken under the carried vehicle: z {s.ground_z:.2f} (1.20 - 1.2), AGL offset {s.agl_off:.2f}',
          abs(s.ground_z - 0.0) < 1e-6 and abs(s.agl_off - 0.0) < 1e-6)
    check(f'shadow: engaged unarmed (mode {s.mode}), goal ({s.goal[0]:.1f}, {s.goal[1]:.1f}) = 30 m north, '
          f'height {s.alt:.2f}', s.mode == 'CA' and np.allclose(s.goal, [2.0, 33.0], atol=1e-6) and abs(s.alt - 1.2) < 1e-6)
    check(f'shadow: path shown on /noclimb/path ({len(vis)} msgs, frame {vis[-1].header.frame_id if vis else "-"})',
          len(vis) > 0 and vis[-1].header.frame_id == 'world' and len(vis[-1].poses) == len(wps[-1].waypoint_x))
    s.shadow_reset(nc.Bool(data=True))
    check('shadow: reset -> WAIT', s.mode == 'WAIT')
    carry(5.0, 5.0, 1.0, 0.0, 0.5)                   # new run, nose east
    check(f'shadow: new run, goal ({s.goal[0]:.1f}, {s.goal[1]:.1f}) = 30 m east of (5, 5)',
          s.mode == 'CA' and np.allclose(s.goal, [35.0, 5.0], atol=1e-6))
    # the carrier 0.8 m behind the nose is masked, a pole 0.8 m ahead is not (nose east at (5, 5))
    s.cloud_cb(pc2.create_cloud_xyz32(Header(), np.array([[4.2, 5.0, 1.0], [5.8, 5.0, 1.0]], dtype=np.float32)))
    kept = s.scans[-1][1]
    check(f'shadow: carrier behind masked, obstacle ahead kept ({len(kept)} of 2 left, x {kept[:, 0].tolist()})',
          len(kept) == 1 and abs(kept[0, 0] - 5.8) < 1e-6)
    s.destroy_node()
    rclpy.try_shutdown()

    # 10. ref_from_start (the vehicle: PX4's local origin is not at the resting vehicle)
    print('10. reference from the start')
    rclpy.init(args=['--ros-args', '-p', f'policy_dir:={POLICY_DIR}', '-p', 'policy:=ca_big', '-p', 'ref_from_start:=true',
                     '-p', 'ground_z_m:=-0.23', '-p', 'agl_offset_m:=0.23', '-p', 'altitude:=1.47',
                     '-p', 'goal_x:=-79.0', '-p', 'goal_y:=0.0'])
    v = nc.NoClimbCA()
    ro = Odometry()
    ro.pose.pose.position.x, ro.pose.pose.position.y, ro.pose.pose.position.z = 10.0, 20.0, 13.2
    ro.pose.pose.orientation.w = 1.0
    v.odom_cb(ro)
    ro.pose.pose.position.z = 15.0                   # later odometry does not move the reference
    v.odom_cb(ro)
    check(f'ref_from_start: ground z {v.ground_z:.2f} (12.97), AGL offset {v.agl_off:.2f} (-12.97), path z {v.alt:.2f} '
          f'(14.67), goal ({v.goal[0]:.1f}, {v.goal[1]:.1f}) (-69, 20)',
          abs(v.ground_z - 12.97) < 1e-6 and abs(v.agl_off + 12.97) < 1e-6 and abs(v.alt - 14.67) < 1e-6
          and np.allclose(v.goal, [-69.0, 20.0]))
    # 11. voxel_m: the nearest return per cube, fewer points, the nearest distance kept
    v.voxel = 0.1
    v.pos = np.array([0.0, 0.0, 0.0])
    rng = np.random.default_rng(3)
    wall = np.column_stack([np.full(20000, 2.0) + rng.uniform(0, 0.02, 20000), rng.uniform(-2, 2, 20000),
                            rng.uniform(-1, 1, 20000)])
    vd = v.voxel_down(wall)
    d0, d1 = np.linalg.norm(wall, axis=1).min(), np.linalg.norm(vd, axis=1).min()
    check(f'voxel 0.1 m: {len(wall)} -> {len(vd)} points, nearest return {d0:.3f} -> {d1:.3f} m',
          len(vd) < len(wall) / 5 and abs(d1 - d0) < 1e-9)
    v.destroy_node()
    rclpy.try_shutdown()

    # 12. goal_topic (flight_driver): no engagement before the driver's goal, heights referenced from it
    #     (not from the first pose), later goals ignored once engaged; /noclimb/ready only with live inputs
    print('12. goal from the flight driver')
    rclpy.init(args=['--ros-args', '-p', f'policy_dir:={POLICY_DIR}', '-p', 'policy:=ca_big', '-p', 'ref_from_start:=true',
                     '-p', 'ground_z_m:=-0.23', '-p', 'agl_offset_m:=0.23', '-p', 'altitude:=3.0',
                     '-p', 'goal_topic:=/flight_driver/goal'])
    g = nc.NoClimbCA()
    g.now = lambda: clock[0]
    g.wp_pub.publish = lambda m: None
    g.speed_pub.publish = lambda m: None
    readies = []
    g.ready_pub.publish = lambda m: readies.append(m.data)
    go = Odometry()
    go.pose.pose.position.x, go.pose.pose.position.y, go.pose.pose.position.z = 10.0, 20.0, 13.2
    go.pose.pose.orientation.w = 1.0
    g.odom_cb(go)
    check(f'goal_topic: the first pose does not set the reference (path z {g.alt:.2f}, ground z {g.ground_z:.2f})',
          abs(g.alt - 3.0) < 1e-9 and abs(g.ground_z + 0.23) < 1e-9)
    g.status_cb(st)
    g.ready()
    check('ready: nothing before a /cloud', not readies)
    g.cloud_cb(cloud(clock[0]))
    g.ready()
    check('ready: pose and a fresh /cloud -> /noclimb/ready', readies == [True])
    for _ in range(40):                                  # 2 s armed, at the path height, no goal yet
        clock[0] += 0.05
        g.cloud_cb(cloud(clock[0]))
        g.tick()
    check(f'goal_topic: armed 2 s without the goal -> still {g.mode}', g.mode == 'WAIT')
    from geometry_msgs.msg import PointStamped  # noqa: E402
    gm = PointStamped()
    gm.point.x, gm.point.y, gm.point.z = -69.0, 20.0, 16.2   # driver: take-off point rested at z 13.2, path 3 m up
    g.goal_cb(gm)
    clock[0] += 0.05
    g.cloud_cb(cloud(clock[0]))
    g.tick()
    check(f'goal_topic: engaged on the goal (mode {g.mode}), goal ({g.goal[0]:.1f}, {g.goal[1]:.1f}) (-69, 20), path z '
          f'{g.alt:.2f} (16.20), ground z {g.ground_z:.2f} (12.97), AGL offset {g.agl_off:.2f} (-12.97)',
          g.mode == 'CA' and np.allclose(g.goal, [-69.0, 20.0]) and abs(g.alt - 16.2) < 1e-9
          and abs(g.ground_z - 12.97) < 1e-9 and abs(g.agl_off + 12.97) < 1e-9)
    gm.point.x, gm.point.z = 50.0, 30.0
    g.goal_cb(gm)
    check('goal_topic: a later goal is ignored once engaged', np.allclose(g.goal, [-69.0, 20.0]) and abs(g.alt - 16.2) < 1e-9)
    readies.clear()
    clock[0] += 2.0
    g.ready()
    check('ready: /cloud 2 s old -> no /noclimb/ready', not readies)
    g.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
