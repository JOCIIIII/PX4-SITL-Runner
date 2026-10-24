#!/usr/bin/env python3
"""Drive scripts/ros2/apps/super_speed_guard.py's ROS node through its callbacks (no simulator):
  1. brake / release: on a straight path a panel crosses ahead at 1 m/s - the speed command must
     go to 0 before it, stay 0 while it crosses, and come back once it has passed;
  2. /cloud_static: once the panel is moving its returns (and the body below the band) leave the
     republished cloud, a pole's stay while within 25 m, and the fields are kept;
  3. the same from a separate gated input (static_cloud_input) with static_cloud_min_z.
Exit status 1 on a failed check. Run it inside the ROS image (see speed_guard_sim.py):
  docker run --rm -v $HOME/Documents/A4VAI-SITL/ROS2:/home/user/workspace/ros2 \
      -v $HOME/PX4-SITL-Runner:$HOME/PX4-SITL-Runner:ro --entrypoint bash jociiiii/a4vai:inhousesim-v0.9 -c \
      "source /opt/ros/humble/setup.bash && source /home/user/workspace/ros2/ros2_ws/install/setup.bash && \
      python3 $HOME/PX4-SITL-Runner/scripts/dev/speed_guard_node_test.py"
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ros2', 'apps'))
import rclpy  # noqa: E402
import sensor_msgs_py.point_cloud2 as pc2  # noqa: E402
from custom_msgs.msg import LocalWaypointSetpoint  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import PointField  # noqa: E402
from std_msgs.msg import Float32, Header  # noqa: E402

import super_speed_guard as sg  # noqa: E402

FIELDS = [PointField(name=c, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, c in enumerate('xyz')] + \
         [PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)]


def cloud(t, panel_x, panel_y, pole=(-8.0, 3.0)):
    """ENU: a 1.2 m panel (z 1.0-2.1) at x = panel_x centred on y = panel_y, its body below the
    band, and a 2 m pole"""
    yy, zz = np.meshgrid(np.linspace(panel_y - 0.6, panel_y + 0.6, 9), np.linspace(1.0, 2.1, 6))
    panel = np.column_stack([np.full(yy.size, panel_x), yy.ravel(), zz.ravel()])
    body = np.column_stack([np.full(6, panel_x), np.linspace(panel_y - 0.2, panel_y + 0.2, 6), np.full(6, -0.3)])
    p = np.column_stack([np.full(10, pole[0]), np.full(10, pole[1]), np.linspace(0, 2, 10)])
    xyz = np.vstack([panel, body, p]).astype(np.float32)
    s = np.zeros(len(xyz), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)])
    s['x'], s['y'], s['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    h = Header()
    h.stamp.sec, h.stamp.nanosec = int(t), int((t % 1) * 1e9)
    return pc2.create_cloud(h, FIELDS, s), len(panel) + len(body), len(p)


def odom(x, v):
    o = Odometry()
    o.pose.pose.position.x, o.pose.pose.position.z = x, 1.5
    o.twist.twist.linear.x = -v
    return o


def main():
    rclpy.init(args=['--ros-args', '-p', 'static_cloud_topic:=/cloud_static'])
    n = sg.make_node()
    cmds, static = [], []
    n.speed_pub.publish = lambda m: cmds.append(m.data)
    n.static_pub.publish = lambda m: static.append(m)
    clock = [100.0]
    n.now = lambda: clock[0]
    wp = LocalWaypointSetpoint()                      # NED: east 0 .. -19.5 m = ENU x decreasing
    wp.waypoint_x, wp.waypoint_y, wp.waypoint_z = [0.0] * 40, [-0.5 * i for i in range(40)], [-1.5] * 40
    n.path_cb(wp)
    ok = True

    # vehicle at 5 m/s toward a panel line 18 m ahead; the panel sets off 1.5 s in, crossing y 4 -> -4
    x, v, py = 0.0, 0.0, 4.6
    log = []
    for k in range(200):                              # 20 s at 10 Hz
        t = 100.0 + 0.1 * k
        clock[0] = t
        n.odom_cb(odom(x, v))
        m, n_cart, n_pole = cloud(t, -18.0, py)
        n.cloud_cb(m)
        out = pc2.read_points(static[-1], skip_nans=False)
        for j in range(2):                            # 20 Hz speed ticks
            clock[0] = t + 0.05 * j
            n.speed_cb(Float32(data=min(5.0, 2.5 * 0.1 * k)))
        cmd = cmds[-1]
        v = max(0.0, v + max(min(2.5 * (cmd - v), 2.2), -3.0) * 0.1)
        x -= v * 0.1
        if k >= 15:
            py -= 0.1
        log.append((k * 0.1, x, v, cmd, py, len(out), n_cart, n_pole, [f.name for f in static[-1].fields]))
    braked = [r for r in log if r[3] == 0.0 and r[0] > 1.5]
    passed = [r for r in log if r[1] < -18.0]
    crossing = [r for r in log if abs(r[4]) < 1.4]   # panel within the vehicle's swept width
    gap = min(abs(r[1] - (-18.0)) for r in crossing) if crossing else None
    checks = [
        ('braked for the moving panel', bool(braked)),
        ('never inside 1.5 m of the panel line while it crossed', gap is not None and gap > 1.5),
        ('went on after it had passed', bool(passed)),
        ('panel returns left /cloud_static once moving', any(r[5] == r[7] for r in log)),
        ('pole returns kept within 24 m', all(r[5] >= r[7] for r in log if np.hypot(r[1] + 8.0, 3.0) < 24.0)),
        ('pole returns dropped beyond 26 m (static_cloud_max_range_m 25)',
         all(r[5] < r[7] for r in log if np.hypot(r[1] + 8.0, 3.0) > 26.0 and abs(r[4]) > 30)),
        ('fields kept', log[-1][8] == ['x', 'y', 'z', 'intensity']),
        ('no guard errors', n.n_err == 0 and n.n_blind == 0),
    ]
    for name, good in checks:
        print(f'  {"ok  " if good else "FAIL"} {name}')
        ok &= good
    print(f'  closest to the panel line while it crossed: {gap:.2f} m' if gap is not None else '  panel never crossed')
    n.destroy_node()

    # 3. /cloud_static from a separate gated input (static_cloud_input), with static_cloud_min_z:
    #    moving returns and returns below the limit leave, the pole above it stays
    n = sg.make_node()
    n.static_input, n.static_min_z = '/cloud_gated', 0.5
    out2 = []
    n.static_pub.publish = lambda m: out2.append(m)
    n.speed_pub.publish = lambda m: None
    n.now = lambda: clock[0]
    n.path_cb(wp)
    py = 4.6
    for k in range(30):
        t = 200.0 + 0.1 * k
        clock[0] = t
        n.odom_cb(odom(-5.0, 0.0))
        m, n_cart, n_pole = cloud(t, -18.0, py)
        n.cloud_cb(m)                                   # tracking only (static input is separate)
        n.static_cb(m)
        if k >= 3:
            py -= 0.1
    got = pc2.read_points(out2[-1], skip_nans=False)
    z = np.asarray(got['z'])
    checks2 = [
        ('gated input: one /cloud_static per gated scan, none from /cloud', len(out2) == 30),
        ('gated input: moving panel removed', int(np.sum(np.abs(np.asarray(got['x']) + 18.0) < 0.5)) == 0),
        ('gated input: nothing below static_cloud_min_z', bool(np.all(z >= 0.5))),
        ('gated input: pole above the limit kept', int(np.sum(np.abs(np.asarray(got['x']) + 8.0) < 0.1)) > 0),
        ('no guard errors (gated input)', n.n_err == 0),
    ]
    for name, good in checks2:
        print(f'  {"ok  " if good else "FAIL"} {name}')
        ok &= good
    n.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
