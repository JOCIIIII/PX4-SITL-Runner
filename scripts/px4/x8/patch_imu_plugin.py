#!/usr/bin/env python3
"""Let PX4's Gazebo-classic IMU plugin measure acceleration from the link's velocity, averaged
over the interval PX4 samples it at (idempotent).

Gazebo steps the physics at 1 kHz and the MAVLink plugin forwards every 4th IMU sample to PX4
(250 Hz) without averaging. Acceleration is a derivative: a touchdown stops the vehicle within
one 1 ms step, so the impact lives in one sample, and 3 times in 4 PX4 never saw it. The stock
reading (Gazebo >= 9: force on the link / mass) has the same problem. PX4's EKF then kept
believing the vehicle was descending after touchdown, AUTO.LAND raised thrust and the X8
bounced up to ~0.9 m (x8_sp5_a, x8_replay_d). Here the acceleration is the change of the link's
world velocity over the last <accelAverageTime> seconds (the 4 ms PX4 interval), so every
impulse reaches PX4 in exactly one sample.

PX4 SITL feeds accel 0 to a simulated 16 g FIFO (int16 counts of g/2048): a sample beyond
+-157 m/s^2 overflows and comes out with the wrong sign, without a clipping flag. A rigid
touchdown easily exceeds that, so the output is also saturated like a real 16 g sensor
(<accelSaturation>), which PX4 then flags as clipping.

New SDF parameters: <accelFromVelocity> (0 = stock behaviour), <accelAverageTime> [s]
(0 = one physics step) and <accelSaturation> [m/s^2] (0 = none).

usage: patch_imu_plugin.py SITL_GAZEBO_CLASSIC_DIR
"""
import sys

MARK = 'A4VAI accel from velocity'


def replace_once(text, anchor, new, path):
    if text.count(anchor) != 1:
        sys.exit(f'[patch_imu_plugin] anchor not found once in {path}: {anchor!r}')
    return text.replace(anchor, new)


def main():
    root = sys.argv[1].rstrip('/')
    h = f'{root}/include/gazebo_imu_plugin.h'
    c = f'{root}/src/gazebo_imu_plugin.cpp'
    hs, cs = open(h).read(), open(c).read()
    if MARK in hs and MARK in cs:
        print('[patch_imu_plugin] already patched')
        return
    if MARK not in hs:
        a = '  ignition::math::Vector3d velocity_prev_W_;\n'
        hs = replace_once(hs, a, a + (
            f'  // {MARK}\n'
            '  bool accel_from_velocity_{false};\n'
            '  double accel_average_time_{0.0};\n'
            '  double accel_saturation_{0.0};\n'
            '  std::deque<std::pair<double, ignition::math::Vector3d>> velocity_history_W_;\n'), h)
        hs = replace_once(hs, '#include <random>\n', '#include <deque>\n#include <random>\n', h)
        open(h, 'w').write(hs)
    if MARK not in cs:
        a = '  getSdfParam<std::string>(_sdf, "imuTopic", imu_topic_, kDefaultImuTopic);\n'
        cs = replace_once(cs, a, a + (
            f'  // {MARK}\n'
            '  getSdfParam<bool>(_sdf, "accelFromVelocity", accel_from_velocity_, accel_from_velocity_);\n'
            '  getSdfParam<double>(_sdf, "accelAverageTime", accel_average_time_, accel_average_time_);\n'
            '  getSdfParam<double>(_sdf, "accelSaturation", accel_saturation_, accel_saturation_);\n'
            '  if (accel_from_velocity_)\n'
            '    std::cout << "[gazebo_imu_plugin] " << link_name_ << ": acceleration from the link velocity over "\n'
            '              << accel_average_time_ << " s" << std::endl;\n'), c)
        a = ('  ignition::math::Vector3d acceleration_I = link_->RelativeLinearAccel() - '
             'C_W_I.RotateVectorReverse(gravity_W_);\n')
        cs = replace_once(cs, a, (
            f'  // {MARK}: change of the world velocity over the last accel_average_time_ (PX4 takes\n'
            '  // every n-th sample; an impact lasts one physics step)\n'
            '  ignition::math::Vector3d acceleration_I;\n'
            '  if (accel_from_velocity_) {\n'
            '    const ignition::math::Vector3d velocity_current_W = link_->WorldLinearVel();\n'
            '    velocity_history_W_.emplace_back(t, velocity_current_W);\n'
            '    const double window = std::max(accel_average_time_, dt);\n'
            '    while (velocity_history_W_.size() > 2 && velocity_history_W_[1].first <= t - window + 1e-9)\n'
            '      velocity_history_W_.pop_front();\n'
            '    const double span = t - velocity_history_W_.front().first;\n'
            '    const ignition::math::Vector3d acceleration_W = span > 0.0 ?\n'
            '        (velocity_current_W - velocity_history_W_.front().second) / span : ignition::math::Vector3d::Zero;\n'
            '    acceleration_I = C_W_I.RotateVectorReverse(acceleration_W - gravity_W_);\n'
            '    if (accel_saturation_ > 0.0) {  // a real sensor saturates; PX4\'s int16 FIFO would wrap\n'
            '      acceleration_I.X(ignition::math::clamp(acceleration_I.X(), -accel_saturation_, accel_saturation_));\n'
            '      acceleration_I.Y(ignition::math::clamp(acceleration_I.Y(), -accel_saturation_, accel_saturation_));\n'
            '      acceleration_I.Z(ignition::math::clamp(acceleration_I.Z(), -accel_saturation_, accel_saturation_));\n'
            '    }\n'
            '  } else {\n'
            '    acceleration_I = link_->RelativeLinearAccel() - C_W_I.RotateVectorReverse(gravity_W_);\n'
            '  }\n'), c)
        open(c, 'w').write(cs)
    print('[patch_imu_plugin] patched', c)


if __name__ == '__main__':
    main()
