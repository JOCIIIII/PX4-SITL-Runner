#!/usr/bin/env python3
"""Add the rotor spin-up reaction torque to PX4's Gazebo-classic motor plugin (idempotent).

When an ESC changes a motor's speed, the motor torque that accelerates the rotor reacts on the
frame: -dir * J_r * d(omega)/dt about the rotor axis. For the X8 (16" props, 25 ms motor lag)
it is an order of magnitude larger than the change of the aerodynamic drag torque at yaw-loop
frequencies, and it is what lets the vehicle's autotuned yaw gains work. The stock plugin only
applies the steady drag torque (momentConstant * thrust) and sets the rotor joint velocity
directly, so no reaction reaches the body; with the vehicle's gains the SITL X8 yaw then
limit-cycled (x8_replay_b). New SDF parameter <rotorInertia> [kg m^2] (default 0 = stock).

usage: patch_motor_model.py SITL_GAZEBO_CLASSIC_DIR
"""
import sys

MARK = 'A4VAI rotor spin-up reaction torque'


def patch(path, anchor, insert, before=False):
    s = open(path).read()
    if MARK in s:
        return False
    if s.count(anchor) != 1:
        sys.exit(f'[patch_motor_model] anchor not found once in {path}: {anchor!r}')
    s = s.replace(anchor, insert + anchor if before else anchor + insert)
    open(path, 'w').write(s)
    return True


def main():
    root = sys.argv[1].rstrip('/')
    h = f'{root}/include/gazebo_motor_model.h'
    c = f'{root}/src/gazebo_motor_model.cpp'
    a = patch(h, '  double moment_constant_{kDefaultMomentConstant};\n',
              f'  // {MARK}\n  double rotor_inertia_{{0.0}};\n  double prev_ref_motor_rot_vel_{{0.0}};\n'
              '  bool have_prev_ref_{false};\n')
    b1 = open(c).read()
    if MARK not in b1:
        patch(c, '  getSdfParam<double>(_sdf, "momentConstant", moment_constant_, moment_constant_);\n',
              f'  // {MARK}\n  getSdfParam<double>(_sdf, "rotorInertia", rotor_inertia_, rotor_inertia_);\n')
        # the marker is now in the file; patch the update step with a distinct anchor check
        s = open(c).read()
        anchor = '  ref_motor_rot_vel = rotor_velocity_filter_->updateFilter(ref_motor_rot_vel_, sampling_time_);\n'
        if s.count(anchor) != 1:
            sys.exit('[patch_motor_model] filter-update anchor not found once')
        add = ('  // ' + MARK + ': -dir * J_r * d(omega)/dt on the frame, about the rotor axis\n'
               '  if (rotor_inertia_ > 0.0 && sampling_time_ > 0.0) {\n'
               '    if (have_prev_ref_) {\n'
               '      double rotor_accel = (ref_motor_rot_vel - prev_ref_motor_rot_vel_) / sampling_time_;\n'
               '      ignition::math::Vector3d reaction(0, 0, -turning_direction_ * rotor_inertia_ * rotor_accel);\n'
               '      parent_links.at(0)->AddRelativeTorque(pose_difference.Rot().RotateVector(reaction));\n'
               '    }\n'
               '    prev_ref_motor_rot_vel_ = ref_motor_rot_vel;\n'
               '    have_prev_ref_ = true;\n'
               '  }\n')
        open(c, 'w').write(s.replace(anchor, anchor + add))
        print('[patch_motor_model] patched', c, '(header patched)' if a else '')
    else:
        print('[patch_motor_model] already patched')


if __name__ == '__main__':
    main()
