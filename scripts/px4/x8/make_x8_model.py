#!/usr/bin/env python3
"""Build the inhouse-sim X8 Gazebo model (x8.sdf.jinja) from the Custom repo's X8 SDF.

The Custom repo (JOCIIIII/A4VAI-Custom-PX4-File x8/x8.sdf) supplies meshes, sensors and
plugin wiring; every physical parameter is replaced here (sources and the identification
from the X8 flight logs: README.md next to this file):

  mass / inertia      measured on the vehicle (10.4 kg; 0.603 / 0.751 / 1.005 kg m^2). The
                      base SDF left the 8 rotor links without <mass>, so Gazebo gave each
                      the SDF default 1 kg and 1 kg m^2 - an 18.4 kg vehicle.
  thrust              T-Motor MN5212 KV420 + 16x5.4CF datasheet (24 V), ESC throttle range
                      1000-2000 us, scaled to the battery voltage that makes the measured
                      mass hover at the PWM flown (1515 us). PX4 SITL hands the normalised
                      motor command u to Gazebo; the real vehicle maps it to PWM
                      PWM_MIN + u (PWM_MAX - PWM_MIN) = 1100 + 800 u (log 2026-01-07), so
                      the channel mapping omega = (u + offset) * scaling is the datasheet's
                      RPM curve linearised at hover (value and slope), clipped at the RPM
                      of 1900 us. The base model fed omega = 1000 u into k = 1.5e-4
                      (117 N per rotor, thrust-to-weight 9).
  yaw moment          datasheet torque/thrust 0.0199 m (steady drag torque) plus the rotor
                      spin-up reaction torque J_r * d(omega)/dt (<rotorInertia>, added to the
                      plugin by patch_motor_model.py). At yaw-loop frequencies the reaction is
                      an order of magnitude larger than the drag-torque change; without it the
                      vehicle's autotuned yaw gains limit-cycled in SITL.
  rotor drag          horizontal specific force / body velocity in hover flights.
  spin directions     from the vehicle's CA_ROTORn_KM (KM > 0: counter-clockwise); the
                      base model had all of them mirrored.
  collision           boxes for the landing skids and the hub instead of the bm.stl mesh.
                      With the mesh, Gazebo's IMU read a 1.1 m/s^2 forward specific force at
                      rest: the EKF started 6 deg tilted and its altitude ran 1-3 m off.
  accelerometer bias  off (turn-on and random walk). PX4 does not learn a 0.1-0.2 m/s^2 bias
                      within a flight and a fresh X8 has no saved calibration; with the base
                      model's bias terms the EKF's vertical velocity stayed 0.14 m/s off.
  IMU                 on the body link; acceleration = change of its velocity over PX4's 4 ms
                      IMU interval, saturated at 16 g (<accelFromVelocity>, <accelAverageTime>,
                      <accelSaturation>; patch_imu_plugin.py). Gazebo steps at 1 kHz and PX4 got
                      every 4th sample, and PX4's simulated 16 g FIFO wraps beyond its range, so
                      touchdowns reached the EKF missing or with the wrong sign: landings bounced
                      ~0.9 m. The skid contact is compliant (spring-damper) for the same reason.

usage: make_x8_model.py BASE_X8_SDF OUT_X8_SDF_JINJA
"""
import argparse
import math
import re

import numpy as np

G = 9.80665

# ---- vehicle (measured) ----
MASS_TOTAL = 10.4
INERTIA = (0.603079, 0.750835, 1.005022)
ROTOR_LINK_MASS = 0.005
IMU_GPS_MASS = 0.015 + 0.015               # /imu_link + gps::link of the base model

# ---- propulsion: T-Motor MN5212 KV420, 16x5.4CF, 24 V (store.tmotor.com test data) ----
DATASHEET = np.array([  # throttle %, thrust g, rpm, torque N m
    [50, 1530, 4815, 0.301], [55, 1795, 5210, 0.354], [60, 2160, 5642, 0.417],
    [65, 2433, 6032, 0.475], [75, 3150, 6811, 0.609], [85, 3818, 7489, 0.743],
    [100, 4903, 8455, 0.965]])
DATASHEET_V = 24.0
ESC_RANGE_US = (1000.0, 2000.0)            # fits the measured thrust slope (README)
PWM_CFG_US = (1100.0, 1900.0)              # PWM_MAIN_MIN/MAX of the vehicle (2026-01-07)
HOVER_PWM_US = 1515.0                      # mean hover PWM over the 7 flight logs

# ---- flight-identified (README) ----
YAW_AUTHORITY_RATIO = 1.0                  # steady yaw torque = datasheet torque/thrust
# rotor + prop spin inertia [kg m^2]: T-Motor 16x5.4 CF prop (~23 g over 0.41 m, tapered,
# ~2.1e-4) + MN5212 rotor bell (~90 g at ~27 mm, ~0.7e-4)
ROTOR_INERTIA = 2.8e-4
DRAG_SPECIFIC = 0.11                       # [1/s] horizontal specific force per body velocity
TIME_CONSTANT_UP = 0.025                   # [s] rotor speed lag (delay + motor lag of the
TIME_CONSTANT_DOWN = 0.025                 #     identified rate plant, see README)

# real CA_ROTORn_KM signs (rotor i in PX4 order): + = counter-clockwise
KM_SIGN = [+1, +1, -1, -1, -1, -1, +1, +1]

# ---- IMU: acceleration = change of the link velocity over PX4's IMU interval (Gazebo steps at
# 1 kHz, the MAVLink plugin forwards every 4th sample; patch_imu_plugin.py) ----
IMU_ACCEL_AVERAGE_S = 0.004
IMU_ACCEL_SATURATION = 156.8               # [m/s^2] above PX4 SITL's 16 g clip flag (156.75), below its int16 FIFO wrap (156.9)

# ---- contact: landing skids from bm.stl (bottom z -0.4366, skids at y +-0.235 over x +-0.227) ----
# compliant contact: a rigid one stops the vehicle within one 1 ms step (30-60 g at touchdown);
# skids that give spread a 0.7 m/s landing over ~30 ms (~7 g), within the 16 g IMU range
CONTACT_KP, CONTACT_KD = 1e5, 1e3          # [N/m], [N s/m]
COLLISION_BOXES = (  # name, centre xyz, size xyz [m]
    ('skid_left', (0, 0.235, -0.4216), (0.46, 0.03, 0.03)),
    ('skid_right', (0, -0.235, -0.4216), (0.46, 0.03, 0.03)),
    ('hub', (0, 0, 0.02), (0.5, 0.5, 0.15)))


def derive():
    thr, Tg, rpm, Q = DATASHEET.T
    T = Tg * G * 1e-3
    w = rpm * 2 * math.pi / 60
    kT = float(np.sum(T * w ** 2) / np.sum(w ** 4))            # T = kT w^2
    kQ = float(np.sum(Q * T) / np.sum(T * T))                   # Q = kQ T
    pw = np.polyfit(thr, w, 2)                                  # w(throttle %) at 24 V
    esc_lo, esc_hi = ESC_RANGE_US
    def thr_of_pwm(p):
        return (p - esc_lo) / (esc_hi - esc_lo) * 100.0
    T_hover = MASS_TOTAL * G / 8
    w_hover = math.sqrt(T_hover / kT)
    th_h = thr_of_pwm(HOVER_PWM_US)
    v_eff = DATASHEET_V * w_hover / np.polyval(pw, th_h)        # battery voltage of the fit
    vs = v_eff / DATASHEET_V
    lo, hi = PWM_CFG_US
    u_h = (HOVER_PWM_US - lo) / (hi - lo)
    dw_du = np.polyval(np.polyder(pw), th_h) * vs * (hi - lo) / (esc_hi - esc_lo) * 100.0
    scaling = float(dw_du)
    offset = float(w_hover / scaling - u_h)
    w_max = float(np.polyval(pw, thr_of_pwm(hi)) * vs)
    thr_mdl_fac = 0.3
    t_hover = thr_mdl_fac * u_h ** 2 + (1 - thr_mdl_fac) * u_h  # PX4 normalised hover thrust
    p = dict(kT=kT, kQ=kQ, momentConstant=kQ * YAW_AUTHORITY_RATIO, v_eff=v_eff, w_hover=w_hover,
             u_hover=u_h, t_hover=t_hover, input_scaling=scaling, input_offset=offset,
             maxRotVelocity=w_max, T_max=kT * w_max ** 2, T_hover=T_hover,
             rotorDragCoefficient=DRAG_SPECIFIC * MASS_TOTAL / (8 * w_hover),
             body_mass=MASS_TOTAL - 8 * ROTOR_LINK_MASS - IMU_GPS_MASS,
             thrust_slope_per_us=2 * kT * w_hover * scaling / (hi - lo))
    p['TW'] = 8 * p['T_max'] / (MASS_TOTAL * G)
    p['pf_max_thrust_per_rotor'] = T_hover / t_hover            # PathFollowing's linear model
    p['specific_force_slope'] = 8 * p['thrust_slope_per_us'] / MASS_TOTAL
    return p


def sub_block(text, start_pat, end_tag, fn):
    """Apply fn to the block starting at the regex start_pat up to the next end_tag."""
    m = re.search(start_pat, text)
    if not m:
        raise SystemExit(f'block not found: {start_pat}')
    end = text.index(end_tag, m.end()) + len(end_tag)
    return text[:m.start()] + fn(text[m.start():end]) + text[end:]


def set_tag(block, tag, value):
    new, n = re.subn(rf'<{tag}>[^<]*</{tag}>', f'<{tag}>{value}</{tag}>', block)
    if n == 0:
        raise SystemExit(f'tag {tag} not found in block')
    return new


def build(base, p):
    s = base.replace('<!-- DO NOT EDIT: Generated from x8.sdf.jinja -->\n', '')
    s = s.replace('<?xml version=\'1.0\'?>', "<?xml version='1.0'?>\n<!-- A4VAI X8 (inhouse sim): physical parameters "
                  "from scripts/px4/x8/make_x8_model.py, see README.md there -->", 1)
    s = re.sub(r"(<model name='x8'>\s*<pose>)[^<]*(</pose>)", r'\g<1>0 0 0 0 0 0\g<2>', s, count=1)
    s = sub_block(s, r"<link name='link_1'>", '</inertial>', lambda b: set_tag(b, 'mass', f"{p['body_mass']:.3f}"))
    rotor_inertial = (f'<inertial>\n        <pose>0 0 0 0 0 0</pose>\n        <mass>{ROTOR_LINK_MASS}</mass>\n'
                      '        <inertia>\n          <ixx>1e-06</ixx>\n          <ixy>0</ixy>\n          <ixz>0</ixz>\n'
                      '          <iyy>1e-06</iyy>\n          <iyz>0</iyz>\n          <izz>1e-06</izz>\n'
                      '        </inertia>\n      </inertial>')
    for i in range(8):
        ccw = KM_SIGN[i] > 0
        mesh = 'CCW.stl' if ccw else 'CW.stl'
        def rotor_link(b, mesh=mesh):
            b = re.sub(r'<inertial>\s*<pose>0 0 0 0 0 0</pose>\s*</inertial>', rotor_inertial, b, count=1)
            return re.sub(r'model://x8/meshes/C?CW\.stl', f'model://x8/meshes/{mesh}', b)
        s = sub_block(s, rf"<link name='rotor_{i}'>", '</link>', rotor_link)
        def motor(b, ccw=ccw):
            b = set_tag(b, 'turningDirection', 'ccw' if ccw else 'cw')
            b = set_tag(b, 'timeConstantUp', f'{TIME_CONSTANT_UP:g}')
            b = set_tag(b, 'timeConstantDown', f'{TIME_CONSTANT_DOWN:g}')
            b = set_tag(b, 'maxRotVelocity', f"{p['maxRotVelocity']:.1f}")
            b = set_tag(b, 'motorConstant', f"{p['kT']:.4e}")
            b = set_tag(b, 'momentConstant', f"{p['momentConstant']:.4f}")
            b = set_tag(b, 'rotorDragCoefficient', f"{p['rotorDragCoefficient']:.3e}")
            if '<rotorInertia>' in b:
                b = set_tag(b, 'rotorInertia', f'{ROTOR_INERTIA:.3e}')
            else:
                b = re.sub(r'(\s*)(<momentConstant>[^<]*</momentConstant>)',
                           rf'\1\2\1<rotorInertia>{ROTOR_INERTIA:.3e}</rotorInertia>', b, count=1)
            return b
        s = sub_block(s, rf"<plugin name='motor_{i}_model'", '</plugin>', motor)
        def channel(b):
            b = set_tag(b, 'input_offset', f"{p['input_offset']:.4f}")
            return set_tag(b, 'input_scaling', f"{p['input_scaling']:.1f}")
        s = sub_block(s, rf"<channel name='rotor{i}'>", '</channel>', channel)
    surface = (f'<surface><contact><ode><kp>{CONTACT_KP:g}</kp><kd>{CONTACT_KD:g}</kd><max_vel>0.1</max_vel>'
               '<min_depth>0.001</min_depth></ode></contact><friction><ode/></friction></surface>')
    boxes = '\n      '.join(
        f"<collision name='{n}'><pose>{c[0]} {c[1]} {c[2]} 0 0 0</pose><geometry><box><size>{d[0]} {d[1]} {d[2]}"
        f"</size></box></geometry>{surface}</collision>" for n, c, d in COLLISION_BOXES)
    s = sub_block(s, r"<collision name='link_1_collision'>", '</collision>', lambda b: boxes)
    def imu(b):
        b = set_tag(b, 'accelerometerTurnOnBiasSigma', '0')
        b = set_tag(b, 'accelerometerRandomWalk', '0')
        b = set_tag(b, 'linkName', 'link_1')
        if '<accelFromVelocity>' not in b:
            b = b.replace('<imuTopic>', '<accelFromVelocity>1</accelFromVelocity>\n'
                          f'      <accelAverageTime>{IMU_ACCEL_AVERAGE_S}</accelAverageTime>\n'
                          f'      <accelSaturation>{IMU_ACCEL_SATURATION}</accelSaturation>\n      <imuTopic>', 1)
        return b
    s = sub_block(s, r"<plugin name='rotors_gazebo_imu_plugin'", '</plugin>', imu)
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('base_sdf')
    ap.add_argument('out_sdf')
    a = ap.parse_args()
    p = derive()
    for k in ('kT', 'kQ', 'momentConstant', 'v_eff', 'w_hover', 'u_hover', 't_hover', 'input_offset',
              'input_scaling', 'maxRotVelocity', 'T_max', 'TW', 'rotorDragCoefficient', 'body_mass',
              'specific_force_slope', 'pf_max_thrust_per_rotor'):
        print(f'{k:24s} {p[k]:.6g}')
    out = build(open(a.base_sdf).read(), p)
    open(a.out_sdf, 'w').write(out)
    print('wrote', a.out_sdf)


if __name__ == '__main__':
    main()
