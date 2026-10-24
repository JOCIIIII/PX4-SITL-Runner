#!/usr/bin/env python3
"""Build the inhouse-sim X500 Gazebo Classic model (x500.sdf.jinja).

PX4 v1.16 ships the X500 for the new Gazebo only (Tools/simulation/gz/models/x500*). This model
takes its geometry and meshes from there, the plugin wiring (MAVLink interface, GPS, mag, baro)
from the Gazebo Classic iris template, the IMU and contact settings from the X8 model
(scripts/px4/x8), and its dynamics from the A4VAI X500's own flight logs (README.md here):

  thrust          omega = 100 + 1000 u (u = PX4's normalised motor command, armed idle 100 as the
                  iris), thrust = k omega^2 with k set so the vehicle hovers at the u flown (0.636,
                  median of 5 flights): thrust-to-weight 2.2.
  roll / pitch    inertia set so the rate plant matches the identified one: PX4's allocation
                  turns a unit roll/pitch torque command into +-0.707 per motor (quad x,
                  normalised), so torque per unit = 4 arm dT/du 0.707 and Ixx = that / K
                  (K = 125 rad/s^2 per unit, motor lag 21 ms; scripts/dev/x8_sysid.py).
  yaw             unit yaw command = +-1 per motor; Izz = 1.85 Ixx (flat quad: PX4's gz X500
                  1.84, iris 1.90); drag torque kQ * thrust plus the rotor spin-up reaction torque
                  (<rotorInertia>, scripts/px4/x8/patch_motor_model.py), sized from SITL flights
                  so the yaw plant has the vehicle's gain (21 rad/s^2 per unit) and no lag (KQ,
                  ROTOR_INERTIA). The roll/pitch inertia carries the SITL torque factor measured
                  in the first SITL flight (SITL_TORQUE_RP).
  mass            the vehicle's all-up mass is not in the logs: MASS_TOTAL is PX4's X500
                  reference. Hover command, thrust-to-weight, rate gains and drag per mass are
                  matched, so the flight dynamics do not depend on it; Ixx/Izz are effective
                  values for that mass.
  rotor drag      DRAG_SPECIFIC [1/s] horizontal specific force per velocity; the X500 logs
                  are too slow (< 2 m/s) to identify it - the X8's identified value is used.
  IMU             on the body link, acceleration from its velocity over PX4's 4 ms interval,
                  16 g clip, no bias (as the X8, scripts/px4/x8/make_x8_model.py).
  contact         skid bars from PX4's gz X500, compliant (spring-damper scaled from the X8 to
                  the same natural frequency and damping for this mass).

usage: make_x500_model.py IRIS_SDF_JINJA OUT_X500_SDF_JINJA
"""
import argparse
import math
import re

G = 9.80665

# ---- vehicle ----
MASS_TOTAL = 2.0                 # [kg] PX4 gz X500 reference (real all-up mass unknown)
ARM = 0.174                      # rotor |x| = |y| [m] (Holybro X500 V2, 500 mm wheelbase)
ROTOR_Z = 0.06
PROP_RADIUS = 0.14               # PX4 gz X500 prop (11 in)
ROTOR_LINK_MASS = 0.016          # PX4 gz X500 rotor link
ROTOR_LINK_INERTIA = (3.85e-07, 2.61e-05, 2.65e-05)
GPS_MASS = 0.015                 # model://gps link

# ---- identified from the X500 flight logs (README.md) ----
U_HOVER = 0.636
OMEGA_PER_UNIT = 1000.0          # rotor speed per unit command [rad/s] (MAVLink channel input_scaling)
OMEGA_IDLE = 100.0               # armed idle (channel zero_position_armed, iris): omega = 100 + 1000 u -
                                 # like the vehicle's ESC, which idles at u = 0 (PWM 1100 of 1000-2000)
K_RP = 125.0                     # [rad/s^2 per unit normalised torque]
K_YAW = 21.0
TAU_M = 0.021                    # [s] motor lag of the roll/pitch fits
# Yaw: drag torque kQ * thrust plus the rotor spin-up reaction torque J_r d(omega)/dt, which leads
# it. SITL flights against the vehicle's yaw plant (K 21, no lag):
#   kQ 0.030 (steady match), J_r 8e-6 (lag-cancelling only)  -> K 22.2, lag 69 ms   (x500_step_b)
#   kQ 0.030, J_r 1.0e-4 (11 in prop ~15 g + bell, physical) -> K 34.8, no lag      (x500_step_c)
# both scaled by 21 / 34.8: the vehicle's K with no lag, and a prop-sized kQ and J_r
KQ = 0.0182                      # [m] drag torque / thrust
ROTOR_INERTIA = 6.0e-5           # [kg m^2]
IZZ_OVER_IXX = 1.85
DRAG_SPECIFIC = 0.11             # [1/s] (X8 value)
ALLOC_RP, ALLOC_YAW = 0.7071, 1.0
# The SITL chain gives less torque per unit than 4 arm dT/du ALLOC: flown with the formula's
# inertias (x500_step_a) the identified roll/pitch K was 0.69 x the formula's. The inertia absorbs
# that, so the SITL rate plant matches the vehicle's (x500_step_b, c: 115-128 against 114-147).
SITL_TORQUE_RP = 0.694

# ---- IMU / contact ----
IMU_ACCEL_AVERAGE_S = 0.004
IMU_ACCEL_SATURATION = 156.8
CONTACT_KP, CONTACT_KD = 2.0e4, 200.0
COLLISION_BOXES = (  # name, centre xyz, size xyz [m] (PX4 gz X500 frame + skids)
    ('hub', (0, 0, 0.007), (0.354, 0.354, 0.05)),
    ('skid_left', (0, 0.132, -0.2195), (0.25, 0.015, 0.015)),
    ('skid_right', (0, -0.132, -0.2195), (0.25, 0.015, 0.015)))
# rotors in PX4 order (CA_ROTORn of the vehicle: KM > 0 = counter-clockwise); Gazebo y is left
ROTORS = (  # x, y, direction, prop mesh
    (ARM, -ARM, 'ccw', '1345_prop_ccw.stl'),
    (-ARM, ARM, 'ccw', '1345_prop_ccw.stl'),
    (ARM, ARM, 'cw', '1345_prop_cw.stl'),
    (-ARM, -ARM, 'cw', '1345_prop_cw.stl'))


def derive():
    T_h = MASS_TOTAL * G / 4
    w_h = OMEGA_IDLE + OMEGA_PER_UNIT * U_HOVER
    w_max = OMEGA_IDLE + OMEGA_PER_UNIT
    k = T_h / w_h ** 2
    dT_du = 2 * k * w_h * OMEGA_PER_UNIT
    ixx = SITL_TORQUE_RP * 4 * ARM * dT_du * ALLOC_RP / K_RP
    izz = IZZ_OVER_IXX * ixx
    kq = KQ
    j_r = ROTOR_INERTIA
    return dict(T_hover=T_h, w_hover=w_h, w_max=w_max, motorConstant=k, dT_du=dT_du, ixx=ixx, izz=izz,
                momentConstant=kq, rotorInertia=j_r, TW=4 * k * w_max ** 2 / (MASS_TOTAL * G),
                rotorDragCoefficient=DRAG_SPECIFIC * MASS_TOTAL / (4 * w_h),
                body_mass=MASS_TOTAL - 4 * ROTOR_LINK_MASS - GPS_MASS,
                prop_tip_radius=math.hypot(ARM, ARM) + PROP_RADIUS)


def contact():
    return (f'<surface><contact><ode><kp>{CONTACT_KP:g}</kp><kd>{CONTACT_KD:g}</kd>'
            f'<max_vel>0.1</max_vel><min_depth>0.001</min_depth></ode></contact><friction><ode/></friction></surface>')


def mesh_visual(name, pose, uri, scale=1.0, material=None):
    mat = (f'<material><script><name>{material}</name>'
           f'<uri>file://media/materials/scripts/gazebo.material</uri></script></material>') if material else ''
    return (f"      <visual name='{name}'><pose>{pose}</pose><geometry><mesh><scale>{scale} {scale} {scale}</scale>"
            f"<uri>model://x500/meshes/{uri}</uri></mesh></geometry>{mat}</visual>\n")


def build(iris):
    p = derive()
    out = ["<!-- DO NOT EDIT: Generated from x500.sdf.jinja (scripts/px4/x500/make_x500_model.py) -->\n",
           "<sdf version='1.6'>\n  <model name='x500'>\n"]
    # body
    out.append("    <link name='base_link'>\n      <pose>0 0 0 0 0 0</pose>\n")
    out.append(f"      <inertial><pose>0 0 0 0 0 0</pose><mass>{p['body_mass']:.4f}</mass><inertia>"
               f"<ixx>{p['ixx']:.5f}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{p['ixx']:.5f}</iyy><iyz>0</iyz>"
               f"<izz>{p['izz']:.5f}</izz></inertia></inertial>\n")
    for name, c, s in COLLISION_BOXES:
        out.append(f"      <collision name='{name}'><pose>{c[0]} {c[1]} {c[2]} 0 0 0</pose><geometry><box>"
                   f"<size>{s[0]} {s[1]} {s[2]}</size></box></geometry>{contact()}</collision>\n")
    out.append(mesh_visual('frame', '0 0 .025 0 0 3.141592654', 'NXP-HGD-CF.dae'))
    for i, (x, y, _, _) in enumerate(ROTORS):
        out.append(mesh_visual(f'motor_base_{i}', f'{x} {y} .032 0 0 -.45', '5010Base.dae'))
    out.append("      <gravity>1</gravity>\n      <velocity_decay/>\n    </link>\n")
    # rotors
    ixx_r, iyy_r, izz_r = ROTOR_LINK_INERTIA
    for i, (x, y, _, prop) in enumerate(ROTORS):
        out.append(f"    <link name='rotor_{i}'>\n      <pose>{x} {y} {ROTOR_Z} 0 0 0</pose>\n"
                   f"      <inertial><pose>0 0 0 0 0 0</pose><mass>{ROTOR_LINK_MASS}</mass><inertia><ixx>{ixx_r}</ixx>"
                   f"<ixy>0</ixy><ixz>0</ixz><iyy>{iyy_r}</iyy><iyz>0</iyz><izz>{izz_r}</izz></inertia></inertial>\n"
                   f"      <collision name='rotor_{i}_collision'><pose>0 0 0 0 0 0</pose><geometry><cylinder>"
                   f"<length>0.005</length><radius>{PROP_RADIUS}</radius></cylinder></geometry><surface><contact><ode/>"
                   f"</contact><friction><ode/></friction></surface></collision>\n")
        out.append(mesh_visual(f'rotor_{i}_visual', '-0.022 -0.14638461538461536 -0.016 0 0 0', prop,
                               0.8461538461538461, 'Gazebo/DarkGrey'))
        out.append(mesh_visual(f'rotor_{i}_bell', '0 0 -.032 0 0 0', '5010Bell.dae'))
        out.append("      <gravity>1</gravity>\n      <velocity_decay/>\n    </link>\n")
        out.append(f"    <joint name='rotor_{i}_joint' type='revolute'><child>rotor_{i}</child><parent>base_link</parent>"
                   "<axis><xyz>0 0 1</xyz><limit><lower>-1e+16</lower><upper>1e+16</upper></limit><dynamics>"
                   "<spring_reference>0</spring_reference><spring_stiffness>0</spring_stiffness></dynamics>"
                   "<use_parent_model_frame>1</use_parent_model_frame></axis></joint>\n")
    out.append("    <plugin name='rosbag' filename='libgazebo_multirotor_base_plugin.so'><robotNamespace/>"
               "<linkName>base_link</linkName><rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim></plugin>\n")
    for i, (_, _, d, _) in enumerate(ROTORS):
        out.append(f"""    <plugin name='motor_model_{i}' filename='libgazebo_motor_model.so'>
      <robotNamespace/>
      <jointName>rotor_{i}_joint</jointName>
      <linkName>rotor_{i}</linkName>
      <turningDirection>{d}</turningDirection>
      <timeConstantUp>{TAU_M}</timeConstantUp>
      <timeConstantDown>{TAU_M}</timeConstantDown>
      <maxRotVelocity>{p['w_max']:g}</maxRotVelocity>
      <motorConstant>{p['motorConstant']:.4e}</motorConstant>
      <momentConstant>{p['momentConstant']:.4f}</momentConstant>
      <rotorInertia>{p['rotorInertia']:.3e}</rotorInertia>
      <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
      <motorNumber>{i}</motorNumber>
      <rotorDragCoefficient>{p['rotorDragCoefficient']:.4e}</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <motorSpeedPubTopic>/motor_speed/{i}</motorSpeedPubTopic>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    </plugin>
""")
    # sensors and the MAVLink interface: the iris template's, unchanged
    m = re.search(r"\n(    <include>\s*<uri>model://gps</uri>.*?</plugin>\n)\s*<static>", iris, re.S)
    if not m:
        raise SystemExit('iris template: GPS include .. MAVLink interface block not found')
    out.append(m.group(1))
    out.append(f"""    <static>0</static>
    <plugin name='rotors_gazebo_imu_plugin' filename='libgazebo_imu_plugin.so'>
      <robotNamespace/>
      <linkName>base_link</linkName>
      <accelFromVelocity>1</accelFromVelocity>
      <accelAverageTime>{IMU_ACCEL_AVERAGE_S}</accelAverageTime>
      <accelSaturation>{IMU_ACCEL_SATURATION}</accelSaturation>
      <imuTopic>/imu</imuTopic>
      <gyroscopeNoiseDensity>0.00018665</gyroscopeNoiseDensity>
      <gyroscopeRandomWalk>0</gyroscopeRandomWalk>
      <gyroscopeBiasCorrelationTime>1000.0</gyroscopeBiasCorrelationTime>
      <gyroscopeTurnOnBiasSigma>0</gyroscopeTurnOnBiasSigma>
      <accelerometerNoiseDensity>0.00186</accelerometerNoiseDensity>
      <accelerometerRandomWalk>0</accelerometerRandomWalk>
      <accelerometerBiasCorrelationTime>300.0</accelerometerBiasCorrelationTime>
      <accelerometerTurnOnBiasSigma>0</accelerometerTurnOnBiasSigma>
    </plugin>
  </model>
</sdf>
""")
    return ''.join(out), p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('iris_jinja')
    ap.add_argument('out')
    a = ap.parse_args()
    text, p = build(open(a.iris_jinja).read())
    open(a.out, 'w').write(text)
    print(f"{a.out}: body {p['body_mass']:.3f} kg (total {MASS_TOTAL}), Ixx=Iyy {p['ixx']:.4f}, Izz {p['izz']:.4f} kg m^2, "
          f"k {p['motorConstant']:.4e}, hover {p['T_hover']:.2f} N/rotor at {p['w_hover']:.0f} rad/s, T/W {p['TW']:.2f}, "
          f"kQ {p['momentConstant']:.4f} m, J_r {p['rotorInertia']:.2e}, rotor drag {p['rotorDragCoefficient']:.3e}, prop tips {p['prop_tip_radius']:.3f} m")


if __name__ == '__main__':
    main()
