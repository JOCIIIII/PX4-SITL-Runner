#!/usr/bin/env python3
"""Gazebo Classic world for a ground_courses layout (layouts/*.json, X = north, Y = east): the stock empty world
(UGV) or boat world (USV) plus every obstacle as a static box / cylinder with collision. Prints the spawn pose for
ground_run.sh (GROUND_SPAWN_POSE: the vehicle at START facing north) and the PX4 local origin for ground_driver.py
(--frame local --origin X,Y).
  ground_world.py LAYOUT ugv|usv OUT_WORLD"""
import json
import math
import os
import sys

PX4_WORLDS = os.path.expanduser('~/Documents/A4VAI-SITL/PX4-Autopilot/PX4-Autopilot/Tools/simulation/gazebo-classic/'
                                'sitl_gazebo-classic/worlds')


def model(o, veh):
    x_gz, y_gz = o['pos'][1], o['pos'][0]               # ENU: x = east, y = north
    if veh == 'usv':                                     # stand in the water: from 2 m below the surface
        z0, h = -2.0, o['size'][2] + 2.0
    else:
        z0, h = float(o.get('z', 0.0)), o['size'][2]
    yaw = math.radians(90.0 - o.get('yaw_deg', 0.0))     # box local x at NED yaw psi -> ENU yaw 90 - psi
    if o['shape'] == 'cylinder':
        geo = f"<cylinder><radius>{o['size'][0] / 2.0}</radius><length>{h}</length></cylinder>"
        yaw = 0.0
    else:
        geo = f"<box><size>{o['size'][0]} {o['size'][1]} {h}</size></box>"
    colour = 'Gazebo/Grey' if veh == 'usv' else 'Gazebo/Orange'
    return f"""    <model name="{o['name']}">
      <static>true</static>
      <pose>{x_gz} {y_gz} {z0 + h / 2.0} 0 0 {yaw}</pose>
      <link name="link">
        <collision name="collision"><geometry>{geo}</geometry></collision>
        <visual name="visual"><geometry>{geo}</geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{colour}</name></script></material>
        </visual>
      </link>
    </model>
"""


def main():
    lay = json.load(open(sys.argv[1]))
    veh, out = sys.argv[2], sys.argv[3]
    base = open(os.path.join(PX4_WORLDS, 'boat.world' if veh == 'usv' else 'empty.world')).read()
    if veh == 'ugv':   # contact correcting velocity limited: keeps ODE's correction velocities out of the IMU (setup.sh)
        base = base.replace('<contact_max_correcting_vel>100</contact_max_correcting_vel>', '<contact_max_correcting_vel>0.1</contact_max_correcting_vel>')
        base = base.replace('<contact_surface_layer>0.001</contact_surface_layer>', '<contact_surface_layer>0.002</contact_surface_layer>')
        assert '<contact_max_correcting_vel>0.1<' in base
    models = ''.join(model(o, veh) for o in lay['obstacles'])
    i = base.rindex('</world>')
    open(out, 'w').write(base[:i] + models + base[i:])
    sx, sy = lay['waypoints']['START']
    z = 0.10 if veh == 'usv' else 0.08
    print(f'GROUND_SPAWN_POSE="-x {sy} -y {sx} -z {z} -Y 1.570796"')
    print(f'ORIGIN={sx},{sy}')


if __name__ == '__main__':
    main()
