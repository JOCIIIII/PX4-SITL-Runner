#!/usr/bin/env python3
"""Add gazebo_zone_wind_plugin (fan-bank wind for scenario B of the test map, see the .cpp next to this file) to
PX4's sitl_gazebo-classic build (idempotent): copies the source into src/ and adds the library to
CMakeLists.txt next to the IMU plugin. The vehicle SDFs (x8 / x500 jinja) load it; without its config file
it does nothing.

usage: patch_zone_wind_plugin.py SITL_GAZEBO_CLASSIC_DIR
"""
import filecmp
import os
import shutil
import sys

MARK = 'A4VAI zone wind'
NAME = 'gazebo_zone_wind_plugin'


def main():
    root = sys.argv[1].rstrip('/')
    src = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'{NAME}.cpp')
    dst = f'{root}/src/{NAME}.cpp'
    if not os.path.exists(dst) or not filecmp.cmp(src, dst, shallow=False):
        shutil.copy(src, dst)
        print(f'[patch_zone_wind] {dst} updated')
    cm = f'{root}/CMakeLists.txt'
    text = open(cm).read()
    if MARK in text:
        print('[patch_zone_wind] CMakeLists.txt already patched')
        return
    lib = 'add_library(gazebo_imu_plugin SHARED src/gazebo_imu_plugin.cpp)\n'
    entry = '  gazebo_imu_plugin\n'
    if text.count(lib) != 1 or text.count('set(plugins\n  gazebo_airspeed_plugin\n') != 1:
        sys.exit(f'[patch_zone_wind] anchors not found in {cm}')
    text = text.replace(lib, lib + f'# {MARK}\nadd_library({NAME} SHARED src/{NAME}.cpp)\n')
    text = text.replace('set(plugins\n  gazebo_airspeed_plugin\n', f'set(plugins\n  gazebo_airspeed_plugin\n  {NAME}\n', 1)
    open(cm, 'w').write(text)
    print('[patch_zone_wind] CMakeLists.txt patched')


if __name__ == '__main__':
    main()
