#! /bin/bash

# ROS2 APP: nothing - keeps the onboard container up with the Mid-360 driver and foxglove_bridge for the bench and
# calibration tools (scripts/dev/onboard_real.sh calib NAME, then onboard_real.sh tool lidar_mount_calib.py ...).
echo "[$(basename "$0")] idle: Mid-360 driver and PX4 topics up, no app"
sleep infinity
