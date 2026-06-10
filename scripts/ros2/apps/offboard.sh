#! /bin/bash

# ROS2 APP: px4_offboard_control (offboard control example).
# Builds the node AND its dependency px4_msgs (--packages-up-to), then runs it.
# Runs inside the ros2-env container, alongside the uXRCE-DDS agent
# (same container => localhost DDS, so /fmu/* topics are directly visible).

# WORKSPACE_DIR / BASE_DIR are exported by sitl-px4.sh; fall back if run standalone.
if [ -z "${WORKSPACE_DIR}" ]; then
    WORKSPACE_DIR=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
fi

ROS2_WS=${WORKSPACE_DIR}/ros2_ws
cd ${ROS2_WS} || exit 1

# px4_msgs is a build dependency of px4_offboard_control. If its install is
# incomplete (no package.sh), force a clean rebuild of it so colcon can resolve it.
if [ ! -f install/px4_msgs/share/px4_msgs/package.sh ]; then
    echo "[$(basename "$0")] px4_msgs install incomplete -> clean rebuilding it"
    rm -rf build/px4_msgs install/px4_msgs
fi

# BUILD THE NODE AND ITS DEPENDENCIES (px4_msgs). First run is slow (px4_msgs),
# subsequent runs are fast (colcon skips up-to-date packages).
colcon build --symlink-install --packages-up-to px4_offboard_control || {
    echo "[$(basename "$0")] colcon build FAILED. Aborting offboard app."
    exit 1
}

# SOURCE THE WORKSPACE SO THE NEW ENTRY POINT IS AVAILABLE
source ${ROS2_WS}/install/setup.bash

# RUN THE NODE
ros2 run px4_offboard_control offboard_control
