#!/bin/bash

# INITIAL STATEMENTS
# >>>----------------------------------------------------

# SET THE BASIC ENVIRONMENT VARIABLE
export TERM=xterm-256color

# SET THE BASE DIRECTORY
BASE_DIR=$(dirname $(readlink -f "$0"))

# SOURCE THE ENVIRONMENT AND FUNCTION DEFINITIONS
for file in ${BASE_DIR}/include/*.sh; do
    source ${file}
done
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


# MAIN STATEMENTS
# >>>----------------------------------------------------

# CHCEK IF DIRECTORY PX4-Autopilot and build EXISTS
CheckDirExists "PX4-Autopilot"
CheckDirExists "PX4-Autopilot/build"

# CHECK IF PX4-AUTOPILOT IS ALREADY BUILT
CheckDirEmpty "PX4-Autopilot/build"

# CONFIGURE CONNECTION TO QGC
EchoGreen "[$(basename $0)] SETTING CONNECTION TO QGC."
if [ ! -z "${QGC_IP}" ]; then 
    EchoGreen "[$(basename $0)] PX4 WILL BE LISTEN TO THE QGC AT ${QGC_IP}."

    sed -i "s/-x -u \$udp_gcs_port_local/-x -t ${QGC_IP} -u \$udp_gcs_port_local/g" \
        PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/px4-rc.mavlink
else
    EchoYellow "[$(basename $0)] PX4 WILL BE LISTEN TO THE QGC AT localhost."
fi
EchoGreen $(EchoBoxLine)


# PREVENT GAZEBO FROM RUNNING
EchoGreen "[$(basename $0)] PREVENTING GAZEBO FROM RUNNING."

# RESET THE sitl_run.sh SCRIPT
EchoYellow "[$(basename $0)] RESETTING THE sitl_run.sh SCRIPT. USING GIT CHECKOUT."
git -C PX4-Autopilot checkout -- Tools/simulation/gazebo-classic/sitl_run.sh

sed -i -E \
    "s|(--model-name=\\\$\{model\}).*?(-x [0-9.]+ -y [0-9.]+ -z [0-9.]+)(.*)|\\1 $GAZEBO_POSE \\3|" \
    PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_run.sh

# FIND THE TARGET LINE WITH STRING "$debugger" == "lldb" AND DELETE THE LINE AFTER IT
START_LINE=$(grep -wn 'if \[ "$debugger" == "lldb" \]; then' PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_run.sh | cut -d: -f1)
END_LINE=$(grep -wn 'kill -9 $SIM_PID' PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_run.sh | cut -d: -f1)

if [ -z "${START_LINE}" ] || [ -z "${END_LINE}" ]; then
    EchoRed "[$(basename $0)] TARGET LINE NOT FOUND."
    EchoYellow "[$(basename $0)] IT SEEMS THAT THE SCRIPT HAS ALREADY BEEN RUN BEFORE."
    EchoYellow "[$(basename $0)] OR, SCRIPT MIGHT BE UPDATED. PLEASE CHECK THE SCRIPT MANUALLY."
else
    EchoGreen "[$(basename $0)] TARGET LINE FOUND AT LINE NUMBER ${START_LINE}."
    EchoYellow "[$(basename $0)] DELETING ALL LINES STARTING FROM LINE NUMBER ${START_LINE}."

    # sed -i "${START_LINE},${END_LINE}d" PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_run.sh
    sed -i "${START_LINE},\$d" PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_run.sh

if [ "$1x" == "airsimx" ]; then
# Runs in the Gazebo container (AIRSIM_IP is set there, not here: escaped so it expands at run
# time). GazeboDrone connects to AirSim once and never retries, like the ROS bridge - wait for
# AirSim's RPC port itself, not the AirSim log (it can still hold the previous run's
# "SimpleFlight"; see scripts/ros2/airsim-bridge.sh). Stuck, the AirSim vehicle would stay at
# the spawn point while the lidar keeps publishing from there.
# GazeboDrone has also died right after connecting (AirSim not fully up behind its open port,
# x8_sp5_b): keep restarting it, and log it next to gazebo.log.
cat << EOF >> PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_run.sh
until timeout 2 bash -c "exec 3<>/dev/tcp/\${AIRSIM_IP}/41451" 2>/dev/null; do
    sleep 0.5
done

(while true; do
    /home/user/workspace/gazebo/GazeboDrone \${AIRSIM_IP}
    echo "[sitl_run.sh] GazeboDrone exited (\$?) - restarting in 2 s"
    sleep 2
done) >> /home/user/workspace/gazebo/logs/gazebodrone.log 2>&1 &

EOF
fi

    echo "# MODIFIED NOT TO RUN GAZEBO ON SITL." >> PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_run.sh
    echo "# DELETED ALL LINES STARTING FROM ${START_LINE}." >> PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_run.sh
fi

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<