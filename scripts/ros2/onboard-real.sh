#! /bin/bash

# ON THE VEHICLE (the onboard computer's container, started by scripts/dev/onboard_real.sh): the Mid-360
# driver and foxglove_bridge, then scripts/ros2/apps/${ROS2_APP}.sh (noclimb_ca with NOCLIMB_LIDAR=livox, mission).
# The uXRCE-DDS agent is NOT started here: it stays up in its own container across runs (PX4 1.16 does
# not reconnect to a restarted agent - the flight controller would need a reboot), so this only waits
# for its /fmu topics. Mid-360 settings: scripts/ros2/livox/MID360_a4vai.json (lidar 192.168.1.100,
# this computer 192.168.1.50 on the lidar's port).

export TERM=xterm-256color
BASE_DIR=$(dirname $(readlink -f "$0"))
export WORKSPACE_DIR=$(dirname ${BASE_DIR})
LOGS=${WORKSPACE_DIR}/logs
mkdir -p ${LOGS}
# this run's logs only (the last run's were collected into test_runs/ by onboard_real.sh stop; files are root's)
rm -f ${LOGS}/*.log ${LOGS}/noclimb_ca.csv ${LOGS}/noclimb_ca_dump.npz ${LOGS}/t1000_pf.csv
source /opt/ros/humble/setup.bash
source ${WORKSPACE_DIR}/livox_ws/install/setup.bash
source ${WORKSPACE_DIR}/ros2_ws/install/setup.bash

# 1) Mid-360 -> /livox/lidar: PointCloud2 (x y z intensity tag line timestamp - each point's own time, ns),
#    10 Hz frames in the sensor frame (ONBOARD_NO_LIDAR=1: none, e.g. a waypoint mission without the lidar)
#    LIVOX_LIDAR_IP: the lidar's own address (X500 192.168.1.100, the X8's Mid-360 192.168.1.182 - each unit has its own)
LIVOX_CFG=${BASE_DIR}/livox/MID360_a4vai.json
if [ -n "${LIVOX_LIDAR_IP:-}" ] && [ "${LIVOX_LIDAR_IP}" != 192.168.1.100 ]; then
    LIVOX_CFG=${LOGS}/MID360_run.json
    sed "s/\"ip\" : \"192.168.1.100\"/\"ip\" : \"${LIVOX_LIDAR_IP}\"/" ${BASE_DIR}/livox/MID360_a4vai.json > ${LIVOX_CFG}
fi
echo "[$(basename "$0")] Mid-360 at $(grep -o '"ip" : "[0-9.]*"' ${LIVOX_CFG} | cut -d'"' -f4)"
[ "${ONBOARD_NO_LIDAR:-0}" != 1 ] && ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
    -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 -p output_data_type:=0 \
    -p frame_id:=livox_frame -p user_config_path:=${LIVOX_CFG} \
    -p "cmdline_input_bd_code:=livox0000000001" 2>&1 | tee ${LOGS}/livox.log &

# 2) foxglove_bridge: the ground station watches /cloud, /noclimb/path, /odom at ws://<this computer>:8765
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 > ${LOGS}/foxglove_bridge.log 2>&1 &

# 3) PX4 through the resident agent
echo "[$(basename "$0")] waiting for /fmu topics (the agent: scripts/dev/onboard_real.sh agent start)"
until ros2 topic list 2>/dev/null | grep -q '^/fmu/'; do sleep 1; done
echo "[$(basename "$0")] /fmu topics up - starting ${ROS2_APP}"

# 4) the app on the real lidar
export NOCLIMB_LIDAR=livox
bash ${BASE_DIR}/apps/${ROS2_APP}.sh 2>&1 | tee ${LOGS}/${ROS2_APP}.log
