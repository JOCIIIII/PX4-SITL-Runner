#! /bin/bash

# ROS2 APP (dev): X8 model validation - replays a recorded attitude-setpoint sequence of the
# real vehicle on the SITL X8 (scripts/ros2/tools/x8_replay.py) through flight_manager.
# knobs (env): X8_REPLAY_CSV (default: test_runs/x8_replay_src/rate_tune_test_25_80.csv,
# exported from the vehicle's 2025-07-31 rate tuning flight). Compare with scripts/dev/x8_sysid.py.

if [ -z "${WORKSPACE_DIR}" ]; then
    WORKSPACE_DIR=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
fi
ROS2_WS=${WORKSPACE_DIR}/ros2_ws
LOGS=${WORKSPACE_DIR}/logs
CSV=${X8_REPLAY_CSV:-${WORKSPACE_DIR}/test_runs/x8_replay_src/rate_tune_test_25_80.csv}

source ${ROS2_WS}/install/setup.bash
python3 ${WORKSPACE_DIR}/scripts/tools/x8_replay.py --ros-args -p csv:=${CSV} \
    2>&1 | tee ${LOGS}/x8_replay_node.log &
sleep 3
ros2 run flight_manager flight_manager --ros-args \
    -p takeoff_altitude:=5.0 \
    -r /vehicle1/fmu/in/offboard_control_mode:=/fmu/in/offboard_control_mode \
    -r /vehicle1/fmu/in/trajectory_setpoint:=/fmu/in/trajectory_setpoint \
    -r /vehicle1/fmu/in/vehicle_attitude_setpoint:=/fmu/in/vehicle_attitude_setpoint \
    -r /vehicle1/fmu/in/vehicle_command:=/fmu/in/vehicle_command \
    -r /vehicle1/fmu/out/vehicle_status_v1:=/fmu/out/vehicle_status_v1 \
    -r /vehicle1/fmu/out/vehicle_local_position:=/fmu/out/vehicle_local_position \
    -r /vehicle1/fmu/out/vehicle_attitude:=/fmu/out/vehicle_attitude \
    -r /vehicle1/fmu/out/sensor_combined:=/fmu/out/sensor_combined \
    2>&1 | tee ${LOGS}/flight_manager.log &
wait
