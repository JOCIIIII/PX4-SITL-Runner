#! /bin/bash

# ROS2 APP (dev): closed-loop response test of the vehicle + PathFollowing in the noclimb CA
# policy's command format (scripts/ros2/tools/x8_step_test.py: speed steps, a 90 deg turn,
# altitude steps, +-40 deg zigzag). Analyse with scripts/dev/x8_step_analysis.py.

if [ -z "${WORKSPACE_DIR}" ]; then
    WORKSPACE_DIR=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
fi
ROS2_WS=${WORKSPACE_DIR}/ros2_ws
LOGS=${WORKSPACE_DIR}/logs
PF_VEHICLE_TYPE=$([ "${SITL_AIRFRAME}" = "x8" ] && echo 2 || echo 1)
MPPI_YAML=$([ "${SITL_AIRFRAME}" = "x8" ] && echo sim_ca_mppi_x8.yaml || echo sim_ca_mppi.yaml)

source ${ROS2_WS}/install/setup.bash
# PathFollowing as in noclimb_ca.sh
# STEP_GUID=2: PF's MPPI guidance with the learned-CA config of this airframe (as noclimb_ca
# NOCLIMB_GUID=2: sim_ca_mppi_x8.yaml for x8, sim_ca_mppi.yaml for iris)
# and the noclimb_ca command shaping of this airframe, so the whole chain the policy drives is
# measured (x8: path extension 5, smoothing NOCLIMB_CMD_TAU with immediate speed decreases;
# x500: extension 2, the same smoothing and immediate decreases; iris: extension 2, smoothing
# NOCLIMB_CMD_TAU). STEP_GUID=0: the raw command, extension 2.
SHAPE_ARGS="-p path_extension:=2.0"
if [ "${STEP_GUID:-0}" = "2" ]; then
    if [ "${SITL_AIRFRAME}" = "x8" ]; then
        SHAPE_ARGS="-p path_extension:=5.0 -p cmd_brake_immediate:=true"
    elif [ "${SITL_AIRFRAME}" = "x500" ]; then
        SHAPE_ARGS="${SHAPE_ARGS} -p cmd_brake_immediate:=true"
    fi
    SHAPE_ARGS="${SHAPE_ARGS} -p cmd_tau_s:=$(printf '%.3f' ${NOCLIMB_CMD_TAU:-0})"
    PF_ARGS="-p guid_type:=2 -p sim_yaml:=${MPPI_YAML}"
    ros2 run pathfollowing node_mppi --ros-args -p guid_type:=2 -p sim_yaml:=${MPPI_YAML} \
        -p vehicle_type:=${PF_VEHICLE_TYPE} 2>&1 | tee ${LOGS}/node_mppi.log &
else
    PF_ARGS="-p guid_type:=0 -p vertical_priority:=true"
fi
ros2 run pathfollowing node_pathfollowing --ros-args ${PF_ARGS} \
    -p vehicle_type:=${PF_VEHICLE_TYPE} 2>&1 | tee ${LOGS}/node_pathfollowing.log &
echo "[$(basename "$0")] airframe ${SITL_AIRFRAME:-iris}, guid ${STEP_GUID:-0}, command shaping: ${SHAPE_ARGS}"
python3 ${WORKSPACE_DIR}/scripts/tools/x8_step_test.py --ros-args -p log_csv:=${LOGS}/x8_step_test.csv ${SHAPE_ARGS} \
    2>&1 | tee ${LOGS}/x8_step_test_node.log &
sleep 3
ros2 run flight_manager flight_manager --ros-args \
    -p takeoff_altitude:=4.0 \
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
