#! /bin/bash

# ROS2 APP: T1000 (JBNU) command flown by PathFollowing + flight_driver - the in-house CA chain with
# T1000 in place of the in-house policy (to compare the two policies on the same follower, and to
# separate T1000's smooth attitude from PX4's velocity loop it normally flies through).
#   px4_lidar_to_odom_cloud (/odom, /cloud)
#   -> t1000_bridge.py (/t1000/pose, /t1000/velocity, /t1000/lidar_bins; virtual heading = the path)
#   -> T1000 in --dry-run (no PX4 output: /t1000/cmd_preview only; no arming, no landing, no mode changes -
#      checked in px4_control_node.py)
#   -> t1000_pf_node.py (T1000's path from where the driver's goal arrives to the goal; its command -> PF waypoint +
#      speed, the relay's brake-law safety filter)
#   -> pathfollowing (guid 2 MPPI as the in-house X8 profile) -> flight_driver (mission ca)
# T1000 settings as t1000.sh (corner-mode rounded, authority always, vx/vy caps 3 / 1.5, +-60 deg
# forward sector); per airframe as t1000.sh / noclimb_ca.sh. Knobs: T1000_* as t1000.sh,
# NOCLIMB_GUID / NOCLIMB_HOVER_THRUST / NOCLIMB_MAX_TILT_DEG / NOCLIMB_MAX_TILT_RATE / NOCLIMB_EKF_SETTLE /
# NOCLIMB_SPEED_LIMIT as noclimb_ca.sh, T1000_PF_LOOKAHEAD (default 3 m), T1000_PF_PATH_EXT, T1000_CLOUD_WORLD=1
# (simulator: bin the adapter's /cloud as on the vehicle). Log: logs/t1000_pf.csv.
# On the vehicle (scripts/ros2/onboard-real.sh, NOCLIMB_LIDAR=livox; scripts/dev/onboard_real.sh fly-t1000): the
# Mid-360 through the lidar adapter with the measured mount (NOCLIMB_LIDAR_* as noclimb_ca.sh), T1000 binning the
# adapter's /cloud (registered, own returns removed) with heights above the take-off point; the goal NOCLIMB_GOAL
# "east,north" from the take-off point or NOCLIMB_GOAL_GPS "lat,lon", NOCLIMB_ALT the path height, the operator's
# go (NOCLIMB_REQUIRE_GO=1), NOCLIMB_EKF_SETTLE.

if [ -z "${WORKSPACE_DIR}" ]; then
    WORKSPACE_DIR=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
fi

ROS2_WS=${WORKSPACE_DIR}/ros2_ws
LOGS=${WORKSPACE_DIR}/logs
APP_DIR=$(dirname $(readlink -f "$0"))
T1000_DIR=${ROS2_WS}/src/T1000_Control_445
GOALS_CSV=${ROS2_WS}/src/realgazebo_fastplanner_adapters/config/goals_super_far.csv

if [ ! -f "${T1000_DIR}/models/policy_445.onnx" ]; then
    echo "[$(basename "$0")] ERROR: ${T1000_DIR}/models/policy_445.onnx not found."
    exit 1
fi
for pkg in pathfollowing flight_driver realgazebo_fastplanner_adapters custom_msgs px4_msgs; do
    if [ ! -d "${ROS2_WS}/install/${pkg}" ]; then
        echo "[$(basename "$0")] ERROR: ${pkg} is not built."
        echo "[$(basename "$0")] Build first: ./scripts/run.sh ros2 build ros2_ws"
        exit 1
    fi
done

source ${ROS2_WS}/install/setup.bash
export PYTHONPATH=${T1000_DIR}:${PYTHONPATH}
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 NUMEXPR_NUM_THREADS=2

deg() { python3 -c "import math; print(f'{math.radians($1):.6f}')"; }
LIVOX=$([ "${NOCLIMB_LIDAR:-sim}" = livox ] && echo 1 || echo 0)
if [ ${LIVOX} = 1 ]; then
    LIDAR_TOPIC=/livox/lidar
    ADAPTER_SRC="-p cloud_latency_s:=$(printf '%.3f' ${NOCLIMB_LIDAR_LATENCY:-0.02}) -p deskew_mode:=field \
        -p self_radius_m:=$(printf '%.2f' ${NOCLIMB_SELF_RADIUS:-0.9}) -p voxel_m:=$(printf '%.3f' ${NOCLIMB_VOXEL:-0.1})"
    MOUNT="-p lidar_offset_x:=$(printf '%.3f' ${NOCLIMB_LIDAR_X:-0}) -p lidar_offset_y:=$(printf '%.3f' ${NOCLIMB_LIDAR_Y:-0}) \
        -p lidar_offset_z:=$(printf '%.3f' ${NOCLIMB_LIDAR_Z:-0.3}) -p lidar_roll:=$(deg ${NOCLIMB_LIDAR_ROLL_DEG:-0}) \
        -p lidar_pitch:=$(deg ${NOCLIMB_LIDAR_PITCH_DEG:-0}) -p lidar_yaw:=$(deg ${NOCLIMB_LIDAR_YAW_DEG:-180})"
    BRIDGE_SRC="-p cloud_world:=true -p z_from_start:=true"
else
    LIDAR_TOPIC=/airsim_node/SimpleFlight/lidar/points/RPLIDAR_A3
    ADAPTER_SRC="-p cloud_latency_s:=0.13 -p airsim_odom_topic:=/airsim_node/SimpleFlight/odom_local \
        -p deskew_mode:=$([ "${SIM_LIDAR:-grid}" = mid360 ] && echo index || echo azimuth)"
    MOUNT=""
    BRIDGE_SRC="-p cloud_world:=$([ "${T1000_CLOUD_WORLD:-0}" = 1 ] && echo true || echo false)"
fi
echo "[$(basename "$0")] waiting for lidar data on ${LIDAR_TOPIC} ..."
T_LIDAR=$(date +%s)
until ros2 topic list 2>/dev/null | grep -qx "${LIDAR_TOPIC}" \
        && timeout 10 ros2 topic echo --once --qos-profile sensor_data --field header.stamp ${LIDAR_TOPIC} > /dev/null 2>&1; do
    if [ $(( $(date +%s) - T_LIDAR )) -gt 300 ]; then
        echo "[$(basename "$0")] ERROR: no lidar data within 300 s (simulator / lidar driver not running?) - not taking off."
        exit 1
    fi
    sleep 1
done

if [ "${SITL_AIRFRAME}" = "x8" ]; then
    RANGE_INFLATE=${T1000_RANGE_INFLATE:-0.8}; SAFETY_R=${T1000_SAFETY_RADIUS_M:-1.2}; SAFETY_K=${T1000_SAFETY_GAIN:-2.0}
    PF_VEHICLE_TYPE=2; MPPI_YAML=sim_ca_mppi_x8.yaml; PATH_EXT=${T1000_PF_PATH_EXT:-5}; LIDAR_Z=0.3
elif [ "${SITL_AIRFRAME}" = "x500" ]; then
    RANGE_INFLATE=${T1000_RANGE_INFLATE:-0}; SAFETY_R=${T1000_SAFETY_RADIUS_M:-0.5}; SAFETY_K=${T1000_SAFETY_GAIN:-2.0}
    PF_VEHICLE_TYPE=1; MPPI_YAML=sim_ca_mppi.yaml; PATH_EXT=${T1000_PF_PATH_EXT:-2}; LIDAR_Z=0.1
else
    RANGE_INFLATE=${T1000_RANGE_INFLATE:-0}; SAFETY_R=${T1000_SAFETY_RADIUS_M:-0}; SAFETY_K=${T1000_SAFETY_GAIN:-2.0}
    PF_VEHICLE_TYPE=1; MPPI_YAML=sim_ca_mppi.yaml; PATH_EXT=${T1000_PF_PATH_EXT:-2}; LIDAR_Z=0.3
fi

ROW=$(sed -n 2p ${GOALS_CSV} | tr -d ' ')
GOAL=${NOCLIMB_GOAL:-${T1000_GOAL:-$(echo ${ROW} | cut -d, -f1,2)}}
GOAL_X=$(printf '%.3f' $(echo ${GOAL} | cut -d, -f1)); GOAL_Y=$(printf '%.3f' $(echo ${GOAL} | cut -d, -f2))
ALT=$(printf '%.3f' ${NOCLIMB_ALT:-${T1000_FLIGHT_ALTITUDE_M:-$(echo ${ROW} | cut -d, -f3)}})
echo "[$(basename "$0")] airframe ${SITL_AIRFRAME:-iris}: goal ${NOCLIMB_GOAL_GPS:-${GOAL_X},${GOAL_Y}} at ${ALT} m, lidar inflate" \
     "${RANGE_INFLATE} m, safety filter ${SAFETY_R} m, PF vehicle_type ${PF_VEHICLE_TYPE} guid ${NOCLIMB_GUID:-2} ${MPPI_YAML}," \
     "path extension ${PATH_EXT}, lidar $([ ${LIVOX} = 1 ] && echo 'Mid-360 (adapter /cloud)' || echo simulator)"

# 1) PX4 pose -> /odom; lidar -> /cloud (registered: T1000's bins on the vehicle, the safety filter, the recorder)
ros2 run realgazebo_fastplanner_adapters px4_lidar_to_odom_cloud --ros-args \
    -p "px4_ns:=''" -p in_topic:=${LIDAR_TOPIC} -p use_ros_time:=true ${ADAPTER_SRC} \
    -p synth_free_rays:=false -p deskew_rot_period_s:=0.1 \
    -p max_tilt_rate:=$(printf '%.3f' ${NOCLIMB_MAX_TILT_RATE:-0}) -p publish_empty_scans:=true \
    ${MOUNT:--p lidar_offset_x:=0.0 -p lidar_offset_y:=0.0 -p lidar_offset_z:=${LIDAR_Z} -p lidar_roll:=0.0 -p lidar_pitch:=0.0 -p lidar_yaw:=0.0} \
    2>&1 | tee ${LOGS}/px4_lidar_to_odom_cloud.log &

# 2) T1000's simulator inputs (bins with the path's yaw: PF turns the real nose)
python3 ${APP_DIR}/t1000_bridge.py --ros-args -p lidar_topic:=${LIDAR_TOPIC} ${BRIDGE_SRC} \
    -p range_inflate_m:=$(printf '%.3f' ${RANGE_INFLATE}) -p virtual_heading:=path \
    2>&1 | tee ${LOGS}/t1000_bridge.log &

# 3) PathFollowing, as the in-house X8 profile
PF_VEHICLE_ARGS="-p hover_thrust:=$(printf '%.4f' ${NOCLIMB_HOVER_THRUST:-0}) -p max_tilt_deg:=$(printf '%.1f' ${NOCLIMB_MAX_TILT_DEG:-0})"
if [ "${NOCLIMB_GUID:-2}" = "2" ]; then
    PF_ARGS="-p guid_type:=2 -p sim_yaml:=${MPPI_YAML}"
    ros2 run pathfollowing node_mppi --ros-args -p guid_type:=2 -p sim_yaml:=${MPPI_YAML} \
        -p vehicle_type:=${PF_VEHICLE_TYPE} ${PF_VEHICLE_ARGS} 2>&1 | tee ${LOGS}/node_mppi.log &
else
    PF_ARGS="-p guid_type:=0 -p vertical_priority:=true"
fi
ros2 run pathfollowing node_pathfollowing --ros-args ${PF_ARGS} -p vehicle_type:=${PF_VEHICLE_TYPE} \
    ${PF_VEHICLE_ARGS} 2>&1 | tee ${LOGS}/node_pathfollowing.log &

# 4) T1000's path + command -> PF
python3 ${APP_DIR}/t1000_pf_node.py --ros-args \
    -p lookahead_m:=$(printf '%.2f' ${T1000_PF_LOOKAHEAD:-3}) -p path_extension:=$(printf '%.2f' ${PATH_EXT}) \
    -p safety_radius_m:=$(printf '%.3f' ${SAFETY_R}) -p safety_gain:=$(printf '%.3f' ${SAFETY_K}) \
    -p goal_handover_m:=$(printf '%.3f' ${SUPER_PF_GOAL_HANDOVER_M:-3.0}) \
    -p log_csv:=${LOGS}/t1000_pf.csv 2>&1 | tee ${LOGS}/t1000_pf_node.log &

# 5) T1000 in dry-run: its full command on /t1000/cmd_preview, nothing to PX4 (--isaac-sim: the bridge's
#    simulator topics as inputs; without --sim-auto-start / --confirm-sitl it has no lifecycle)
sleep 3
export T1000_SETTINGS_JSON=${T1000_SETTINGS_JSON:-'{"AVOID_FORWARD_HALF_ANGLE_DEG": 60}'}
T1000_FLIGHT_ALTITUDE_M=${ALT} python3 ${APP_DIR}/t1000_run.py \
    --isaac-sim --dry-run --allow-no-rc \
    --vx-cap ${T1000_VX_CAP:-3} --vy-cap ${T1000_VY_CAP:-1.5} \
    --corner-mode rounded --policy-authority ${T1000_POLICY_AUTHORITY:-always} --ordered-waypoints \
    2>&1 | tee ${LOGS}/t1000_control.log &

# 6) Flight FSM: arm (on go, on the vehicle), AUTO_TAKEOFF, OFFBOARD, forward PF's attitude, land at the goal
GPS_ARGS=""
[ -n "${NOCLIMB_GOAL_GPS:-}" ] && GPS_ARGS="-p goal_lat:=$(echo ${NOCLIMB_GOAL_GPS} | cut -d, -f1) -p goal_lon:=$(echo ${NOCLIMB_GOAL_GPS} | cut -d, -f2)"
ros2 run flight_driver flight_driver --ros-args -p mission:=ca \
    -p takeoff_altitude:=${ALT} -p goal_north_m:=${GOAL_Y} -p goal_east_m:=${GOAL_X} ${GPS_ARGS} \
    -p goal_handover_radius_m:=$(printf '%.3f' ${SUPER_PF_GOAL_HANDOVER_M:-3.0}) \
    -p require_go:=$([ "${NOCLIMB_REQUIRE_GO:-0}" = 1 ] && echo true || echo false) \
    -p ekf_settle_s:=$(printf '%.1f' ${NOCLIMB_EKF_SETTLE:-$([ ${LIVOX} = 1 ] && echo 30 || echo 5)}) \
    -p speed_limit_mps:=$(printf '%.2f' ${NOCLIMB_SPEED_LIMIT:-$(python3 -c "print(${T1000_VX_CAP:-3} + 2.5)")}) \
    -p log_dir:=${LOGS} 2>&1 | tee ${LOGS}/flight_driver.log &

wait
