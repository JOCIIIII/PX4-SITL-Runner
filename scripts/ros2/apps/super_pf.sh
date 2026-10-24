#! /bin/bash

# ROS2 APP: SUPER planner -> PathFollowing pipeline on AirSim lidar.
#   px4_lidar_to_odom_cloud (AirSim lidar+PX4 pose -> /odom + /cloud, world ENU)
#   -> super_planner fsm_node (rog_map + trajectory, /planning_cmd/poly_traj)
#   -> super_pf_bridge poly_to_waypoints (-> /local_waypoint_setpoint_to_PF, NED)
#   -> pathfollowing node_pathfollowing + node_mppi (-> /pf_att_2_control)
#   -> flight_manager (arm/offboard FSM, forwards to /fmu/in/*)
#   goals come from csv_goal_pub (goals_super.csv -> /planning/click_goal)
# Build first: ./scripts/run.sh ros2 build ros2_ws

# WORKSPACE_DIR is exported by the launcher; fall back if run standalone.
if [ -z "${WORKSPACE_DIR}" ]; then
    WORKSPACE_DIR=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
fi

ROS2_WS=${WORKSPACE_DIR}/ros2_ws
LOGS=${WORKSPACE_DIR}/logs

for pkg in super_planner super_pf_bridge pathfollowing flight_manager realgazebo_fastplanner_adapters px4_msgs; do
    if [ ! -d "${ROS2_WS}/install/${pkg}" ]; then
        echo "[$(basename "$0")] ERROR: ${pkg} is not built."
        echo "[$(basename "$0")] Build first: ./scripts/run.sh ros2 build ros2_ws"
        exit 1
    fi
done

source ${ROS2_WS}/install/setup.bash

# Do not fly blind: if AirSim never came up (container killed, bridge not connected) there
# is no lidar and SUPER would plan through an empty map. Wait for real scans first; without
# them exit before flight_manager can arm.
LIDAR_TOPIC=/airsim_node/SimpleFlight/lidar/points/RPLIDAR_A3
echo "[$(basename "$0")] waiting for lidar data on ${LIDAR_TOPIC} ..."
if ! timeout 300 ros2 topic echo --once --qos-profile sensor_data --field header.stamp \
        ${LIDAR_TOPIC} > /dev/null 2>&1; then
    echo "[$(basename "$0")] ERROR: no lidar data within 300 s (AirSim not running?) - not taking off."
    exit 1
fi

# numpy/BLAS 가 코어 전체에 스레드를 깔면 MPPI 등 작은 행렬 연산이 스래싱해
# CPU 를 통째로 먹고 Gazebo RTF 가 떨어진다. 노드당 2 스레드로 캡.
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 NUMEXPR_NUM_THREADS=2

# 1) AirSim lidar + PX4 pose -> /odom + /cloud (settings.json: lidar at Z=-0.3 NED, no tilt).
#    Scans shot while tilting > 0.8 rad/s must not reach SUPER: mis-registered, they put phantom
#    obstacles into its map in hard brakes (ctrack_123_j). With the speed guard on they stay on
#    /cloud for the guard's tracking (dropped there they left gaps that restarted the RC cart's
#    track: ctrack_s900_c) and only the others go to /cloud_gated -> the guard -> SUPER; without
#    the guard the adapter drops them from /cloud.
ADAPTER_GATED=$([ "${SUPER_PF_SPEED_GUARD}" = "1" ] && echo "-p gated_topic:=/cloud_gated")
ros2 run realgazebo_fastplanner_adapters px4_lidar_to_odom_cloud --ros-args \
    -p "px4_ns:=''" \
    -p in_topic:=${LIDAR_TOPIC} \
    -p use_ros_time:=true -p cloud_latency_s:=0.13 \
    -p airsim_odom_topic:=/airsim_node/SimpleFlight/odom_local \
    -p synth_free_rays:=false -p deskew_rot_period_s:=0.1 \
    -p deskew_mode:=$([ "${SIM_LIDAR:-grid}" = mid360 ] && echo index || echo azimuth) -p max_tilt_rate:=0.8 ${ADAPTER_GATED} \
    -p lidar_offset_x:=0.0 -p lidar_offset_y:=0.0 -p lidar_offset_z:=0.3 \
    -p lidar_roll:=0.0 -p lidar_pitch:=0.0 -p lidar_yaw:=0.0 \
    2>&1 | tee ${LOGS}/px4_lidar_to_odom_cloud.log &

# 2) SUPER planner (config from SOURCE tree: super/super_planner/config/static_realgazebo.yaml,
#    static_realgazebo_x8.yaml for the X8 - its thrust / body-rate limits and mass / drag)
SUPER_CONFIG=$([ "${SITL_AIRFRAME}" = "x8" ] && echo static_realgazebo_x8.yaml || echo static_realgazebo.yaml)
#    SUPER plans at SUPER_PF_MAX_SPEED: a copy of the config with that max_vel goes to the logs.
#    SUPER opens <its source dir>/config/<config_name>, so the name is relative to that.
#    SUPER_PF_VIRTUAL_GROUND / SUPER_PF_VIRTUAL_CEIL (optional) replace the virtual ground /
#    ceiling in that copy: the configs keep the trajectory in a band for the 5 m forest flights
#    (x8: about 4-8.8 m), so a low flight (C-Track at 1.5 m) needs its own band.
sed -E "s/^(\s*max_vel:\s*)[0-9.]+/\1${SUPER_PF_MAX_SPEED:-7.0}/" \
    ${ROS2_WS}/src/super/super_planner/config/${SUPER_CONFIG} > ${LOGS}/super_config.yaml
if [ -n "${SUPER_PF_VIRTUAL_GROUND}" ]; then
    sed -i -E "s/^(\s*virtual_ground_height:\s*)[-0-9.]+/\1${SUPER_PF_VIRTUAL_GROUND}/" ${LOGS}/super_config.yaml
fi
if [ -n "${SUPER_PF_VIRTUAL_CEIL}" ]; then
    sed -i -E "s/^(\s*virtual_ceil_height:\s*)[-0-9.]+/\1${SUPER_PF_VIRTUAL_CEIL}/" ${LOGS}/super_config.yaml
fi
#    SUPER_PF_INFLATION_STEP (optional): rog_map inflation_step in that copy (see ctrack_super.sh)
if [ -n "${SUPER_PF_INFLATION_STEP}" ]; then
    sed -i -E "s/^(\s*inflation_step:\s*)[0-9]+/\1${SUPER_PF_INFLATION_STEP}/" ${LOGS}/super_config.yaml
fi
#    SUPER_PF_CORRIDOR_Z_MARGIN (optional): super_planner corridor_z_margin (the corridor's centre
#    stays that far inside the virtual ground / ceiling; see super_core/config.hpp)
if [ -n "${SUPER_PF_CORRIDOR_Z_MARGIN}" ]; then
    sed -i -E "s/^(\s*)(robot_r:.*)$/\1\2\n\1corridor_z_margin: ${SUPER_PF_CORRIDOR_Z_MARGIN}/" ${LOGS}/super_config.yaml
fi
#    SUPER_PF_SPEED_GUARD=1: SUPER maps /cloud_static (super_speed_guard.py drops moving objects)
if [ "${SUPER_PF_SPEED_GUARD}" = "1" ]; then
    sed -i -E 's#^(\s*cloud_topic:\s*)"/cloud"#\1"/cloud_static"#' ${LOGS}/super_config.yaml
fi
grep -E "^\s*(max_vel|virtual_ground_height|virtual_ceil_height|inflation_step|robot_r|corridor_z_margin|cloud_topic):" ${LOGS}/super_config.yaml
ros2 run super_planner fsm_node --ros-args \
    -p config_name:=../../../../../logs/super_config.yaml -p live_mode:=true \
    2>&1 | tee ${LOGS}/super_fsm.log &

# 3) SUPER poly traj -> PF waypoints (also publishes pp/ca heartbeats)
#    SUPER_PF_SPEED_GUARD=1: the bridge's /desired_speed goes through super_speed_guard.py, which
#    brakes for a lidar return that blocks the path ahead - SUPER's static map does not stop for
#    an obstacle moving into its committed trajectory (ctrack_123_d: RC cart). Prop-tip radius:
#    SUPER_PF_GUARD_RADIUS (x8 0.8 m; iris 0.4 m, untested).
SPEED_TOPIC_ARG=$([ "${SUPER_PF_SPEED_GUARD}" = "1" ] && echo "-r /desired_speed:=/desired_speed_raw")
#    SUPER_PF_HOLD_Z (optional): every waypoint to PF at this PX4-local height (ENU z) - the
#    flight height stays fixed while SUPER dodges sideways (C-Track 1.5 m; poly_to_waypoints hold_z)
HOLD_ARG=$([ -n "${SUPER_PF_HOLD_Z}" ] && printf -- '-p hold_z:=%.3f' "${SUPER_PF_HOLD_Z}")
ros2 run super_pf_bridge poly_to_waypoints --ros-args -p speed_max:=${SUPER_PF_MAX_SPEED:-7.0} ${SPEED_TOPIC_ARG} ${HOLD_ARG} \
    2>&1 | tee ${LOGS}/super_pf_bridge.log &
if [ "${SUPER_PF_SPEED_GUARD}" = "1" ]; then
    GUARD_R=${SUPER_PF_GUARD_RADIUS:-$([ "${SITL_AIRFRAME}" = "x8" ] && echo 0.8 || echo 0.4)}
    #    SUPER_PF_STATIC_MIN_Z (optional): nothing below this PX4-local height reaches SUPER (C-Track at
    #    a held 1.5 m: 1.0 m above ground - mis-registered ground lands lower, every obstacle is taller)
    STATIC_MIN_Z_ARG=$([ -n "${SUPER_PF_STATIC_MIN_Z}" ] && printf -- '-p static_cloud_min_z:=%.3f' "${SUPER_PF_STATIC_MIN_Z}")
    rm -f ${LOGS}/super_speed_guard_scans.pkl      # its inputs, for scripts/dev/speed_guard_sim.py
    #    It also feeds SUPER /cloud without moving objects (/cloud_static, see the sed above), so
    #    it is restarted if it ever dies: without it SUPER gets no scans and PF keeps the last speed.
    (
        while true; do
            python3 $(dirname $(readlink -f "$0"))/super_speed_guard.py --ros-args \
                -p block_radius_m:=${GUARD_R} -p nominal_speed:=${SUPER_PF_MAX_SPEED:-7.0} \
                -p static_cloud_topic:=/cloud_static -p static_cloud_input:=/cloud_gated ${STATIC_MIN_Z_ARG} \
                -p dump_path:=${LOGS}/super_speed_guard_scans.pkl
            echo "[super_pf] super_speed_guard exited - restarting"
            sleep 0.5
        done
    ) 2>&1 | tee ${LOGS}/super_speed_guard.log &
fi

# 4) PathFollowing, guid 0 (speed-regulated pursuit, sim.yaml pos_speed_mode). The bridge
#    drives the speed via /desired_speed. MPPI (guid 1/2) never engaged on this
#    continuously replanned path anyway (passed_wp_idx stays < 1 -> forced guid 0).
# PathFollowing vehicle model follows the simulated airframe (envs/gazebo-classic.env
# SITL_AIRFRAME, copied into ros2.env by run.sh): x8 -> octo.yaml (2), iris -> quad.yaml (1)
PF_VEHICLE_TYPE=$([ "${SITL_AIRFRAME}" = "x8" ] && echo 2 || echo 1)
ros2 run pathfollowing node_pathfollowing --ros-args -p guid_type:=0 -p vehicle_type:=${PF_VEHICLE_TYPE} \
    2>&1 | tee ${LOGS}/node_pathfollowing.log &
ros2 run pathfollowing node_mppi --ros-args -p guid_type:=0 -p vehicle_type:=${PF_VEHICLE_TYPE} \
    2>&1 | tee ${LOGS}/node_mppi.log &

# 5) Flight FSM. flight_manager hardcodes vehicle1/fmu/* - remap to this SITL's /fmu/*.
#    Near the final goal (last line of the goal CSV) it hands over from PF to PX4 position
#    control, stops at the goal and lands there (SUPER_PF_GOAL_HANDOVER_M, envs/ros2.env).
GOAL_CSV=${ROS2_WS}/src/realgazebo_fastplanner_adapters/config/goals_super_far.csv
GOAL_ENU=$(awk -F, '$1 ~ /^ *-?[0-9.]+ *$/ {g = sprintf("%.3f, %.3f, %.3f", $1, $2, $3)} END {print g}' ${GOAL_CSV})
#    SUPER_PF_ALT (optional): take-off height [m] (flight_manager's default is 5).
FM_ALT_ARG=$([ -n "${SUPER_PF_ALT}" ] && printf -- '-p takeoff_altitude:=%.3f' "${SUPER_PF_ALT}")
ros2 run flight_manager flight_manager --ros-args ${FM_ALT_ARG} \
    -p "goal_enu:=[${GOAL_ENU}]" -p goal_handover_radius_m:=${SUPER_PF_GOAL_HANDOVER_M:-3.0} \
    -r /vehicle1/fmu/in/offboard_control_mode:=/fmu/in/offboard_control_mode \
    -r /vehicle1/fmu/in/trajectory_setpoint:=/fmu/in/trajectory_setpoint \
    -r /vehicle1/fmu/in/vehicle_attitude_setpoint:=/fmu/in/vehicle_attitude_setpoint \
    -r /vehicle1/fmu/in/vehicle_command:=/fmu/in/vehicle_command \
    -r /vehicle1/fmu/out/vehicle_status_v1:=/fmu/out/vehicle_status_v1 \
    -r /vehicle1/fmu/out/vehicle_local_position:=/fmu/out/vehicle_local_position \
    -r /vehicle1/fmu/out/vehicle_attitude:=/fmu/out/vehicle_attitude \
    -r /vehicle1/fmu/out/sensor_combined:=/fmu/out/sensor_combined \
    2>&1 | tee ${LOGS}/flight_manager.log &

# 6) Goal feeder: wait for takeoff, then stream goals_super.csv to SUPER + bridge
(
    sleep 20
    ros2 run realgazebo_fastplanner_adapters csv_goal_pub --ros-args \
        -p csv_path:=${ROS2_WS}/src/realgazebo_fastplanner_adapters/config/goals_super_far.csv \
        -p goal_pose_topic:=/planning/click_goal \
        -p sequential:=true -p reach_radius:=2.0 -p goal_repub_count:=3 \
        2>&1 | tee ${LOGS}/csv_goal_pub.log
) &
