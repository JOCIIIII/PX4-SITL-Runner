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

# numpy/BLAS 가 코어 전체에 스레드를 깔면 MPPI 등 작은 행렬 연산이 스래싱해
# CPU 를 통째로 먹고 Gazebo RTF 가 떨어진다. 노드당 2 스레드로 캡.
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 NUMEXPR_NUM_THREADS=2

# 1) AirSim lidar + PX4 pose -> /odom + /cloud (settings.json: lidar at Z=-0.3 NED, no tilt)
ros2 run realgazebo_fastplanner_adapters px4_lidar_to_odom_cloud --ros-args \
    -p "px4_ns:=''" \
    -p in_topic:=/airsim_node/SimpleFlight/lidar/points/RPLIDAR_A3 \
    -p use_ros_time:=true \
    -p synth_free_rays:=true \
    -p lidar_offset_x:=0.0 -p lidar_offset_y:=0.0 -p lidar_offset_z:=0.3 \
    -p lidar_roll:=0.0 -p lidar_pitch:=0.0 -p lidar_yaw:=0.0 \
    2>&1 | tee ${LOGS}/px4_lidar_to_odom_cloud.log &

# 2) SUPER planner (config from SOURCE tree: super/super_planner/config/static_realgazebo.yaml)
ros2 run super_planner fsm_node --ros-args \
    -p config_name:=static_realgazebo.yaml -p live_mode:=true \
    2>&1 | tee ${LOGS}/super_fsm.log &

# 3) SUPER poly traj -> PF waypoints (also publishes pp/ca heartbeats)
ros2 run super_pf_bridge poly_to_waypoints 2>&1 | tee ${LOGS}/super_pf_bridge.log &

# 4) PathFollowing (sim.yaml: wp_type 0, guid_type 2 -> needs MPPI)
ros2 run pathfollowing node_pathfollowing 2>&1 | tee ${LOGS}/node_pathfollowing.log &
ros2 run pathfollowing node_mppi 2>&1 | tee ${LOGS}/node_mppi.log &

# 5) Flight FSM. flight_manager hardcodes vehicle1/fmu/* - remap to this SITL's /fmu/*.
ros2 run flight_manager flight_manager --ros-args \
    -r /vehicle1/fmu/in/offboard_control_mode:=/fmu/in/offboard_control_mode \
    -r /vehicle1/fmu/in/trajectory_setpoint:=/fmu/in/trajectory_setpoint \
    -r /vehicle1/fmu/in/vehicle_attitude_setpoint:=/fmu/in/vehicle_attitude_setpoint \
    -r /vehicle1/fmu/in/vehicle_command:=/fmu/in/vehicle_command \
    -r /vehicle1/fmu/out/vehicle_status_v1:=/fmu/out/vehicle_status_v1 \
    -r /vehicle1/fmu/out/vehicle_local_position:=/fmu/out/vehicle_local_position \
    -r /vehicle1/fmu/out/vehicle_attitude:=/fmu/out/vehicle_attitude \
    2>&1 | tee ${LOGS}/flight_manager.log &

# 6) Goal feeder: wait for takeoff, then stream goals_super.csv to SUPER + bridge
(
    sleep 20
    ros2 run realgazebo_fastplanner_adapters csv_goal_pub --ros-args \
        -p csv_path:=${ROS2_WS}/src/realgazebo_fastplanner_adapters/config/goals_super_far.csv \
        -p goal_pose_topic:=/planning/click_goal \
        -p sequential:=true -p reach_radius:=2.0 \
        2>&1 | tee ${LOGS}/csv_goal_pub.log
) &
