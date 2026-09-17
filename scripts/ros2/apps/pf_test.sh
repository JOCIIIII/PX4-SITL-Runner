#! /bin/bash

# ROS2 APP: PathFollowing ONLY test (no SUPER / no planner).
# path_following_test publishes a fixed waypoint list from wp.csv (package share)
# and runs the arm/offboard/takeoff -> follow -> land FSM itself.
#   waypoints override: edit src/path_following_test/wp.csv and rebuild, or
#   set wp_csv_path below to an absolute path (no rebuild needed).
# Build first: ./scripts/run.sh ros2 build ros2_ws

# WORKSPACE_DIR is exported by the launcher; fall back if run standalone.
if [ -z "${WORKSPACE_DIR}" ]; then
    WORKSPACE_DIR=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
fi

ROS2_WS=${WORKSPACE_DIR}/ros2_ws
LOGS=${WORKSPACE_DIR}/logs

for pkg in pathfollowing path_following_test px4_msgs; do
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

# PathFollowing (sim.yaml: wp_type 0, guid_type 2 -> needs MPPI)
ros2 run pathfollowing node_pathfollowing 2>&1 | tee ${LOGS}/node_pathfollowing.log &
ros2 run pathfollowing node_mppi 2>&1 | tee ${LOGS}/node_mppi.log &

# Flight FSM + CSV waypoint publisher. Hardcodes vehicle1/fmu/* - remap to /fmu/*.
# wp_csv_path points at the SOURCE csv so editing it needs no rebuild.
ros2 run path_following_test path_following_test --ros-args \
    -p wp_csv_path:=${ROS2_WS}/src/path_following_test/wp.csv \
    -r /vehicle1/fmu/in/offboard_control_mode:=/fmu/in/offboard_control_mode \
    -r /vehicle1/fmu/in/trajectory_setpoint:=/fmu/in/trajectory_setpoint \
    -r /vehicle1/fmu/in/vehicle_attitude_setpoint:=/fmu/in/vehicle_attitude_setpoint \
    -r /vehicle1/fmu/in/vehicle_command:=/fmu/in/vehicle_command \
    -r /vehicle1/fmu/out/vehicle_status_v1:=/fmu/out/vehicle_status_v1 \
    -r /vehicle1/fmu/out/vehicle_local_position:=/fmu/out/vehicle_local_position \
    -r /vehicle1/fmu/out/vehicle_attitude:=/fmu/out/vehicle_attitude \
    2>&1 | tee ${LOGS}/path_following_test.log &
