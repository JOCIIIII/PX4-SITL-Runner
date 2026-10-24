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
# PathFollowing vehicle model follows the simulated airframe (envs/gazebo-classic.env
# SITL_AIRFRAME, copied into ros2.env by run.sh): x8 -> octo.yaml (2), iris -> quad.yaml (1)
PF_VEHICLE_TYPE=$([ "${SITL_AIRFRAME}" = "x8" ] && echo 2 || echo 1)
# PF_TEST_SIM_YAML (optional): another PF/MPPI config, e.g. sim_ca_mppi_x8.yaml (the X8 MPPI as flown with
# the CA policy: 2048 samples, correlated noise, pursuit beyond 90 deg heading error, speed P) - sim.yaml's
# MPPI stalled on the first 90 deg corner of a square at 1.5 m/s without wind (x8_mppi_wind7_a)
SIM_YAML_ARG=$([ -n "${PF_TEST_SIM_YAML}" ] && echo "-p sim_yaml:=${PF_TEST_SIM_YAML} -p guid_type:=2")
# PF_TEST_CORNER=1: corner speed limit + MPPI rollout = PF guidance (GuidanceTuningConfig
# corner_speed_enable / mppi_rollout_sync), the same for both nodes. PF_TEST_PF_ARGS: more node
# parameters as name:=value,name:=value (floats with a decimal point), e.g.
# PF_TEST_PF_ARGS=corner_max_error_m:=1.0,corner_lat_accel_mps2:=5.0
PF_EXTRA_ARGS=$([ "${PF_TEST_CORNER:-0}" = "1" ] && echo "-p corner_speed_enable:=true -p mppi_rollout_sync:=true")
for kv in $(echo "${PF_TEST_PF_ARGS:-}" | tr ',' ' '); do
    PF_EXTRA_ARGS="${PF_EXTRA_ARGS} -p ${kv}"
done
ros2 run pathfollowing node_pathfollowing --ros-args -p vehicle_type:=${PF_VEHICLE_TYPE} ${SIM_YAML_ARG} ${PF_EXTRA_ARGS} \
    2>&1 | tee ${LOGS}/node_pathfollowing.log &
ros2 run pathfollowing node_mppi --ros-args -p vehicle_type:=${PF_VEHICLE_TYPE} ${SIM_YAML_ARG} ${PF_EXTRA_ARGS} \
    2>&1 | tee ${LOGS}/node_mppi.log &

# PF_TEST_SPEED [m/s] (optional): MPPI cruise speed instead of sim.yaml's desired_speed (1.5) -
# node_pathfollowing and node_mppi both take /desired_speed; re-sent at 2 Hz for late joiners.
if [ -n "${PF_TEST_SPEED}" ]; then
    ros2 topic pub -r 2 /desired_speed std_msgs/msg/Float32 "{data: $(printf '%.2f' ${PF_TEST_SPEED})}" > /dev/null 2>&1 &
fi

# Flight FSM + CSV waypoint publisher. Hardcodes vehicle1/fmu/* - remap to /fmu/*.
# wp_csv_path points at the SOURCE csv so editing it needs no rebuild; PF_TEST_WP_CSV (envs/ros2.env
# or FLY_ENV) points it at another file, e.g. /home/user/workspace/ros2/test_runs/<file>.csv.
ros2 run path_following_test path_following_test --ros-args \
    -p wp_csv_path:=${PF_TEST_WP_CSV:-${ROS2_WS}/src/path_following_test/wp.csv} \
    -r /vehicle1/fmu/in/offboard_control_mode:=/fmu/in/offboard_control_mode \
    -r /vehicle1/fmu/in/trajectory_setpoint:=/fmu/in/trajectory_setpoint \
    -r /vehicle1/fmu/in/vehicle_attitude_setpoint:=/fmu/in/vehicle_attitude_setpoint \
    -r /vehicle1/fmu/in/vehicle_command:=/fmu/in/vehicle_command \
    -r /vehicle1/fmu/out/vehicle_status_v1:=/fmu/out/vehicle_status_v1 \
    -r /vehicle1/fmu/out/vehicle_local_position:=/fmu/out/vehicle_local_position \
    -r /vehicle1/fmu/out/vehicle_attitude:=/fmu/out/vehicle_attitude \
    2>&1 | tee ${LOGS}/path_following_test.log &
