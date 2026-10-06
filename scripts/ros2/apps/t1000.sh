#! /bin/bash

# ROS2 APP: T1000 Control 445 (learned local-avoidance policy, ONNX) on AirSim lidar.
#   px4_lidar_to_odom_cloud (/odom, world ENU = PX4 origin; /cloud only for recording)
#   -> t1000_bridge.py   (/t1000/pose, /t1000/velocity, /t1000/lidar_bins [432])
#   -> T1000 path_publisher (/t1000/path: straight line start -> goal, world ENU)
#   -> T1000 node in its SITL mode (--isaac-sim --sim-auto-start): OFFBOARD, arm, velocity
#      setpoints on /fmu/in/trajectory_setpoint, LAND after the goal dwell.
# SUPER / PathFollowing / flight_manager are not started: T1000 owns the offboard stream.
# T1000 sources: ros2_ws/src/T1000_Control_445 (run in place, not a colcon package).
#
# knobs (env): T1000_FLIGHT_ALTITUDE_M (default 4.0), T1000_POLICY_AUTHORITY (always |
# obstacle, default always), T1000_VX_CAP / T1000_VY_CAP (world ENU x / y command caps,
# default 3 / 1.5 m/s), T1000_SETTINGS_JSON (settings.py overrides, default widens the
# avoidance forward sector to +-60 deg), T1000_CLIMB_FIRST (default 0.8), T1000_GOAL "x,y"
# (default: first row of realgazebo_fastplanner_adapters/config/goals_super_far.csv, shared
# with super_pf). See t1000_run.py and t1000_bridge.py.
#
# Differences from T1000's README SITL example, each from flights in this simulator:
#  - lidar bins numbered counter-clockwise (t1000_bridge.py bin_azimuth): the policy was
#    trained that way; the README's xyz_to_432bins is the mirror image.
#  - --policy-authority always (settings.py's own default) instead of the README's obstacle:
#    obstacle mode's PASS phase holds the side-step lane with no policy authority and flew
#    into the second tree of a pair (0.10 / 0.33 m); always: 4/4 runs >= 1.85 m clearance.
#  - vy cap 1.5 and a +-60 deg forward sector: tuned while flying obstacle mode, kept as part
#    of the validated configuration.

# WORKSPACE_DIR is exported by the launcher; fall back if run standalone.
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
for pkg in realgazebo_fastplanner_adapters px4_msgs; do
    if [ ! -d "${ROS2_WS}/install/${pkg}" ]; then
        echo "[$(basename "$0")] ERROR: ${pkg} is not built."
        echo "[$(basename "$0")] Build first: ./scripts/run.sh ros2 build ros2_ws"
        exit 1
    fi
done

source ${ROS2_WS}/install/setup.bash
export PYTHONPATH=${T1000_DIR}:${PYTHONPATH}
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 NUMEXPR_NUM_THREADS=2

# Do not fly blind: wait for real scans (AirSim up and bridged) before anything can arm.
LIDAR_TOPIC=/airsim_node/SimpleFlight/lidar/points/RPLIDAR_A3
echo "[$(basename "$0")] waiting for lidar data on ${LIDAR_TOPIC} ..."
if ! timeout 300 ros2 topic echo --once --qos-profile sensor_data --field header.stamp \
        ${LIDAR_TOPIC} > /dev/null 2>&1; then
    echo "[$(basename "$0")] ERROR: no lidar data within 300 s (AirSim not running?) - not taking off."
    exit 1
fi

GOAL=${T1000_GOAL:-$(sed -n 2p ${GOALS_CSV} | tr -d ' ' | cut -d, -f1,2)}
echo "[$(basename "$0")] path: 0,0 -> ${GOAL}  altitude ${T1000_FLIGHT_ALTITUDE_M:-4.0} m"

# 1) PX4 pose -> /odom (ENU); /cloud (AirSim-registered, de-skewed) is kept for the recorder
ros2 run realgazebo_fastplanner_adapters px4_lidar_to_odom_cloud --ros-args \
    -p "px4_ns:=''" \
    -p in_topic:=${LIDAR_TOPIC} \
    -p use_ros_time:=true -p cloud_latency_s:=0.13 \
    -p airsim_odom_topic:=/airsim_node/SimpleFlight/odom_local \
    -p synth_free_rays:=false -p deskew_rot_period_s:=0.1 \
    -p lidar_offset_x:=0.0 -p lidar_offset_y:=0.0 -p lidar_offset_z:=0.3 \
    -p lidar_roll:=0.0 -p lidar_pitch:=0.0 -p lidar_yaw:=0.0 \
    2>&1 | tee ${LOGS}/px4_lidar_to_odom_cloud.log &

# 2) simulator inputs in T1000's SITL contract
python3 ${APP_DIR}/t1000_bridge.py --ros-args -p lidar_topic:=${LIDAR_TOPIC} \
    2>&1 | tee ${LOGS}/t1000_bridge.log &

# 3) path (republished at 2 Hz; repeats refresh it without resetting progress)
python3 -m t1000_control.ros.path_publisher --points "0,0 ${GOAL}" --frame map --rate 2 \
    2>&1 | tee ${LOGS}/t1000_path.log &

# 4) T1000 controller (SITL lifecycle: OFFBOARD at 1.5 s, arm at 2 s, LAND on goal)
sleep 3
export T1000_SETTINGS_JSON=${T1000_SETTINGS_JSON:-'{"AVOID_FORWARD_HALF_ANGLE_DEG": 60}'}
T1000_FLIGHT_ALTITUDE_M=${T1000_FLIGHT_ALTITUDE_M:-4.0} python3 ${APP_DIR}/t1000_run.py \
    --isaac-sim --sim-auto-start --allow-no-rc --confirm-sitl \
    --vx-cap ${T1000_VX_CAP:-3} --vy-cap ${T1000_VY_CAP:-1.5} \
    --corner-mode rounded --policy-authority ${T1000_POLICY_AUTHORITY:-always} \
    --ordered-waypoints --goal-stop --until-goal --land-on-goal \
    2>&1 | tee ${LOGS}/t1000_control.log &

wait
