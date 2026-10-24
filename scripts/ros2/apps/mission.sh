#! /bin/bash

# ROS2 APP: a waypoint mission flown by PathFollowing alone (no planner, no avoidance) under flight_driver -
# the vehicle's GPS-waypoint flight, A4VAI-Flight-Test's path_following_test flow with our PathFollowing
# (scripts/dev/onboard_real.sh mission on the vehicle; SITL: ROS2_APP=mission).
#   node_pathfollowing + node_mppi: MPPI guidance (guid 2) with the config flown under the learned CA
#     (sim_ca_mppi.yaml; x8: sim_ca_mppi_x8.yaml) and the corner speed limit (MISSION_CORNER=1, default: 90 deg
#     corners at 10 m/s within 1.2 m instead of 8-10 m off, scripts/dev/pf_wind_batch.py runs)
#   flight_driver (ros2_ws/src/flight_driver): AUTO_TAKEOFF to MISSION_ALT above the take-off point, OFFBOARD,
#     the waypoints to PathFollowing at MISSION_SPEED, lands where PathFollowing ends; yields to the pilot,
#     lands in place on lost inputs (see its header)
# knobs (env):
#   MISSION_CSV       waypoints (required): wp_gps rows "lat,lon,alt", wp_local rows "x,y,z" = north, east; the
#                     heights are above the take-off point. scripts/dev/gps_mission.py checks / converts them
#   MISSION_TYPE      wp_gps | wp_local (default wp_local)
#   MISSION_SPEED     cruise speed [m/s] (default 3.0), MISSION_ALT take-off height [m] (default 5.0)
#   MISSION_REQUIRE_GO=1   arm only on the operator's Bool on /flight_driver/go (the vehicle; default 0)
#   MISSION_EKF_SETTLE     [s] of an unreset PX4 estimate before arming (default 30; SITL 5)
#   MISSION_SPEED_LIMIT    [m/s] lands in place above it (default MISSION_SPEED + 2.5)
#   MISSION_HOVER_THRUST / MISSION_MAX_TILT_DEG   the vehicle's measured hover thrust and its tilt limit for both
#                     PathFollowing nodes (0 = the yaml), MISSION_SIM_YAML another PathFollowing config
# Logs: logs/flight_driver_<time>.csv (the FSM, 10 Hz), PathFollowing's own CSV, *.log.

if [ -z "${WORKSPACE_DIR}" ]; then
    WORKSPACE_DIR=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
fi
ROS2_WS=${WORKSPACE_DIR}/ros2_ws
LOGS=${WORKSPACE_DIR}/logs
mkdir -p ${LOGS}

for pkg in pathfollowing flight_driver custom_msgs px4_msgs; do
    if [ ! -d "${ROS2_WS}/install/${pkg}" ]; then
        echo "[$(basename "$0")] ERROR: ${pkg} is not built."
        echo "[$(basename "$0")] Build first: ./scripts/run.sh ros2 build ros2_ws"
        exit 1
    fi
done
CSV=${MISSION_CSV:-}
TYPE=${MISSION_TYPE:-wp_local}
if [ -z "${CSV}" ] || [ ! -f "${CSV}" ]; then
    echo "[$(basename "$0")] ERROR: MISSION_CSV '${CSV}' not found (rows lat,lon,alt or north,east,height)."
    exit 1
fi
source ${ROS2_WS}/install/setup.bash
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 NUMEXPR_NUM_THREADS=2

if [ "${SITL_AIRFRAME}" = "x8" ]; then
    PF_VEHICLE_TYPE=2; YAML=sim_ca_mppi_x8.yaml
else
    PF_VEHICLE_TYPE=1; YAML=sim_ca_mppi.yaml
fi
YAML=${MISSION_SIM_YAML:-${YAML}}
SPEED=$(printf '%.2f' ${MISSION_SPEED:-3.0})
PF_ARGS="-p vehicle_type:=${PF_VEHICLE_TYPE} -p guid_type:=2 -p sim_yaml:=${YAML} \
    -p hover_thrust:=$(printf '%.4f' ${MISSION_HOVER_THRUST:-0}) -p max_tilt_deg:=$(printf '%.1f' ${MISSION_MAX_TILT_DEG:-0})"
[ "${MISSION_CORNER:-1}" = 1 ] && PF_ARGS="${PF_ARGS} -p corner_speed_enable:=true -p mppi_rollout_sync:=true"
echo "[$(basename "$0")] ${TYPE} ${CSV} at ${SPEED} m/s, take-off ${MISSION_ALT:-5.0} m; PathFollowing ${YAML}" \
     "(vehicle ${PF_VEHICLE_TYPE}, corner limit ${MISSION_CORNER:-1})"
cat "${CSV}"

ros2 run pathfollowing node_pathfollowing --ros-args ${PF_ARGS} 2>&1 | tee ${LOGS}/node_pathfollowing.log &
ros2 run pathfollowing node_mppi --ros-args ${PF_ARGS} 2>&1 | tee ${LOGS}/node_mppi.log &

ros2 run flight_driver flight_driver --ros-args -p mission:=${TYPE} -p wp_csv_path:=${CSV} \
    -p takeoff_altitude:=$(printf '%.2f' ${MISSION_ALT:-5.0}) -p desired_speed:=${SPEED} \
    -p require_go:=$([ "${MISSION_REQUIRE_GO:-0}" = 1 ] && echo true || echo false) \
    -p ekf_settle_s:=$(printf '%.1f' ${MISSION_EKF_SETTLE:-30}) \
    -p speed_limit_mps:=$(printf '%.2f' ${MISSION_SPEED_LIMIT:-$(python3 -c "print(${SPEED} + 2.5)")}) \
    -p log_dir:=${LOGS} 2>&1 | tee ${LOGS}/flight_driver.log &

wait
