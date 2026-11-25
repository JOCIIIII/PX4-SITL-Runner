#!/bin/bash
# Runs inside the ROS2 container (ground_run.sh VEHICLE NAME -- /ground_scripts/ground_mission.sh ...):
# PathFollowing (the overlay copy with the ground output stage) + ground_driver on a waypoint CSV.
#   ground_mission.sh CSV SPEED [GUID] [driver options, e.g. --frame local --origin N,E --timeout 200]
#     GUID  2 (default): MPPI guidance, 0: speed-regulated pursuit only (no node_mppi)
# env: GROUND_SIM_YAML (default sim_ground_<vehicle>.yaml), GROUND_PF_ARGS (extra -p name:=value)
set -e
CSV=$1; SPEED=$2; GUID=${3:-2}; shift 3 || shift 2
case "${GROUND_VEHICLE}" in
    ugv) VT=3 ;; usv) VT=4 ;; *) echo "[ground_mission] GROUND_VEHICLE '${GROUND_VEHICLE}'?"; exit 1 ;;
esac
YAML=${GROUND_SIM_YAML:-sim_ground_${GROUND_VEHICLE}.yaml}
LOG=/ground-run/logs
PF_ARGS="-p vehicle_type:=${VT} -p guid_type:=${GUID} -p sim_yaml:=${YAML} ${GROUND_PF_ARGS:-}"
echo "[ground_mission] ${GROUND_VEHICLE}: ${CSV} at ${SPEED} m/s, guidance ${GUID}, ${YAML}"
ros2 run pathfollowing node_pathfollowing --ros-args ${PF_ARGS} > ${LOG}/node_pathfollowing.log 2>&1 &
PF_PID=$!
if [ "${GUID}" != 0 ]; then
    ros2 run pathfollowing node_mppi --ros-args ${PF_ARGS} > ${LOG}/node_mppi.log 2>&1 &
    MPPI_PID=$!
fi
python3 /ground_scripts/ground_driver.py --csv ${CSV} --speed ${SPEED} "$@" 2>&1 | tee ${LOG}/ground_driver.log
kill ${PF_PID} ${MPPI_PID:-} 2>/dev/null || true
cp /home/user/workspace/ros2/logs/*.csv /ground-run/out/pf_log.csv 2>/dev/null || true
