#!/bin/bash
# Runs inside ground-sitl-<name> (px4io/px4-dev-simulation-focal, own docker bridge network): gzserver + the
# vehicle model + PX4 SITL with the private etc copy (/ground_etc: the UGV/USV airframes 2010/2011, uXRCE-DDS
# client to 127.0.0.1). The ROS2 container shares this network namespace. See ground_run.sh.
set -e
PX4_SRC=/home/user/workspace/PX4-Autopilot
PX4_BUILD=${PX4_SRC}/build/px4_sitl_default
LOG=/ground-logs
mkdir -p ${LOG}
case "${GROUND_VEHICLE:-ugv}" in
    ugv) MODEL=r1_rover; export PX4_SYS_AUTOSTART=2010; DEF_POSE="-x 0 -y 0 -z 0.08 -Y 1.570796" ;;
    usv) MODEL=boat;     export PX4_SYS_AUTOSTART=2011; DEF_POSE="-x 0 -y 0 -z 0.10 -Y 1.570796" ;;
    *) echo "[ground] unknown vehicle ${GROUND_VEHICLE}"; exit 1 ;;
esac
MODELS=${PX4_SRC}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
source ${PX4_SRC}/Tools/simulation/gazebo-classic/setup_gazebo.bash ${PX4_SRC} ${PX4_BUILD} > /dev/null
export GAZEBO_IP=127.0.0.1
WORLD=${GROUND_WORLD:-}
if [ -z "${WORLD}" ]; then
    # UGV: the stock empty world with the contact correcting velocity limited (setup.sh models)
    [ "${MODEL}" = boat ] && WORLD=${PX4_SRC}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/boat.world \
                          || WORLD=/ground_worlds/ugv_flat.world
fi
echo "[ground] vehicle ${GROUND_VEHICLE} (${MODEL}, autostart ${PX4_SYS_AUTOSTART}), world ${WORLD}"
gzserver --verbose ${WORLD} > ${LOG}/gzserver.log 2>&1 &

SDF=${GROUND_MODEL_SDF:-/ground_models/${MODEL}.sdf}
SPAWN="--model-name=${MODEL} ${GROUND_SPAWN_POSE:-${DEF_POSE}}"
while gz model --verbose --spawn-file=${SDF} ${SPAWN} 2>&1 | grep -q "An instance of Gazebo is not running"; do
    sleep 1
done
echo "[ground] ${MODEL} spawned (${SPAWN})"

# let the model come to rest before PX4 starts: the estimator aligns on its first IMU samples, and aligned
# while the rover was still dropping onto its wheels it started 2-3 deg tilted = up to 16 deg off in yaw
# (63 deg magnetic inclination), which the mag fusion then kept (ekf_e)
sleep ${GROUND_SPAWN_SETTLE_S:-6}
export PX4_SIM_MODEL=gazebo-classic_${MODEL}
mkdir -p /px4_rootfs
cd /px4_rootfs
ETC=/ground_etc
if [ -n "${GROUND_PX4_PARAMS:-}" ]; then      # "NAME=VALUE ..." on top of the airframe file, this run only
    ETC=/tmp/ground_etc; cp -r /ground_etc ${ETC}
    AF=$(ls ${ETC}/init.d-posix/airframes/${PX4_SYS_AUTOSTART}_*)
    for kv in ${GROUND_PX4_PARAMS}; do echo "param set-default ${kv%%=*} ${kv#*=}" >> ${AF}; done
    echo "[ground] extra parameters: ${GROUND_PX4_PARAMS}"
fi
${PX4_BUILD}/bin/px4 -d -w /px4_rootfs ${ETC} > ${LOG}/px4.log 2>&1 &
wait
