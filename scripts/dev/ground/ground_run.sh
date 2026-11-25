#!/bin/bash
# One UGV/USV SITL run: PX4 SITL + Gazebo Classic (headless) in one container, the ROS2 side in a second one
# that shares its network namespace (the layout of the R2 Monte Carlo sets; nothing of the compose stack is used).
#   scripts/dev/ground/ground_run.sh VEHICLE NAME [-- COMMAND...]
#     VEHICLE   ugv (r1_rover, airframe 2010) | usv (boat, airframe 2011)
#     NAME      run directory ~/Documents/A4VAI-SITL/ground_sim/runs/NAME (logs/, sitl_logs/, px4_rootfs/, out/)
#     COMMAND   runs in the ROS2 container after the uXRCE-DDS agent is up (cwd /ground-run); the run ends when
#               it returns. /ground_scripts = scripts/dev/ground (read-only), /ground-run = the run directory.
# env: GROUND_INSTANCE (default 0: network 172.60.0.0/16, ROS_DOMAIN_ID 40), GROUND_WORLD (world file inside
#      ground_sim/worlds, default the stock empty / boat world), GROUND_SPAWN_POSE ("-x .. -y .. -z .. -Y ..", ENU),
#      GROUND_MODEL_SDF, GROUND_PX4_PARAMS ("NAME=VALUE ..." on top of the airframe, this run only), GROUND_CPUS (docker --cpuset-cpus), GROUND_KEEP=1 (leave the containers up).
set -e
HERE=$(dirname $(readlink -f "$0"))
REPO=$(dirname $(dirname $(dirname "$HERE")))
W=~/Documents/A4VAI-SITL
GS=$W/ground_sim
[ $# -ge 2 ] || { sed -n 2,12p "$0"; exit 1; }
VEH=$1; NAME=$2; shift 2
[ "${1:-}" = "--" ] && shift
K=${GROUND_INSTANCE:-0}
NET=ground-net-$K; SITL=ground-sitl-$K; ROS2=ground-ros2-$K
SITL_IMAGE=px4io/px4-dev-simulation-focal:latest
ROS2_IMAGE=$(grep '^ROS2_ENV_IMAGE=' $REPO/envs/ros2.env | cut -d= -f2 | tr -d '"')
RUN=$GS/runs/$NAME
[ -d $GS/etc ] || { echo "[ground_run] run scripts/dev/ground/setup.sh first"; exit 1; }
rm -rf $RUN/px4_rootfs; rm -rf $RUN/pf_logs; mkdir -p $RUN/logs $RUN/sitl_logs $RUN/px4_rootfs $RUN/out $RUN/pf_logs
docker rm -f $ROS2 $SITL > /dev/null 2>&1 || true
docker network inspect $NET > /dev/null 2>&1 || docker network create --subnet 172.$((60 + K)).0.0/16 $NET > /dev/null
CPUS=${GROUND_CPUS:+--cpuset-cpus $GROUND_CPUS}
WORLD=${GROUND_WORLD:+/ground_worlds/$GROUND_WORLD}
cleanup() {
    if [ "${GROUND_KEEP:-0}" != 1 ]; then docker rm -f $ROS2 $SITL > /dev/null 2>&1 || true; fi
}
trap cleanup EXIT
docker run -d --rm --name $SITL --hostname $SITL --network $NET $CPUS --user $(id -u):$(id -g) -e HOME=/tmp \
    -e ROS_DOMAIN_ID=$((40 + K)) -e GROUND_VEHICLE=$VEH -e GROUND_WORLD="$WORLD" \
    -e GROUND_SPAWN_POSE="${GROUND_SPAWN_POSE:-}" -e GROUND_MODEL_SDF="${GROUND_MODEL_SDF:-}" \
    -e GROUND_PX4_PARAMS="${GROUND_PX4_PARAMS:-}" \
    -v $W/PX4-Autopilot:/home/user/workspace:ro -v $GS/etc:/ground_etc:ro -v $GS/worlds:/ground_worlds:ro -v $GS/models:/ground_models:ro \
    -v $RUN/sitl_logs:/ground-logs -v $RUN/px4_rootfs:/px4_rootfs -v $HERE:/ground_scripts:ro \
    --entrypoint bash $SITL_IMAGE /ground_scripts/sitl_entry.sh > /dev/null
docker run --rm --name $ROS2 --network container:$SITL $CPUS --gpus all --user $(id -u):$(id -g) -e HOME=/tmp \
    -e ROS_DOMAIN_ID=$((40 + K)) -e GROUND_VEHICLE=$VEH -e PYCUDA_CACHE_DIR=/ground-pycuda \
    -e OMP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=2 -e PYTHONUNBUFFERED=1 \
    -v $W/ROS2:/home/user/workspace/ros2:ro -v $GS/ws:/home/user/workspace/ground_ws:ro \
    -v $GS/pycuda_cache:/ground-pycuda -v $HERE:/ground_scripts:ro -v $RUN:/ground-run -w /ground-run \
    -v $RUN/pf_logs:/home/user/workspace/ros2/logs -v $GS/courses:/ground_courses:ro \
    --entrypoint bash $ROS2_IMAGE /ground_scripts/ros2_entry.sh "$@"
