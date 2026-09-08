#!/bin/bash

# ONE-SHOT INSTALLER FOR THE A4VAI SITL SIMULATOR (DISTRIBUTION)
#
# After this finishes, run either:
#   ./scripts/run-integrated-algorithm.sh      # SUPER planner + PathFollowing (collision avoidance)
#   ./scripts/run-pf-test.sh    # PathFollowing only (fixed wp.csv waypoints)
# Remove everything with ./scripts/uninstall.sh
#
# Stages (idempotent - safe to re-run after a failure):
#   1. prerequisite checks (docker, compose, nvidia toolkit, display)
#   2. docker images (all public on Docker Hub)
#   3. PX4-Autopilot clone + build (v1.16.0 + A4VAI custom files)
#   4. ROS2 workspace sources (repo + pinned submodules)
#   5. AirSim ROS2 bridge build (Cosys-AirSim 5.4-v3.2)
#   6. ROS2 workspace build (colcon)
#   7. runtime resources (settings.json, GazeboDrone, AirSim binary)
#
# The single ROS2 image (ROS2_ENV_IMAGE in envs/ros2.env) is devel-grade:
# it both RUNS the sim and BUILDS the workspaces - no separate build image.

set -o pipefail

BASE_DIR=$(dirname $(readlink -f "$0"))
REPO_DIR=$(dirname ${BASE_DIR})

source ${BASE_DIR}/include/commonFcn.sh
source ${BASE_DIR}/include/commonEnv.sh

# SAVE THE WHOLE INSTALL OUTPUT TO ${SITL_DEPLOY_DIR}/logs/install_<timestamp>.log
LogToFile install "$0" "$@"

# ---- SOURCES / ASSETS (edit here if forks move) -----------------------------
ROS2_SRC_REPO="https://github.com/JOCIIIII/A4VAI-Algorithms-ROS2.git"
ROS2_SRC_BRANCH="inhousesim-v0.9"
COSYS_REPO="https://github.com/Cosys-Lab/Cosys-AirSim.git"
COSYS_TAG="5.4-v3.2"                       # must match the AirSim Unreal binary
RESOURCE_URL="https://github.com/kestr31/PX4-SITL-Runner/releases/download/Resources"
AIRSIM_BIN_URL="https://github.com/JOCIIIII/PX4-SITL-Runner/releases/download/inhousesim-v0.9/sim_world.tar.gz"

ROS2_SRC=${ROS2_WORKSPACE}/ros2_ws/src
ROS2_IMAGE=$(grep '^ROS2_ENV_IMAGE=' ${REPO_DIR}/envs/ros2.env | cut -d= -f2)
# builds must mount the ROS2 dir at the SAME path the sim uses -
# colcon hardcodes absolute paths into the install tree.
ROS2_MNT="/home/user/workspace/ros2"
# -----------------------------------------------------------------------------

Fail() { EchoRed "[install.sh] $1"; exit 1; }

# COLCON BUILDS RUN IN THE (devel-grade) ROS2 IMAGE.
BuildRun() { docker run --rm --entrypoint bash -v ${ROS2_WORKSPACE}:${ROS2_MNT} ${ROS2_IMAGE} -c "$1"; }

# ---- 1. PREREQUISITES -------------------------------------------------------
EchoGreen "[install.sh] [1/7] CHECKING PREREQUISITES"
command -v git    >/dev/null || Fail "git not found - run ./scripts/setup-host.sh first"
command -v docker >/dev/null || Fail "docker not found - run ./scripts/setup-host.sh first"
docker compose version >/dev/null 2>&1 || Fail "docker compose plugin not found - run ./scripts/setup-host.sh first"
docker info 2>/dev/null | grep -qi "nvidia" || \
    EchoYellow "[install.sh] WARNING: nvidia runtime not visible in 'docker info' - check NVIDIA Container Toolkit"
if [ -z "${DISPLAY}" ] && [ -z "${WAYLAND_DISPLAY}" ]; then
    Fail "no display (DISPLAY/WAYLAND_DISPLAY empty) - a graphical session is mandatory"
fi
EchoBoxLine

# ---- 2. DOCKER IMAGES -------------------------------------------------------
EchoGreen "[install.sh] [2/7] PULLING DOCKER IMAGES (Docker Hub)"
for var in ROS2_ENV_IMAGE PX4_ENV_IMAGE GAZEBO_CLASSIC_ENV_IMAGE AIRSIM_BINARY_IMAGE QGC_ENV_IMAGE; do
    img=$(grep -h "^${var}=" ${REPO_DIR}/envs/*.env | cut -d= -f2)
    [ -n "${img}" ] && ! docker image inspect "${img}" >/dev/null 2>&1 && { docker pull "${img}" || Fail "pull failed: ${img}"; }
done
EchoBoxLine

# ---- 3. PX4 -----------------------------------------------------------------
EchoGreen "[install.sh] [3/7] PX4-AUTOPILOT CLONE + BUILD"
${BASE_DIR}/run.sh px4 clone || Fail "px4 clone failed"
if [ ! -f ${PX4_WORKSPACE}/PX4-Autopilot/build/px4_sitl_default/bin/px4 ]; then
    ${BASE_DIR}/run.sh px4 build || Fail "px4 build failed"
else
    EchoYellow "[install.sh] PX4 already built - skipping"
fi
EchoBoxLine

# ---- 4. ROS2 WORKSPACE SOURCES ---------------------------------------------
EchoGreen "[install.sh] [4/7] ROS2 WORKSPACE SOURCES"
mkdir -p $(dirname ${ROS2_SRC})
if [ ! -d ${ROS2_SRC}/.git ] && [ ! -d ${ROS2_SRC}/custom_msgs ]; then
    git clone -b ${ROS2_SRC_BRANCH} ${ROS2_SRC_REPO} ${ROS2_SRC} || Fail "clone A4VAI-Algorithms-ROS2 failed"
fi
# SUPER / A4VAI-PathFollowing / px4_msgs are pinned submodules (inhousesim-v0.9
# branches carry all SITL patches - ROS2 mode, configs, A* timeout, wp_type).
(cd ${ROS2_SRC} && git submodule update --init A4VAI-PathFollowing super px4_msgs) || \
    Fail "submodule init failed"
mkdir -p ${ROS2_SRC}/super/super_planner/log   # empty dir - not carried by git
chmod -R o+rwx ${ROS2_WORKSPACE}
EchoBoxLine

# ---- 5. AIRSIM ROS2 BRIDGE (Cosys-AirSim, source build) --------------------
EchoGreen "[install.sh] [5/7] AIRSIM ROS2 BRIDGE BUILD (Cosys-AirSim ${COSYS_TAG})"
COSYS=${ROS2_WORKSPACE}/cosys-airsim
[ -d ${COSYS} ] || git clone --depth 1 --branch ${COSYS_TAG} ${COSYS_REPO} ${COSYS} || Fail "clone Cosys-AirSim failed"
if [ ! -d ${COSYS}/external/rpclib/rpclib-2.3.1 ]; then
    (cd ${COSYS} && wget -q https://github.com/WouterJansen/rpclib/archive/refs/tags/v2.3.1.zip && \
     mkdir -p external/rpclib && unzip -q v2.3.1.zip -d external/rpclib && \
     mv external/rpclib/rpclib-2.3.1* external/rpclib/rpclib-2.3.1 2>/dev/null; rm -f v2.3.1.zip) || Fail "rpclib fetch failed"
fi
if [ ! -d ${COSYS}/AirLib/deps/eigen3/Eigen ]; then
    (cd ${COSYS} && wget -q -O eigen3.zip https://github.com/WouterJansen/eigen/archive/refs/tags/3.4.1r.zip && \
     unzip -q eigen3.zip -d temp_eigen && mkdir -p AirLib/deps/eigen3 && \
     mv temp_eigen/eigen*/Eigen AirLib/deps/eigen3/ && rm -rf temp_eigen eigen3.zip) || Fail "eigen fetch failed"
fi
if [ ! -f ${COSYS}/ros2/install/setup.bash ]; then
    BuildRun "set -e; source /opt/ros/humble/setup.bash; \
        pip install -q --no-warn-script-location 'packaging>=23.2' 2>/dev/null || true; \
        cd ${ROS2_MNT}/cosys-airsim/ros2 && \
        colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release" \
        || Fail "airsim bridge build failed"
fi
# point airsim/install at the source build (backup any previous install)
mkdir -p ${ROS2_WORKSPACE}/airsim
if [ ! -L ${ROS2_WORKSPACE}/airsim/install ]; then
    [ -d ${ROS2_WORKSPACE}/airsim/install ] && mv ${ROS2_WORKSPACE}/airsim/install ${ROS2_WORKSPACE}/airsim/install.bak
    ln -s ../cosys-airsim/ros2/install ${ROS2_WORKSPACE}/airsim/install
fi
EchoBoxLine

# ---- 6. ROS2 WORKSPACE BUILD ------------------------------------------------
EchoGreen "[install.sh] [6/7] ROS2 WORKSPACE BUILD (colcon)"
if [ ! -d ${ROS2_WORKSPACE}/ros2_ws/install/super_planner ]; then
    BuildRun "set -e; source /opt/ros/humble/setup.bash; \
        pip install -q --no-warn-script-location 'packaging>=23.2' 2>/dev/null || true; \
        cd ${ROS2_MNT}/ros2_ws && rm -rf build install log && \
        colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF" \
        || Fail "ros2_ws build failed"
else
    EchoYellow "[install.sh] ros2_ws already built - skipping"
fi
EchoBoxLine

# ---- 7. RUNTIME RESOURCES ---------------------------------------------------
EchoGreen "[install.sh] [7/7] RUNTIME RESOURCES"
cp ${REPO_DIR}/resources/airsim/settings.json ${AIRSIM_WORKSPACE}/settings.json
if [ ! -f ${GAZEBO_CLASSIC_WORKSPACE}/GazeboDrone ]; then
    wget -q --show-progress ${RESOURCE_URL}/GazeboDrone -O ${GAZEBO_CLASSIC_WORKSPACE}/GazeboDrone && \
    chmod +x ${GAZEBO_CLASSIC_WORKSPACE}/GazeboDrone
fi
# AirSim Unreal binary (sim_world world package)
if [ -z "$(ls ${AIRSIM_WORKSPACE}/binary/*.sh 2>/dev/null)" ]; then
    EchoGreen "[install.sh] DOWNLOADING AIRSIM BINARY (sim_world, ~280MB)..."
    mkdir -p ${AIRSIM_WORKSPACE}/binary
    wget -q --show-progress ${AIRSIM_BIN_URL} -O ${AIRSIM_WORKSPACE}/sim_world.tar.gz && \
    tar -xzf ${AIRSIM_WORKSPACE}/sim_world.tar.gz -C ${AIRSIM_WORKSPACE}/binary && \
    chmod +x ${AIRSIM_WORKSPACE}/binary/*.sh && \
    rm -f ${AIRSIM_WORKSPACE}/sim_world.tar.gz || \
    EchoYellow "[install.sh] AirSim binary download failed - place it manually in ${AIRSIM_WORKSPACE}/binary/"
fi
EchoBoxLine

# ---- SUMMARY ----------------------------------------------------------------
EchoGreen "[install.sh] INSTALL COMPLETE."
EchoGreen "[install.sh] run the simulator with:"
EchoGreen "[install.sh]   ./scripts/run-integrated-algorithm.sh      (SUPER + PathFollowing, collision avoidance)"
EchoGreen "[install.sh]   ./scripts/run-pf-test.sh    (PathFollowing only)"
EchoGreen "[install.sh] uninstall with: ./scripts/uninstall.sh"
