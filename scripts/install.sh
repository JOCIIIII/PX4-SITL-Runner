#!/bin/bash

# ONE-SHOT INSTALLER FOR THE A4VAI SITL SIMULATOR (DISTRIBUTION)
#
# After this finishes (+ placing the AirSim binary), run either:
#   ./scripts/run-super.sh      # SUPER planner + PathFollowing (collision avoidance)
#   ./scripts/run-pf-test.sh    # PathFollowing only (fixed wp.csv waypoints)
# Remove everything with ./scripts/uninstall.sh
#
# Stages (idempotent - safe to re-run after a failure):
#   1. prerequisite checks (docker, compose, nvidia toolkit, display)
#   2. docker images (all public on Docker Hub - pulled automatically)
#   3. PX4-Autopilot clone + build (v1.16.0 + A4VAI custom files)
#   4. ROS2 workspace sources + SUPER setup
#   5. AirSim ROS2 bridge build (Cosys-AirSim 5.4-v3.2, built in the devel image)
#   6. ROS2 workspace build (colcon, in the devel image)
#   7. runtime resources (AirSim settings.json, GazeboDrone)
#
# NOT automated (must be provided manually):
#   - AirSim Unreal binary package -> ~/Documents/A4VAI-SITL/AirSim/binary/
#     (full package incl. Engine/ dir + exactly one launcher .sh)

set -o pipefail

BASE_DIR=$(dirname $(readlink -f "$0"))
REPO_DIR=$(dirname ${BASE_DIR})

source ${BASE_DIR}/include/commonFcn.sh
source ${BASE_DIR}/include/commonEnv.sh

# ---- SOURCE REPOSITORIES / IMAGES (edit here if forks move) -----------------
ROS2_SRC_REPO="https://github.com/JOCIIIII/A4VAI-Algorithms-ROS2.git"
ROS2_SRC_BRANCH="inhousesim-v0.9"
COSYS_REPO="https://github.com/Cosys-Lab/Cosys-AirSim.git"
COSYS_TAG="5.4-v3.2"                       # must match the AirSim Unreal binary
DEVEL_IMAGE="jociiiii/a4vai:devel"          # build environment (compilers/colcon)

RESOURCE_URL="https://github.com/kestr31/PX4-SITL-Runner/releases/download/Resources"
# AirSim Unreal binary (sim_world, Cosys-AirSim 5.4-v3.2 world) - release asset
AIRSIM_BIN_URL="https://github.com/JOCIIIII/PX4-SITL-Runner/releases/download/inhousesim-v0.9/sim_world.tar.gz"
ROS2_SRC=${ROS2_WORKSPACE}/ros2_ws/src
# ROS2 dir must be mounted at the SAME path for build and runtime -
# colcon hardcodes absolute paths into the install tree.
ROS2_MNT="/home/user/workspace/ros2"
# -----------------------------------------------------------------------------

Fail() { EchoRed "[install.sh] $1"; exit 1; }

# BUILDS RUN IN THE DEVEL IMAGE (the runtime image has no compilers).
DevelRun() { docker run --rm --entrypoint bash -v ${ROS2_WORKSPACE}:${ROS2_MNT} ${DEVEL_IMAGE} -c "$1"; }

# ---- 1. PREREQUISITES -------------------------------------------------------
EchoGreen "[install.sh] [1/7] CHECKING PREREQUISITES"
command -v git    >/dev/null || Fail "git not found"
command -v docker >/dev/null || Fail "docker not found"
docker compose version >/dev/null 2>&1 || Fail "docker compose plugin not found"
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
docker image inspect ${DEVEL_IMAGE} >/dev/null 2>&1 || docker pull ${DEVEL_IMAGE} || Fail "pull failed: ${DEVEL_IMAGE}"
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
    DevelRun "set -e; source /opt/ros/humble/setup.bash; \
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
EchoGreen "[install.sh] [6/7] ROS2 WORKSPACE BUILD (colcon, devel image)"
if [ ! -d ${ROS2_WORKSPACE}/ros2_ws/install/super_planner ]; then
    DevelRun "set -e; source /opt/ros/humble/setup.bash; \
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
    EchoGreen "[install.sh] DOWNLOADING AIRSIM BINARY (sim_world, ~large file)..."
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
if [ ! -d ${AIRSIM_WORKSPACE}/binary ] || [ -z "$(ls ${AIRSIM_WORKSPACE}/binary/*.sh 2>/dev/null)" ]; then
    EchoYellow "[install.sh] REMAINING MANUAL STEP:"
    EchoYellow "[install.sh]   place the AirSim Unreal binary package (with Engine/ dir and ONE launcher .sh)"
    EchoYellow "[install.sh]   into ${AIRSIM_WORKSPACE}/binary/"
fi
EchoGreen "[install.sh] run the simulator with:"
EchoGreen "[install.sh]   ./scripts/run-super.sh      (SUPER + PathFollowing, collision avoidance)"
EchoGreen "[install.sh]   ./scripts/run-pf-test.sh    (PathFollowing only)"
EchoGreen "[install.sh] uninstall with: ./scripts/uninstall.sh"
