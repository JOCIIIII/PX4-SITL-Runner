#!/bin/bash

# ONE-SHOT INSTALLER FOR THE A4VAI SITL SIMULATOR (DISTRIBUTION)
#
# After this finishes:
#   ./scripts/ctrack.sh fly x8_5s first_flight   # C-Track Unreal field, learned avoidance (X8), see README.md
#   ./scripts/run-integrated-algorithm.sh        # legacy AirSim forest: SUPER planner + PathFollowing
#   ./scripts/run-pf-test.sh                     # legacy AirSim forest: PathFollowing only (wp.csv)
# Remove everything with ./scripts/uninstall.sh
#
# Stages (idempotent - safe to re-run after a failure or to upgrade an older install):
#   1. prerequisite checks (docker, compose, nvidia toolkit, display, python3)
#   2. docker images (all public on Docker Hub)
#   3. PX4-Autopilot clone + build (v1.16.0 + A4VAI custom files + X8/X500 models)
#   4. ROS2 workspace sources (repo + pinned submodules, branch ROS2_SRC_BRANCH)
#   5. AirSim ROS2 bridge build (Cosys-AirSim 5.4-v3.2)
#   6. ROS2 workspace build (colcon; again whenever the sources change)
#   7. runtime resources (settings.json, GazeboDrone, AirSim forest binary)
#   8. C-Track Unreal simulator (A4VAI-Unreal at UNREAL_REF + its packaged build, no Unreal Engine needed)
#
# Overrides (environment): ROS2_SRC_REPO, ROS2_SRC_BRANCH, UNREAL_REPO, UNREAL_REF, UNREAL_ASSET_URL,
#   UNREAL_ASSET_DIR (a folder with CTrackSim-Linux.tar.gz, ctrack-gazebo-prebuilt.tar.gz, SHA256SUMS -
#   used instead of downloading them).
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
ROS2_SRC_REPO=${ROS2_SRC_REPO:-"https://github.com/JOCIIIII/A4VAI-Algorithms-ROS2.git"}
ROS2_SRC_BRANCH=${ROS2_SRC_BRANCH:-"inhousesim-v1.0"}
# C-Track Unreal field (UE 5.8 + Cosys-AirSim 3.5): the repo (Gazebo entry, layouts, settings, run_unreal.sh)
# and the release assets built from the same commit by its package.sh
UNREAL_REPO=${UNREAL_REPO:-"https://github.com/JOCIIIII/A4VAI-Unreal.git"}
UNREAL_REF=${UNREAL_REF:-"inhousesim-v1.0"}
UNREAL_ASSET_URL=${UNREAL_ASSET_URL:-"https://github.com/JOCIIIII/A4VAI-Unreal/releases/download/${UNREAL_REF}"}
UNREAL_ASSET_DIR=${UNREAL_ASSET_DIR:-""}
UNREAL_DIR=${CTRACK_UNREAL_DIR}
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
EchoGreen "[install.sh] [1/8] CHECKING PREREQUISITES"
command -v git    >/dev/null || Fail "git not found - run ./scripts/setup-host.sh first"
command -v docker >/dev/null || Fail "docker not found - run ./scripts/setup-host.sh first"
docker compose version >/dev/null 2>&1 || Fail "docker compose plugin not found - run ./scripts/setup-host.sh first"
docker info 2>/dev/null | grep -qi "nvidia" || \
    EchoYellow "[install.sh] WARNING: nvidia runtime not visible in 'docker info' - check NVIDIA Container Toolkit"
if [ -z "${DISPLAY}" ] && [ -z "${WAYLAND_DISPLAY}" ]; then
    Fail "no display (DISPLAY/WAYLAND_DISPLAY empty) - a graphical session is mandatory"
fi
command -v python3 >/dev/null || Fail "python3 not found (the C-Track scripts use it)"
command -v sha256sum >/dev/null || Fail "sha256sum not found"
EchoBoxLine

# ---- 2. DOCKER IMAGES -------------------------------------------------------
EchoGreen "[install.sh] [2/8] PULLING DOCKER IMAGES (Docker Hub)"
for var in ROS2_ENV_IMAGE PX4_ENV_IMAGE GAZEBO_CLASSIC_ENV_IMAGE AIRSIM_BINARY_IMAGE QGC_ENV_IMAGE; do
    img=$(grep -h "^${var}=" ${REPO_DIR}/envs/*.env | cut -d= -f2)
    [ -n "${img}" ] && ! docker image inspect "${img}" >/dev/null 2>&1 && { docker pull "${img}" || Fail "pull failed: ${img}"; }
done
# the C-Track Gazebo/PX4 container (scripts/dev/ctrack_super.sh: ctrack-sitl) runs PX4's own simulation image
CTRACK_SITL_IMAGE="px4io/px4-dev-simulation-focal:latest"
docker image inspect "${CTRACK_SITL_IMAGE}" >/dev/null 2>&1 || docker pull "${CTRACK_SITL_IMAGE}" || Fail "pull failed: ${CTRACK_SITL_IMAGE}"
EchoBoxLine

# ---- 3. PX4 -----------------------------------------------------------------
EchoGreen "[install.sh] [3/8] PX4-AUTOPILOT CLONE + BUILD"
${BASE_DIR}/run.sh px4 clone || Fail "px4 clone failed"
if [ ! -f ${PX4_WORKSPACE}/PX4-Autopilot/build/px4_sitl_default/bin/px4 ]; then
    ${BASE_DIR}/run.sh px4 build || Fail "px4 build failed"
else
    EchoYellow "[install.sh] PX4 already built - skipping"
fi
EchoBoxLine

# ---- 4. ROS2 WORKSPACE SOURCES ---------------------------------------------
EchoGreen "[install.sh] [4/8] ROS2 WORKSPACE SOURCES (${ROS2_SRC_BRANCH})"
mkdir -p $(dirname ${ROS2_SRC})
if [ ! -d ${ROS2_SRC}/.git ] && [ ! -d ${ROS2_SRC}/custom_msgs ]; then
    git clone -b ${ROS2_SRC_BRANCH} ${ROS2_SRC_REPO} ${ROS2_SRC} || Fail "clone A4VAI-Algorithms-ROS2 failed"
elif [ -d ${ROS2_SRC}/.git ] && [ "$(git -C ${ROS2_SRC} rev-parse --abbrev-ref HEAD)" != "${ROS2_SRC_BRANCH}" ]; then
    # an older install (e.g. inhousesim-v0.9): move it to this release's branch
    [ -z "$(git -C ${ROS2_SRC} status --porcelain --untracked-files=no)" ] || \
        Fail "${ROS2_SRC} has local changes - commit or stash them, then re-run (it moves to ${ROS2_SRC_BRANCH})"
    git -C ${ROS2_SRC} fetch origin ${ROS2_SRC_BRANCH} && \
        git -C ${ROS2_SRC} checkout -B ${ROS2_SRC_BRANCH} FETCH_HEAD || Fail "switching ${ROS2_SRC} to ${ROS2_SRC_BRANCH} failed"
fi
# SUPER / A4VAI-PathFollowing / px4_msgs are pinned submodules (their inhousesim-v1.0
# branches carry all SITL patches - ROS2 mode, configs, A* timeout, wp_type, CA/X8 configs).
(cd ${ROS2_SRC} && git submodule sync -q && git submodule update --init A4VAI-PathFollowing super px4_msgs) || \
    Fail "submodule init failed"
mkdir -p ${ROS2_SRC}/super/super_planner/log   # empty dir - not carried by git
chmod -R o+rwx ${ROS2_WORKSPACE}
EchoBoxLine

# ---- 5. AIRSIM ROS2 BRIDGE (Cosys-AirSim, source build) --------------------
EchoGreen "[install.sh] [5/8] AIRSIM ROS2 BRIDGE BUILD (Cosys-AirSim ${COSYS_TAG})"
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
EchoGreen "[install.sh] [6/8] ROS2 WORKSPACE BUILD (colcon)"
# rebuilt whenever the sources moved (a new release, an upgraded install): the stamp is the source revision
SRC_REV=$(cd ${ROS2_SRC} && git rev-parse HEAD 2>/dev/null)-$(cd ${ROS2_SRC} && git submodule status 2>/dev/null | md5sum | cut -c1-12)
STAMP=${ROS2_WORKSPACE}/ros2_ws/.built_rev
if [ ! -d ${ROS2_WORKSPACE}/ros2_ws/install/super_planner ] || [ ! -d ${ROS2_WORKSPACE}/ros2_ws/install/flight_driver ] || \
   [ "$(cat ${STAMP} 2>/dev/null)" != "${SRC_REV}" ]; then
    BuildRun "set -e; source /opt/ros/humble/setup.bash; \
        pip install -q --no-warn-script-location 'packaging>=23.2' 2>/dev/null || true; \
        cd ${ROS2_MNT}/ros2_ws && rm -rf build install log && \
        colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF" \
        || Fail "ros2_ws build failed"
    echo "${SRC_REV}" > ${STAMP}
else
    EchoYellow "[install.sh] ros2_ws already built from these sources - skipping"
fi
EchoBoxLine

# ---- 7. RUNTIME RESOURCES ---------------------------------------------------
EchoGreen "[install.sh] [7/8] RUNTIME RESOURCES"
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

# ---- 8. C-TRACK UNREAL SIMULATOR ---------------------------------------------
EchoGreen "[install.sh] [8/8] C-TRACK UNREAL SIMULATOR (A4VAI-Unreal ${UNREAL_REF}, packaged build)"
if [ ! -d ${UNREAL_DIR}/.git ]; then
    # only what the packaged build needs next to it (Gazebo entry/world, layouts, settings, run_unreal.sh):
    # Content/ (the editor project, ~0.8 GB) is not checked out
    git clone -q --depth 1 --filter=blob:none --sparse --branch ${UNREAL_REF} ${UNREAL_REPO} ${UNREAL_DIR} && \
        git -C ${UNREAL_DIR} sparse-checkout init --cone && \
        git -C ${UNREAL_DIR} sparse-checkout set gazebo layout settings || Fail "clone A4VAI-Unreal ${UNREAL_REF} failed"
fi
if [ ! -x ${UNREAL_DIR}/packaged/CTrackSim-Linux/CTrackSim.sh ] || [ ! -x ${UNREAL_DIR}/gazebo/bin/GazeboDrone ]; then
    ASSETS=${UNREAL_DIR}/.release
    mkdir -p ${ASSETS}
    for a in SHA256SUMS CTrackSim-Linux.tar.gz ctrack-gazebo-prebuilt.tar.gz; do
        [ -f ${ASSETS}/${a} ] && continue
        if [ -n "${UNREAL_ASSET_DIR}" ]; then
            cp "${UNREAL_ASSET_DIR}/${a}" ${ASSETS}/${a} || Fail "${UNREAL_ASSET_DIR}/${a} not found"
        else
            EchoGreen "[install.sh] DOWNLOADING ${a} (C-Track packaged build is ~400 MB)..."
            wget -q --show-progress ${UNREAL_ASSET_URL}/${a} -O ${ASSETS}/${a}.part && mv ${ASSETS}/${a}.part ${ASSETS}/${a} || \
                Fail "download failed: ${UNREAL_ASSET_URL}/${a}"
        fi
    done
    (cd ${ASSETS} && sha256sum -c --quiet SHA256SUMS) || { rm -f ${ASSETS}/*.tar.gz; Fail "checksum mismatch - re-run to download again"; }
    mkdir -p ${UNREAL_DIR}/packaged
    rm -rf ${UNREAL_DIR}/packaged/CTrackSim-Linux
    tar -xzf ${ASSETS}/CTrackSim-Linux.tar.gz -C ${UNREAL_DIR}/packaged || Fail "extracting the packaged build failed"
    tar -xzf ${ASSETS}/ctrack-gazebo-prebuilt.tar.gz -C ${UNREAL_DIR} || Fail "extracting the Gazebo binaries failed"
    rm -f ${ASSETS}/*.tar.gz
else
    EchoYellow "[install.sh] C-Track Unreal already installed - skipping"
fi
EchoBoxLine

# ---- SUMMARY ----------------------------------------------------------------
EchoGreen "[install.sh] INSTALL COMPLETE."
EchoGreen "[install.sh] fly on the C-Track field (README.md):"
EchoGreen "[install.sh]   ./scripts/ctrack.sh fly x8_5s first_flight      (Unreal starts with the profile's airframe)"
EchoGreen "[install.sh] legacy AirSim forest flows:"
EchoGreen "[install.sh]   ./scripts/run-integrated-algorithm.sh      (SUPER + PathFollowing, collision avoidance)"
EchoGreen "[install.sh]   ./scripts/run-pf-test.sh    (PathFollowing only)"
EchoGreen "[install.sh] uninstall with: ./scripts/uninstall.sh"
