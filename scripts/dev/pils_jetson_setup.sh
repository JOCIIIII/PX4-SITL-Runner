#!/bin/bash
# PILS: set up the onboard computer (Jetson Orin, aarch64 / JetPack 6) to run the ROS2 apps against this
# PC's simulation - scripts/dev/pils_onboard.sh runs them there, CTRACK_MODE=pils in
# scripts/dev/ctrack_super.sh flies them. Idempotent; rerun after changing the apps or their sources.
#   1. this checkout, uncommitted work included -> ~/PX4-SITL-Runner there
#   2. sources from this PC's workspace -> ~/Documents/A4VAI-SITL/ROS2 there (the same layout as here):
#      ros2_ws/src without SUPER (not run onboard; T1000 runs in place - t1000_pf.sh) and the Cosys-AirSim source the bridge is
#      built from (without docs, the Unreal plugin, the clients, git history)
#   3. the onboard image: BASE (aarch64 Humble with MicroXRCEAgent, onnxruntime, scipy, foxglove_bridge,
#      PCL and NVIDIA's OpenCV 4.8) + the AirSim bridge's ROS dependencies from apt. cv_bridge is built
#      from source against that OpenCV instead (the apt one brings Ubuntu's OpenCV 4.5 into the process)
#   4. colcon builds in that image with the workspace mounted where the apps run it
#      (/home/user/workspace/ros2 - colcon keeps absolute paths): cv_bridge (an underlay the bridge's
#      setup.bash chains), the bridge, then px4_msgs .. flight_manager, flight_driver, pathfollowing, the lidar adapter
#
#   scripts/dev/pils_jetson_setup.sh [user@host]      default jetson@192.168.219.152 (orin-nano-01, LAN)
# env: PILS_BASE_IMAGE (default ros2-px4-jetson:humble-a4vai-hybrid), PILS_IMAGE (default a4vai-pils:humble),
#      PILS_SKIP_BUILD=1 (sync only - scripts, sources, policy bundles; python changes need no build)
set -eu
HOST=${1:-jetson@192.168.219.152}
REPO=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
WS=$HOME/Documents/A4VAI-SITL/ROS2
RWS=Documents/A4VAI-SITL/ROS2          # there, relative to the login's home
BASE=${PILS_BASE_IMAGE:-ros2-px4-jetson:humble-a4vai-hybrid}
IMG=${PILS_IMAGE:-a4vai-pils:humble}
MNT=/home/user/workspace/ros2
on() { ssh -o BatchMode=yes "$HOST" "$@"; }

echo "[pils_setup] 1/4 $REPO -> $HOST:PX4-SITL-Runner"
rsync -a --delete --exclude /.git --exclude __pycache__ "$REPO/" "$HOST:PX4-SITL-Runner/"

echo "[pils_setup] 2/4 sources -> $HOST:$RWS"
on "mkdir -p $RWS/ros2_ws/src $RWS/airsim $RWS/logs $RWS/test_runs $RWS/scripts"
rsync -a --delete --exclude .git --exclude __pycache__ --exclude /super/ --exclude '/T1000*.zip' \
    --exclude /super_pf_bridge/ "$WS/ros2_ws/src/" "$HOST:$RWS/ros2_ws/src/"
rsync -a --delete --exclude /.git --exclude /docs/ --exclude /Unreal/ --exclude /Matlab/ --exclude /PythonClient/ \
    --exclude /ros2/build/ --exclude /ros2/install/ --exclude /ros2/log/ "$WS/cosys-airsim/" "$HOST:$RWS/cosys-airsim/"
on "ln -sfn ../cosys-airsim/ros2/install $RWS/airsim/install"
if [ "${PILS_SKIP_BUILD:-0}" = 1 ]; then echo "[pils_setup] synced (PILS_SKIP_BUILD=1)"; exit 0; fi

echo "[pils_setup] 3/4 image $IMG (from $BASE)"
if ! on "docker image inspect $IMG > /dev/null 2>&1"; then
    # the ROS apt key was rotated in 2025: refresh it before apt
    on "mkdir -p /tmp/pils_image && cat > /tmp/pils_image/Dockerfile && docker build -q -t $IMG /tmp/pils_image" <<EOF
FROM $BASE
RUN (curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/ros.key \\
        && mv /tmp/ros.key /usr/share/keyrings/ros-archive-keyring.gpg || true) \\
 && apt-get update \\
 && apt-get install -y --no-install-recommends ros-humble-image-transport ros-humble-geographic-msgs \\
        ros-humble-mavros-msgs \\
 && rm -rf /var/lib/apt/lists/*
EOF
fi

echo "[pils_setup] 4/4 colcon builds in $IMG (log: $HOST:$RWS/logs/pils_build.log)"
# cloned here and copied over: GitHub from orin-nano-01 stalled (a clone hung for minutes)
if ! on "[ -d $RWS/cv_bridge_ws/src/vision_opencv/cv_bridge ]"; then
    T=$(mktemp -d)
    git clone -q --depth 1 -b humble https://github.com/ros-perception/vision_opencv.git "$T/vision_opencv"
    on "mkdir -p $RWS/cv_bridge_ws/src"
    rsync -a --exclude .git "$T/vision_opencv" "$HOST:$RWS/cv_bridge_ws/src/"
    rm -rf "$T"
fi
on "touch $RWS/cv_bridge_ws/src/vision_opencv/image_geometry/COLCON_IGNORE \
          $RWS/cv_bridge_ws/src/vision_opencv/opencv_tests/COLCON_IGNORE"
on "cat > $RWS/pils_build.sh" <<EOF
#!/bin/bash
# written by scripts/dev/pils_jetson_setup.sh; runs in $IMG with the workspace at $MNT
exec > $MNT/logs/pils_build.log 2>&1
set -e
# files made here are root's: keep the workspace writable for the next sync, also after a failure
trap 'chmod -R a+rwX $MNT' EXIT
source /opt/ros/humble/setup.bash
cd $MNT
T0=\$(date +%s)
echo "== cv_bridge (against the image's OpenCV)"
(cd cv_bridge_ws && MAKEFLAGS=-j4 colcon build --event-handlers console_cohesion+ \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release)
source cv_bridge_ws/install/setup.bash
echo "== AirSim bridge (Cosys-AirSim) \$(( \$(date +%s) - T0 )) s"
(cd cosys-airsim/ros2 && MAKEFLAGS=-j3 colcon build --symlink-install --event-handlers console_cohesion+ \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release)
echo "== ros2_ws apps \$(( \$(date +%s) - T0 )) s"
(cd ros2_ws && MAKEFLAGS=-j4 colcon build --symlink-install --parallel-workers 2 --event-handlers console_cohesion+ \
    --cmake-args -DBUILD_TESTING=OFF --packages-up-to flight_manager flight_driver pathfollowing realgazebo_fastplanner_adapters)
echo "PILS_BUILD_OK after \$(( \$(date +%s) - T0 )) s"
EOF
on "docker rm -f pils-build > /dev/null 2>&1; docker run -d --name pils-build --runtime nvidia \
    -v \$HOME/$RWS:$MNT --entrypoint bash $IMG $MNT/pils_build.sh > /dev/null"
code=$(on "docker wait pils-build")
on "tail -n 5 $RWS/logs/pils_build.log; docker rm pils-build > /dev/null"
[ "$code" = 0 ] && echo "[pils_setup] done - fly with CTRACK_MODE=pils ONBOARD_IP=${HOST#*@} ONBOARD_SSH=$HOST" \
    || { echo "[pils_setup] build failed (exit $code), see $HOST:$RWS/logs/pils_build.log"; exit 1; }
