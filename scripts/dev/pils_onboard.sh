#!/bin/bash
# PILS onboard side: run the ROS2 apps on the companion computer (Jetson) against the simulator PC.
# The onboard computer reaches the simulation only through two links, exactly as on the vehicle:
#   uXRCE-DDS UDP 8888  PX4 (ctrack-sitl on the PC) -> MicroXRCEAgent here
#   AirSim RPC 41451    the AirSim ROS2 bridge here -> Unreal on the PC (lidar, simulator pose)
# Everything else - agent, bridge, lidar adapter, the app (noclimb_ca, super_pf, ...), the flight
# recorder - runs here, in one container on the host network, from the same scripts as SILS
# (scripts/ros2/sitl-px4-airsim.sh launches scripts/ros2/apps/${ROS2_APP}.sh).
#
#   scripts/dev/pils_onboard.sh NAME          start (detached); the run lands in $ONBOARD_WS/test_runs/NAME
#   scripts/dev/pils_onboard.sh --stop NAME   stop the recorder, collect the logs, remove the container
#
# env:
#   AIRSIM_IP        the simulator PC (LAN 192.168.219.177 or Tailscale 100.64.0.119; 127.0.0.1 = loopback test on the PC)
#   ROS_DOMAIN_ID    must match the PC's ctrack-sitl (PX4 uses it as its DDS domain; default 0)
#   ONBOARD_WS       host dir mounted at /home/user/workspace/ros2: ros2_ws built ON THIS MACHINE (colcon keeps
#                    absolute paths, so it must have been built at that same container path), scripts/, logs/
#                    (default ~/Documents/A4VAI-SITL/ROS2)
#   ONBOARD_IMAGE    ROS2 image for this machine (default: jociiiii/a4vai:inhousesim-v0.9 on x86,
#                    a4vai-pils:humble on aarch64 - made, with the aarch64 builds, by scripts/dev/pils_jetson_setup.sh)
#   ROS2_APP, SITL_AIRFRAME, SIM_LIDAR, NOCLIMB_*, SUPER_PF_*, T1000_*  the app and its knobs, as in SILS;
#   NOCLIMB_GOAL "x,y" / NOCLIMB_ALT give the goal (PX4 local ENU, metres) - the PC does not write the
#   goal file here
set -u
STOP=0
[ "${1:-}" = "--stop" ] && { STOP=1; shift; }
NAME=${1:?usage: pils_onboard.sh [--stop] NAME}
WS=${ONBOARD_WS:-$HOME/Documents/A4VAI-SITL/ROS2}
IMAGE=${ONBOARD_IMAGE:-$([ "$(uname -m)" = aarch64 ] && echo a4vai-pils:humble || echo jociiiii/a4vai:inhousesim-v0.9)}
CMNT=/home/user/workspace/ros2/test_runs/$NAME
C=ros2-onboard

R=$WS/test_runs/$NAME
if [ $STOP = 1 ]; then
    [ -f "$R/.load_pids" ] && { kill $(cat "$R/.load_pids") 2>/dev/null; rm -f "$R/.load_pids"; }
    docker exec $C pkill -INT -f flight_recorder.py 2>/dev/null
    for i in $(seq 1 30); do [ -f "$WS/test_runs/$NAME/cloud_map.npz" ] && break; sleep 1; done
    docker exec $C bash -c "cp /home/user/workspace/ros2/logs/*.log /home/user/workspace/ros2/logs/*.pkl $CMNT/ 2>/dev/null; \
        cp /home/user/workspace/ros2/logs/noclimb_ca.csv /home/user/workspace/ros2/logs/noclimb_ca_dump.npz $CMNT/ 2>/dev/null; \
        f=\$(ls -t /home/user/workspace/ros2/logs/[0-9]*.csv 2>/dev/null | head -1); [ -n \"\$f\" ] && cp \"\$f\" $CMNT/pf_log.csv; \
        f=\$(ls -t /home/user/workspace/ros2/logs/flight_driver_*.csv 2>/dev/null | head -1); [ -n \"\$f\" ] && cp \"\$f\" $CMNT/flight_driver.csv; \
        chmod -R a+rw $CMNT"
    docker rm -f $C > /dev/null 2>&1
    echo "[pils_onboard] $NAME: logs in $WS/test_runs/$NAME"
    exit 0
fi

: ${AIRSIM_IP:?set AIRSIM_IP to the simulator PC}
: ${ROS2_APP:?set ROS2_APP}
mkdir -p "$WS/test_runs/$NAME" "$WS/logs" "$WS/scripts/include"
# the container runs this checkout's scripts, as run.sh deploys them for SILS
REPO=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
cp -r "$REPO"/scripts/ros2/* "$WS/scripts/" && cp -r "$REPO"/scripts/include/* "$WS/scripts/include/"
ENV=(-e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" -e AIRSIM_IP="$AIRSIM_IP" -e PX4_IP=127.0.0.1 -e ROS2_APP="$ROS2_APP"
     -e SITL_AIRFRAME="${SITL_AIRFRAME:-x500}")
for k in $(compgen -e | grep -E '^(NOCLIMB_|SUPER_PF_|T1000_|SIM_LIDAR$|PILS_LINK$|PILS_RELAY_PORT$)'); do ENV+=(-e "$k=${!k}"); done
# onnxruntime-gpu needs the GPU: --gpus on a PC, the NVIDIA runtime on a Jetson (L4T)
GPU=()
if docker info 2>/dev/null | grep -qi nvidia; then
    [ "$(uname -m)" = aarch64 ] && GPU=(--runtime nvidia) || GPU=(--gpus all)
fi
docker rm -f $C > /dev/null 2>&1
docker run -d --rm --name $C --hostname $C --network host "${GPU[@]}" --privileged "${ENV[@]}" \
    -v "$WS":/home/user/workspace/ros2 -w /home/user/workspace --entrypoint bash "$IMAGE" \
    /home/user/workspace/ros2/scripts/sitl-px4-airsim.sh > /dev/null || { echo "[pils_onboard] docker run failed"; exit 1; }
T0=$(date +%s)
until docker exec $C test -f /home/user/workspace/ros2/ros2_ws/install/setup.bash 2>/dev/null; do
    sleep 2; [ $(( $(date +%s) - T0 )) -gt 120 ] && { echo "[pils_onboard] container did not come up"; exit 1; }
done
docker exec -d $C bash -c "source /opt/ros/humble/setup.bash; source /home/user/workspace/ros2/ros2_ws/install/setup.bash; \
    python3 /home/user/workspace/ros2/scripts/tools/flight_recorder.py $CMNT > $CMNT/recorder.log 2>&1"
# the onboard computer's load over the run (does it keep up?): tegrastats on a Jetson, top on any host
rm -f "$R/.load_pids"
if command -v tegrastats > /dev/null; then
    nohup tegrastats --interval 1000 --logfile "$R/tegrastats.log" > /dev/null 2>&1 & echo $! >> "$R/.load_pids"
fi
nohup top -b -c -d 2 -w 400 > "$R/top.log" 2>&1 & echo $! >> "$R/.load_pids"
echo "[pils_onboard] $NAME: $ROS2_APP up (domain ${ROS_DOMAIN_ID:-0}, AirSim $AIRSIM_IP, image $IMAGE)"
