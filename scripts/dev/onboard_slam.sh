#!/bin/bash
# ON THE VEHICLE'S JETSON: SLAM positioning for PX4 (no GNSS) next to the flight apps of onboard_real.sh - the
# slam-ev container of ~/slam_ws (FAST-LIO on the Mid-360 -> lio_px4_bridge -> /fmu/in/vehicle_visual_odometry),
# the way the X500 flew on 2026-10-31 (ground carry g1/g2 -> manual hover f1/f2 -> CA missions m1/m2). This is the
# launcher that used to live on another PC (ground_ev_jetson.sh); the container side is ~/slam_ws/scripts/ground_ev_inner.sh.
#
#   SLAM_AIRFRAME=x8|x500 scripts/dev/onboard_slam.sh start NAME ["what this run is"]
#                                      the container: Mid-360 driver (unless the flight app's ros2-real container is up:
#                                      then its /livox/* is shared), FAST-LIO, the bridge, PX4 / odometry recorders,
#                                      ev_watch (one line a second in runs/NAME/ev_watch.log)
#   scripts/dev/onboard_slam.sh origin LAT LON [ALT]   give PX4 a global origin (SLAM only: PX4 has none without GNSS,
#                                      so flight_driver waits and Hold does not arm). ALT is only logged; 0 keeps PX4's z
#   scripts/dev/onboard_slam.sh status                 the last ev_watch line: bridge state, FAST-LIO health, EKF aiding
#   scripts/dev/onboard_slam.sh stop                   stop the container (bags close), note the end in case.txt
#   scripts/dev/onboard_slam.sh mount NAME             the lidar mount roll / pitch from a still recording (mount_check.py):
#                                      EV_SEND=0 EV_SENSOR=1 ... start NAME, keep the vehicle still 40 s, stop, then this
# Records: ~/slam_ws/runs/NAME (case.txt, components.txt, bridge.csv, ev_watch.log, fast_lio.log, odom_bag, px4_bag)
#
# env: SLAM_AIRFRAME (required for start: x8 | x500 - picks the FAST-LIO and bridge configs and the lidar address),
#      EV_SEND (1 default: the bridge feeds PX4; 0 = shadow, PX4 gets nothing),
#      EV_RPY roll,pitch,yaw deg of the lidar mount (default: x500 the 10-31 measurement -1.6,-3.0,180.0; x8 the
#              config file's 2026-11-03 measurement -0.07,-2.28,0 - re-measure with 'mount' / lidar_mount_calib.py when the lidar moves),
#      EV_DRIVER (default 0 when ros2-real runs - the flight app's driver - else 1), EV_SENSOR (1 records /livox/lidar
#      + /livox/imu, ~5 MB/s; default 0), EV_PX4BAG (light default: the PX4 topics the post-flight look needs; full = every
#      /fmu/out topic, picked with EV_SENSOR=1 for mount_check; 0 = none - the full bag cost ~530 msg/s of CPU on 10-31),
#      EV_IPC_HOST (1 default: shared memory with ros2-real; 0 = UDP only),
#      ROS_DOMAIN_ID (default 0 = the vehicle's PX4), LIVOX_LIDAR_IP (default 192.168.1.100 for both: the X500's Mid-360 is on the X8 since 11-03; the X8's own unit is 192.168.1.182),
#      SLAM_WS (default ~/slam_ws), ONBOARD_WS (default ~/Documents/A4VAI-SITL/ROS2: px4_msgs + livox_ws installs),
#      SLAM_IMAGE (default a4vai-onboard:humble)
# PX4 parameters for SLAM only (QGC, then reboot): EKF2_EV_CTRL 15, EKF2_HGT_REF 3, EKF2_GPS_CTRL 0.
# Back to GNSS: EKF2_EV_CTRL 0, EKF2_GPS_CTRL 7, EKF2_HGT_REF as before. The bridge sends only while EV_SEND=1.
set -u
C=slam-ev
SLAM_WS=${SLAM_WS:-$HOME/slam_ws}
WS=${ONBOARD_WS:-$HOME/Documents/A4VAI-SITL/ROS2}
IMAGE=${SLAM_IMAGE:-a4vai-onboard:humble}
CS=/home/user/workspace/slam_ws               # the container's view of SLAM_WS
CW=/home/user/workspace/ros2                  # ... and of WS (where onboard_real.sh mounts it too)
RUNS=$SLAM_WS/runs
CUR=$RUNS/.current
SRC="source /opt/ros/humble/setup.bash; source $CW/ros2_ws/install/setup.bash"

[ -d "$SLAM_WS/scripts" ] && [ -f "$SLAM_WS/scripts/ground_ev_inner.sh" ] || { echo "[onboard_slam] no $SLAM_WS/scripts/ground_ev_inner.sh"; exit 1; }

case "${1:-}" in
start)
    NAME=${2:?usage: onboard_slam.sh start NAME [desc]}; DESC=${3:-}
    AF=${SLAM_AIRFRAME:?set SLAM_AIRFRAME=x8 or x500}
    case "$AF" in
        x8)   FL_CFG=$CS/config/fast_lio_mid360_x8.yaml; BR_CFG=$CS/config/bridge_x8_mid360.yaml
              IP=${LIVOX_LIDAR_IP:-192.168.1.100}; RPY=${EV_RPY:-} ;;
        x500) FL_CFG=$CS/config/fast_lio_mid360_x500.yaml; BR_CFG=$CS/config/bridge_x500_mid360.yaml
              IP=${LIVOX_LIDAR_IP:-192.168.1.100}; RPY=${EV_RPY:--1.6,-3.0,180.0} ;;
        *)    echo "[onboard_slam] SLAM_AIRFRAME=$AF: x8 or x500"; exit 1 ;;
    esac
    for f in "$SLAM_WS/config/$(basename $FL_CFG)" "$SLAM_WS/config/$(basename $BR_CFG)"; do
        [ -f "$f" ] || { echo "[onboard_slam] missing $f"; exit 1; }
    done
    if docker ps --format '{{.Names}}' | grep -qx $C; then
        echo "[onboard_slam] $C is running (run $(cat $CUR 2>/dev/null)) - stop it first"; exit 1
    fi
    [ -d "$RUNS/$NAME" ] && { echo "[onboard_slam] $RUNS/$NAME exists - another name"; exit 1; }
    if [ -z "${EV_DRIVER:-}" ]; then
        docker ps --format '{{.Names}}' | grep -qx ros2-real && EV_DRIVER=0 || EV_DRIVER=1
    fi
    SEND=${EV_SEND:-1}; SENSOR=${EV_SENSOR:-0}; IPC=${EV_IPC_HOST:-1}; DOM=${ROS_DOMAIN_ID:-0}
    mkdir -p "$RUNS/$NAME"
    {
        echo "case $NAME"; echo "desc ${DESC:-$AF SLAM}"; echo "start $(date '+%F %T')"; echo "airframe $AF"
        echo "send_to_px4 $SEND"; echo "lidar_rpy_deg ${RPY:-file}"; echo "driver $EV_DRIVER"; echo "ipc_host $IPC"
        echo "fastlio_cfg $(basename $FL_CFG)"; echo "bridge_cfg $(basename $BR_CFG)"; echo "lidar_ip $IP"
        echo "workspace $WS"
    } > "$RUNS/$NAME/case.txt"
    DDS=()
    if [ "$IPC" = 1 ]; then
        DDS=(--ipc host)
    else
        DDS=(-v "$SLAM_WS/fastdds_udp_only.xml":/fastdds_udp_only.xml:ro -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_udp_only.xml)
    fi
    docker run -d --rm --name $C --hostname $C --network host "${DDS[@]}" \
        -e ROS_DOMAIN_ID=$DOM -e EV_SEND=$SEND -e EV_SENSOR=$SENSOR -e EV_DRIVER=$EV_DRIVER ${RPY:+-e EV_RPY=$RPY} \
        -e EV_PX4BAG=${EV_PX4BAG:-$([ "$SENSOR" = 1 ] && echo full || echo light)} \
        -e EV_FASTLIO_CFG=$FL_CFG -e EV_BRIDGE_CFG=$BR_CFG -e LIVOX_LIDAR_IP=$IP \
        -v "$WS":$CW -v "$SLAM_WS":$CS -w $CS/runs/$NAME --entrypoint bash "$IMAGE" $CS/scripts/ground_ev_inner.sh \
        > /dev/null || { echo "[onboard_slam] docker run failed"; exit 1; }
    echo "$NAME" > $CUR
    echo "[onboard_slam] $NAME: $AF, send_to_px4 $SEND, mount rpy ${RPY:-file}, lidar driver $EV_DRIVER (ip $IP), domain $DOM"
    [ "$AF" = x8 ] && [ -z "$RPY" ] && echo "[onboard_slam] X8 mount roll/pitch not measured yet (0,0,180 from the config) - 'mount' first for SLAM height"
    [ "$SEND" = 1 ] && echo "[onboard_slam] PX4 must be set for SLAM (EKF2_EV_CTRL 15, EKF2_HGT_REF 3, EKF2_GPS_CTRL 0); watch: onboard_slam.sh status"
    ;;
origin)
    LAT=${2:?usage: onboard_slam.sh origin LAT LON [ALT]}; LON=${3:?LON}; ALT=${4:-0.0}
    docker ps --format '{{.Names}}' | grep -qx $C || { echo "[onboard_slam] $C is not running"; exit 1; }
    NAME=$(cat $CUR 2>/dev/null)
    if docker exec $C bash -c "$SRC; python3 $CS/scripts/px4_set_origin.py $LAT $LON $ALT"; then
        echo "set-origin $LAT,$LON,$ALT $(date '+%F %T')" >> "$RUNS/$NAME/case.txt"
        echo "[onboard_slam] origin set"
    else
        echo "set-origin FAILED $LAT,$LON,$ALT $(date '+%F %T')" >> "$RUNS/$NAME/case.txt"
        echo "[onboard_slam] origin NOT set (PX4 topics missing or z moved) - see above"; exit 1
    fi ;;
status)
    NAME=$(cat $CUR 2>/dev/null) || { echo "[onboard_slam] no run yet"; exit 1; }
    docker ps --format '{{.Names}}' | grep -qx $C && echo "[onboard_slam] $C up: run $NAME" || echo "[onboard_slam] $C down (last run $NAME)"
    grep -E 'EVENT' "$RUNS/$NAME/ev_watch.log" 2>/dev/null | tail -3
    tail -1 "$RUNS/$NAME/ev_watch.log" 2>/dev/null
    tail -1 "$RUNS/$NAME/fast_lio_watch.log" 2>/dev/null ;;
stop)
    NAME=$(cat $CUR 2>/dev/null)
    if docker ps --format '{{.Names}}' | grep -qx $C; then
        [ -n "$NAME" ] && echo "end $(date '+%F %T')" >> "$RUNS/$NAME/case.txt"
        docker stop -t 25 $C > /dev/null
        echo "[onboard_slam] stopped ($NAME); records in $RUNS/$NAME"
    else
        echo "[onboard_slam] $C is not running"
    fi
    [ -n "$NAME" ] && [ -d "$RUNS/$NAME" ] && docker run --rm -v "$SLAM_WS":$CS "$IMAGE" chmod -R a+rw $CS/runs/$NAME ;;
mount)
    NAME=${2:?usage: onboard_slam.sh mount NAME (a run recorded with EV_SENSOR=1, vehicle still)}
    [ -d "$RUNS/$NAME/sensor_bag" ] || { echo "[onboard_slam] $RUNS/$NAME has no sensor_bag - record with EV_SENSOR=1"; exit 1; }
    docker run --rm --network none -v "$WS":$CW -v "$SLAM_WS":$CS -w $CS/runs "$IMAGE" \
        bash -c "$SRC; python3 $CS/scripts/mount_check.py $CS/runs/$NAME" ;;
*)
    sed -n 2,30p "$0"; exit 1 ;;
esac
