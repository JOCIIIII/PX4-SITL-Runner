#!/bin/bash
# ground_ev_inner.sh - SLAM -> PX4 지상 시험(프롭 없이)과 그 뒤 SLAM 비행(10-31)에서 ground_ev_jetson.sh가 젯슨에 띄운
# 임시 컨테이너 slam-ev 안에서 돌림. 기록은 지금 폴더(~/slam_ws/runs/<이름>)에.
# ROS 도메인 0(PX4 uxrce-agent와 같은 도메인). Fast DDS는 둘로 나눔:
#   - 라이다 드라이버·FAST-LIO·센서 녹화: 기본 전송(이 컨테이너 안 공유 메모리, 10-28 dt_fix·10-30과 같음). 컨테이너 전체를
#     UDP만으로 했더니 점군(약 0.5 MB, 10 Hz)이 빠지거나 밀려 FAST-LIO 출력이 0.3-3.4 s씩 끊김(10-31 g1_ground, 2 s에 10-20 스캔)
#   - 브리지·ev_watch·녹화기(odom_bag, px4_bag): UDP만(바깥에서 준 FASTRTPS_DEFAULT_PROFILES_FILE). 공유 메모리로는 다른
#     컨테이너인 agent와 데이터가 안 오감(10-28). 둘 사이의 작은 메시지(/Odometry, /fast_lio/health, /livox/imu)는 UDP로 만남
#   - EV_IPC_HOST=1(바깥에서 --ipc host, 프로필 없음): 모두 기본 전송. 같은 호스트 IPC의 ros2-real과 공유 메모리로,
#     UDP만인 agent와는 UDP로 만남. EV_DRIVER=0이면 드라이버를 안 띄우고 ros2-real의 /livox/*를 받음(10-31, 탁상 시험 전)
#   Mid-360 드라이버(기체와 같은 설정) -> FAST-LIO(mid360_a4vai.yaml) -> lio_px4_bridge(x500_mid360.yaml)
#   EV_SEND=1일 때만 브리지가 /fmu/in/vehicle_visual_odometry로 보냄(기본 0 = 그림자, ~/vehicle_odometry). PX4가 실제로
#     쓰는지는 FC 파라미터 EKF2_EV_CTRL에 달림(0이면 받기만 함). FC 파라미터는 여기서 안 바꿈
#   EV_RPY="r,p,y": 브리지 장착각 lidar_rpy_deg를 이 실행에서만 덮어씀(설정 파일은 그대로). 비우면 설정 파일 값
#   녹화: odom_bag(FAST-LIO, 브리지 상태·출력, PX4로 보낸 것), px4_bag(/fmu/out/*), EV_SENSOR=1(기본)이면 sensor_bag
#     (라이다·IMU 원본, 약 5 MB/s). 요약: ev_watch.log(1초마다 + 바뀐 순간 EVENT), fast_lio_watch.log, bridge.csv
#   'ground_ev_inner.sh fastlio': FAST-LIO만 다시 켬(SLAM 끊김 시험 뒤 docker exec로, 로그 fast_lio_<시각>.log)
# SIGINT/SIGTERM을 받으면 모두 끄고 끝남
source /opt/ros/humble/setup.bash
source /home/user/workspace/ros2/ros2_ws/install/setup.bash        # px4_msgs (bridge, ev_watch)
source /home/user/workspace/ros2/livox_ws/install/setup.bash
source /home/user/workspace/slam_ws/install/setup.bash
FL=$(ros2 pkg prefix fast_lio)
FL_ARGS=(--ros-args --params-file ${EV_FASTLIO_CFG:-$FL/share/fast_lio/config/mid360_a4vai.yaml})   # EV_FASTLIO_CFG: per airframe (onboard_slam.sh)
LOCAL=(env -u FASTRTPS_DEFAULT_PROFILES_FILE)      # 기본 전송(공유 메모리 + UDP): 점군을 주고받는 것만
if [ "${1:-}" = fastlio ]; then
    echo "fastlio-restart $(date '+%F %T')" >> case.txt
    exec "${LOCAL[@]}" $FL/lib/fast_lio/fastlio_mapping "${FL_ARGS[@]}" > fast_lio_$(date +%H%M%S).log 2>&1
fi

SEND=false
[ "${EV_SEND:-0}" = 1 ] && SEND=true
RPY_ARGS=()
[ -n "${EV_RPY:-}" ] && RPY_ARGS=(-p "lidar_rpy_deg:=[${EV_RPY}]")
LIVOX_CFG=/home/user/workspace/slam_ws/pils/PX4-SITL-Runner/scripts/ros2/livox/MID360_a4vai.json
if [ -n "${LIVOX_LIDAR_IP:-}" ] && [ "${LIVOX_LIDAR_IP}" != 192.168.1.100 ]; then   # the X8's Mid-360 is 192.168.1.182 (onboard_slam.sh)
    sed "s/\"ip\" : \"192.168.1.100\"/\"ip\" : \"${LIVOX_LIDAR_IP}\"/" $LIVOX_CFG > /tmp/MID360_slam.json && LIVOX_CFG=/tmp/MID360_slam.json
fi
echo "EV_SEND=${EV_SEND:-0} EV_RPY=${EV_RPY:-file} EV_SENSOR=${EV_SENSOR:-1} EV_DRIVER=${EV_DRIVER:-1}" \
    "ROS_DOMAIN_ID=$ROS_DOMAIN_ID cfg=${EV_FASTLIO_CFG:-default}/${EV_BRIDGE_CFG:-default} lidar_ip=${LIVOX_LIDAR_IP:-file} DDS=driver/fast_lio/sensor_bag default," \
    "bridge/watch/recorders ${FASTRTPS_DEFAULT_PROFILES_FILE:+udp_only}${FASTRTPS_DEFAULT_PROFILES_FILE:-default (ipc host)}" \
    > components.txt

PIDS=""
if [ "${EV_DRIVER:-1}" = 1 ]; then
    "${LOCAL[@]}" ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
        -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 -p output_data_type:=0 \
        -p frame_id:=livox_frame -p user_config_path:=$LIVOX_CFG \
        -p "cmdline_input_bd_code:=livox0000000001" > livox.log 2>&1 &
    PIDS="$!"
else
    echo "no driver: /livox/lidar, /livox/imu from another container (ros2-real)" > livox.log
fi
"${LOCAL[@]}" $FL/lib/fast_lio/fastlio_mapping "${FL_ARGS[@]}" > fast_lio.log 2>&1 &
PIDS="$! $PIDS"
BR=$(ros2 pkg prefix lio_px4_bridge)
$BR/lib/lio_px4_bridge/bridge --ros-args --params-file ${EV_BRIDGE_CFG:-$BR/share/lio_px4_bridge/config/x500_mid360.yaml} \
    -p send_to_px4:=$SEND -p log_csv:=bridge.csv "${RPY_ARGS[@]}" > bridge.log 2>&1 &
PIDS="$! $PIDS"
ros2 bag record -o odom_bag /Odometry /fast_lio/health /lio_px4_bridge/status /lio_px4_bridge/vehicle_odometry \
    /fmu/in/vehicle_visual_odometry > record.log 2>&1 &
PIDS="$! $PIDS"
# EV_PX4BAG (10-31 evening, CPU): light (default) = what the post-flight look needs, ~200 msg/s; full = every /fmu/out
# topic (~530 msg/s: sensor_combined, vehicle_odometry, manual_control_setpoint ... - mount_check.py needs
# sensor_combined, so onboard_slam.sh picks full with EV_SENSOR=1); 0 = no PX4 bag
case "${EV_PX4BAG:-light}" in
    full)  ros2 bag record -e "/fmu/out/.*" -o px4_bag > px4_record.log 2>&1 & PIDS="$! $PIDS" ;;
    0)     echo "no px4_bag (EV_PX4BAG=0)" > px4_record.log ;;
    *)     ros2 bag record -o px4_bag /fmu/out/vehicle_local_position /fmu/out/vehicle_attitude /fmu/out/estimator_status_flags \
               /fmu/out/failsafe_flags /fmu/out/vehicle_status_v1 /fmu/out/vehicle_gps_position /fmu/out/battery_status \
               /fmu/out/vehicle_land_detected /fmu/out/vehicle_global_position > px4_record.log 2>&1 & PIDS="$! $PIDS" ;;
esac
if [ "${EV_SENSOR:-1}" = 1 ]; then
    "${LOCAL[@]}" ros2 bag record -o sensor_bag /livox/lidar /livox/imu > sensor_record.log 2>&1 &
    PIDS="$! $PIDS"
fi
python3 /home/user/workspace/slam_ws/scripts/watch_fast_lio.py > fast_lio_watch.log 2>&1 &
PIDS="$! $PIDS"
python3 /home/user/workspace/slam_ws/scripts/ev_watch.py > ev_watch.log 2>&1 &
PIDS="$! $PIDS"

stop() {
    kill -INT $PIDS 2>/dev/null
    sleep 3
    kill -9 $PIDS 2>/dev/null
    exit 0
}
trap stop INT TERM
echo "[slam-ev] up (send_to_px4 $SEND, rpy ${EV_RPY:-file}): $PIDS"
wait
