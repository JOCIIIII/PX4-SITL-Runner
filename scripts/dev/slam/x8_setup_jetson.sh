#!/bin/bash
# ON THE JETSON, once: X8 (and X500) configs for the SLAM container next to ~/slam_ws's packages, and the
# per-airframe knobs in its container script (scripts/dev/onboard_slam.sh uses both). Idempotent; the original
# ground_ev_inner.sh is kept as ground_ev_inner.sh.before_x8. X500 files = exactly what flew on 2026-10-31.
set -eu
cd "${SLAM_WS:-$HOME/slam_ws}"
mkdir -p config
FL=src/FAST_LIO/config/mid360_a4vai.yaml
BR=src/lio_px4_bridge/config/x500_mid360.yaml
cp -n $FL config/fast_lio_mid360_x500.yaml
cp -n $BR config/bridge_x500_mid360.yaml
{
    echo "# X8 (A4VAI, 2026-10-31): = fast_lio_mid360_x500.yaml (mid360_a4vai.yaml, the file that flew on the X500) with"
    echo "# blind 0.9 m: the X8's arms and props reach 0.77 m from the centre (scripts/dev/livox_bench_check.py), the X500's 0.55 m."
    sed -E 's/^(\s*blind:\s*)0\.6(.*)/\10.9\2   # X8: own returns to 0.77 m/' $FL
} > config/fast_lio_mid360_x8.yaml
{
    echo "# X8 (A4VAI, 2026-10-31): = bridge_x500_mid360.yaml (x500_mid360.yaml) with the X8's Mid-360 mount: 0.05 m above the"
    echo "# vehicle centre (10-17: Pixhawk 0.55 m, lidar 0.60 m above ground), connector to the nose (yaw 180). Roll / pitch are NOT"
    echo "# measured yet (0, 0): record a still run (EV_SENSOR=1), run onboard_slam.sh mount, and put the result here or in EV_RPY."
    sed -E 's/^(\s*lidar_xyz:\s*)\[[^]]*\](.*)/\1[0.0, 0.0, 0.05]\2/' $BR
} > config/bridge_x8_mid360.yaml
echo "== x8 vs x500 (fast_lio)"; diff <(grep -v '^#' config/fast_lio_mid360_x500.yaml) <(grep -v '^#' config/fast_lio_mid360_x8.yaml) || true
echo "== x8 vs x500 (bridge)";   diff <(grep -v '^#' config/bridge_x500_mid360.yaml) <(grep -v '^#' config/bridge_x8_mid360.yaml) || true

cp -n scripts/ground_ev_inner.sh scripts/ground_ev_inner.sh.before_x8
python3 - <<'EOF'
p = 'scripts/ground_ev_inner.sh'
s = open(p).read()
if 'EV_FASTLIO_CFG' in s:
    print('inner script already patched')
    raise SystemExit
pairs = [
    ('FL_ARGS=(--ros-args --params-file $FL/share/fast_lio/config/mid360_a4vai.yaml)',
     'FL_ARGS=(--ros-args --params-file ${EV_FASTLIO_CFG:-$FL/share/fast_lio/config/mid360_a4vai.yaml})   # EV_FASTLIO_CFG: per airframe (onboard_slam.sh)'),
    ('LIVOX_CFG=/home/user/workspace/slam_ws/pils/PX4-SITL-Runner/scripts/ros2/livox/MID360_a4vai.json',
     'LIVOX_CFG=/home/user/workspace/slam_ws/pils/PX4-SITL-Runner/scripts/ros2/livox/MID360_a4vai.json\n'
     'if [ -n "${LIVOX_LIDAR_IP:-}" ] && [ "${LIVOX_LIDAR_IP}" != 192.168.1.100 ]; then   # the X8\'s Mid-360 is 192.168.1.182 (onboard_slam.sh)\n'
     '    sed "s/\\"ip\\" : \\"192.168.1.100\\"/\\"ip\\" : \\"${LIVOX_LIDAR_IP}\\"/" $LIVOX_CFG > /tmp/MID360_slam.json && LIVOX_CFG=/tmp/MID360_slam.json\n'
     'fi'),
    ('--params-file $BR/share/lio_px4_bridge/config/x500_mid360.yaml',
     '--params-file ${EV_BRIDGE_CFG:-$BR/share/lio_px4_bridge/config/x500_mid360.yaml}'),
    ('"ROS_DOMAIN_ID=$ROS_DOMAIN_ID DDS=driver/fast_lio/sensor_bag default," \\',
     '"ROS_DOMAIN_ID=$ROS_DOMAIN_ID cfg=${EV_FASTLIO_CFG:-default}/${EV_BRIDGE_CFG:-default} lidar_ip=${LIVOX_LIDAR_IP:-file} DDS=driver/fast_lio/sensor_bag default," \\'),
]
for a, b in pairs:
    assert a in s, a
    s = s.replace(a, b)
open(p, 'w').write(s)
print('inner script patched')
EOF
bash -n scripts/ground_ev_inner.sh && echo "inner syntax ok"
echo "== inner diff"; diff scripts/ground_ev_inner.sh.before_x8 scripts/ground_ev_inner.sh | grep -E '^[<>]' || true
