#! /bin/bash

# ROS2 APP: no-climb learned collision avoidance (CA) -> PathFollowing (PF) on AirSim lidar.
#   px4_lidar_to_odom_cloud (AirSim lidar + PX4 pose -> /odom + /cloud, world ENU)
#   -> noclimb_ca_node.py (ca_policy_ref.NoClimbPolicy at 20 Hz: WP 3 m ahead + speed
#      -> /local_waypoint_setpoint_to_PF, /desired_speed, PP/CA heartbeats)
#   -> pathfollowing node_pathfollowing (guid_type 0 speed mode -> /pf_att_2_control)
#   -> flight_manager (arm/offboard/take-off FSM, forwards PF attitude, lands on PF done)
# Policy: ros2_ws/src/Inhouse-collision-avoidance/noclimb_onnx (ONNX + reference code, run
# in place). The global path is a straight line from where CA engages to the goal.
#
# knobs (env): NOCLIMB_POLICY (ca_noclimb_tau05-12 | ca_noclimb_tau03-08, default tau05-12:
# agent.md's safe choice when the follower's response time is not known), NOCLIMB_GOAL "x,y"
# and NOCLIMB_ALT (default: first row of realgazebo_fastplanner_adapters/config/
# goals_super_far.csv, shared with super_pf / t1000), NOCLIMB_DENSITY (cluster | constant in
# [0, 1], default cluster), NOCLIMB_SCAN_MEMORY (seconds of scans in the image, default 0 =
# latest scan only, as the policy was trained; with NOCLIMB_SCAN_MEMORY_MIN the last N scans stay
# whatever their age, up to NOCLIMB_SCAN_MEMORY_MAX s - default 1.5 - so a scan gap cannot wipe
# the memory; C-Track 0.5 m poles: 0.4 s kept soft at >= +1.10 m), NOCLIMB_GUID (0 pursuit | 2 MPPI, see below),
# NOCLIMB_CMD_TAU (first-order smoothing of the policy's command [s], default 0 = off),
# NOCLIMB_CMD_ACCEL_MAX / NOCLIMB_CMD_JERK_MAX (caps on the smoothed command's acceleration [m/s^2] and
# its rate [m/s^3] - tilt ~ atan(a/g), tilt rate ~ jerk/g; default 0 = off; speed decreases bypass them),
# NOCLIMB_CMD_FADE_NEAR / _FAR [m] (obstacle band, 0 = off: the caps act only beyond _FAR from the nearest return,
# none inside _NEAR; NOCLIMB_CMD_TAU_NEAR [s] - default -1 = NOCLIMB_CMD_TAU everywhere - is the smoothing inside
# _NEAR, blending to NOCLIMB_CMD_TAU at _FAR),
# NOCLIMB_CONTACT_DIST [m] (a return closer than this at the vehicle's height for NOCLIMB_CONTACT_TIME s - default
# 1.0 - lands in place; default the prop tips: x8 0.77, x500 0.39, iris 0.35; 0 = off),
# NOCLIMB_GOAL_BRAKE (deceleration of the goal speed cap [m/s^2], default 1.5; the cap aims for
# 1 m/s at flight_manager's handover radius SUPER_PF_GOAL_HANDOVER_M - with the follower's lag
# the vehicle arrives there at ~3 m/s from 5 m/s, which PX4 stops gently), NOCLIMB_WP_Z_RATE
# (m/s the WP height sent to PF may move toward the policy's; default -1 = the bundle's
# ctrl_vz_max (1.5) when it may climb (ca_big), off for the no-climb bundles; 0 = off). Per-step log:
# logs/noclimb_ca.csv; replay dump (scans + policy steps, written at disarm):
# logs/noclimb_ca_dump.npz.
# On the vehicle (scripts/ros2/onboard-real.sh): NOCLIMB_LIDAR=livox (default sim = AirSim) takes the Mid-360
# from livox_ros_driver2 (/livox/lidar, per-point times, registered with the PX4 pose; NOCLIMB_LIDAR_LATENCY [s],
# default 0.02, frame end to receipt), and NOCLIMB_SHADOW=1 runs the chain without flying: no PathFollowing,
# no flight_manager, the node engages unarmed with the goal NOCLIMB_SHADOW_AHEAD m (default 30) ahead of the
# nose and publishes only under /shadow/... (+ /noclimb/path for Foxglove; Bool /shadow/reset = new run);
# NOCLIMB_SHADOW_MASK [m] (default 1.5) leaves out the returns behind the nose that close - the carrier.
# With livox the heights and NOCLIMB_GOAL are taken from where the vehicle rests when the app starts (on the
# ground): PX4's local origin is not there on the vehicle (13-45 m off in z in the X500's logs).
# NOCLIMB_VOXEL [m] (livox default 0.1, sim 0 = off): the nearest return per voxel - the real lidar's density.
# Lidar mount (livox, measured on the vehicle - scripts/dev/lidar_mount_calib.py): NOCLIMB_LIDAR_YAW_DEG (default 180),
# NOCLIMB_LIDAR_ROLL_DEG / NOCLIMB_LIDAR_PITCH_DEG (default 0), NOCLIMB_LIDAR_X / _Y (forward / left of the centre, 0)
# and NOCLIMB_LIDAR_Z (above the centre; default the bundle's lidar_z_offset).
# NOCLIMB_DRIVER: flight_manager (default, SITL) | flight_driver (the vehicle's flight FSM, also in SILS / PILS:
# AUTO_TAKEOFF relative to the take-off point, OFFBOARD in the air, yields to the pilot, lands on lost inputs;
# ros2_ws/src/flight_driver). With it: NOCLIMB_GOAL is relative to the take-off point (SITL: the spawn = PX4's
# origin, the same numbers), NOCLIMB_GOAL_GPS "lat,lon" instead, NOCLIMB_REQUIRE_GO=1 arms only on the operator's
# Bool on /flight_driver/go, NOCLIMB_EKF_SETTLE [s] (livox 30, sim 5) of an unreset estimate before arming,
# NOCLIMB_SPEED_LIMIT [m/s] (default the bundle's v_max + 2.5) lands in place above it.
# PathFollowing on the vehicle: NOCLIMB_HOVER_THRUST (the measured normalised hover thrust; sets the thrust scale of
# both PF nodes - 0 / unset = the yaml) and NOCLIMB_MAX_TILT_DEG (0 / unset = the yaml's 35).
# NOCLIMB_PF_DIRECT=1: PathFollowing alone flies the straight line to the goal once it is clear (noclimb_ca_node.py
# pf_direct: nothing within NOCLIMB_PF_DIRECT_CORRIDOR m - default 1.8 - of it for NOCLIMB_PF_DIRECT_ON m - default 25 -
# or up to the goal, for 1 s, and the velocity within NOCLIMB_PF_DIRECT_ALIGN deg - default 2.5 - of it; back to the
# policy at a return within NOCLIMB_PF_DIRECT_OFF m - default 18); default 0 = the policy everywhere.
# Flight profiles (the same knobs for the PILS gate and the vehicle): scripts/dev/flight_profiles/*.env.

# WORKSPACE_DIR is exported by the launcher; fall back if run standalone.
if [ -z "${WORKSPACE_DIR}" ]; then
    WORKSPACE_DIR=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
fi

ROS2_WS=${WORKSPACE_DIR}/ros2_ws
LOGS=${WORKSPACE_DIR}/logs
APP_DIR=$(dirname $(readlink -f "$0"))
# NOCLIMB_POLICY_DIR (optional): another bundle of the same interface (216 in, 3 out, the same
# ca_policy_ref.py), e.g. ca_big_onnx (1 m class, climbing allowed: WP up to +6 m) with
# NOCLIMB_POLICY=ca_big, or ca_small_onnx / ca_small
POLICY_DIR=${ROS2_WS}/src/Inhouse-collision-avoidance/${NOCLIMB_POLICY_DIR:-noclimb_onnx}
POLICY=${NOCLIMB_POLICY:-ca_noclimb_tau05-12}
GOALS_CSV=${ROS2_WS}/src/realgazebo_fastplanner_adapters/config/goals_super_far.csv
# lidar mount above the vehicle centre [m] (the bundle's cfg lidar_z_offset; the Unreal settings must match:
# X8 0.3, X500 0.1 - scripts/dev/unreal_settings/settings_gazebo_mid360_x500.json)
LIDAR_Z=${NOCLIMB_LIDAR_Z:-$(python3 -c "import json; print(json.load(open('${POLICY_DIR}/cfg.json'))['cfg'].get('lidar_z_offset', 0.3))")}

for f in ${POLICY}.onnx cfg.json ca_policy_ref.py; do
    if [ ! -f "${POLICY_DIR}/${f}" ]; then
        echo "[$(basename "$0")] ERROR: ${POLICY_DIR}/${f} not found."
        exit 1
    fi
done
DRIVER=${NOCLIMB_DRIVER:-flight_manager}
for pkg in pathfollowing ${DRIVER} realgazebo_fastplanner_adapters custom_msgs px4_msgs; do
    if [ ! -d "${ROS2_WS}/install/${pkg}" ]; then
        echo "[$(basename "$0")] ERROR: ${pkg} is not built."
        echo "[$(basename "$0")] Build first: ./scripts/run.sh ros2 build ros2_ws"
        exit 1
    fi
done

source ${ROS2_WS}/install/setup.bash
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 NUMEXPR_NUM_THREADS=2

# Lidar: the simulator's (AirSim pose registration, de-skew by azimuth or shot index) or the vehicle's
# Mid-360 (each point's own time, PX4 pose)
if [ "${NOCLIMB_LIDAR:-sim}" = livox ]; then
    LIDAR_TOPIC=/livox/lidar
    # NOCLIMB_SELF_RADIUS [m] (default 0.55): returns this close to the vehicle centre (horizontally, +-0.5 m in
    # height) are the vehicle's own parts - the X500's Mid-360 saw ~2000 a frame within 0.5 m on the bench
    ADAPTER_SRC="-p cloud_latency_s:=$(printf '%.3f' ${NOCLIMB_LIDAR_LATENCY:-0.02}) -p deskew_mode:=field \
        -p self_radius_m:=$(printf '%.2f' ${NOCLIMB_SELF_RADIUS:-0.55})"
    # NOCLIMB_LIDAR_YAW_DEG: the Mid-360's +X (away from its connector) relative to the nose; the X500's sits
    # with the connector to the front, i.e. looking back: 180 (2026-10-15)
    LIDAR_YAW=$(python3 -c "import math; print(f'{math.radians(${NOCLIMB_LIDAR_YAW_DEG:-180}):.6f}')")
    LIDAR_ROLL=$(python3 -c "import math; print(f'{math.radians(${NOCLIMB_LIDAR_ROLL_DEG:-0}):.6f}')")
    LIDAR_PITCH=$(python3 -c "import math; print(f'{math.radians(${NOCLIMB_LIDAR_PITCH_DEG:-0}):.6f}')")
    LIDAR_X=$(printf '%.3f' ${NOCLIMB_LIDAR_X:-0}); LIDAR_Y=$(printf '%.3f' ${NOCLIMB_LIDAR_Y:-0})
else
    LIDAR_TOPIC=/airsim_node/SimpleFlight/lidar/points/RPLIDAR_A3
    LIDAR_YAW=0.0; LIDAR_ROLL=0.0; LIDAR_PITCH=0.0; LIDAR_X=0.0; LIDAR_Y=0.0
    ADAPTER_SRC="-p cloud_latency_s:=0.13 -p airsim_odom_topic:=/airsim_node/SimpleFlight/odom_local \
        -p deskew_mode:=$([ "${SIM_LIDAR:-grid}" = mid360 ] && echo index || echo azimuth)"
fi
SHADOW=${NOCLIMB_SHADOW:-0}
[ "${SHADOW}" = 1 ] && echo "[$(basename "$0")] SHADOW: no PathFollowing, no flight_manager - the policy's commands go to /shadow/..."

# Do not fly blind: wait for real scans (simulator / lidar driver up) before anything can arm. The topic
# has to exist first: echo gives up at once on an unknown topic (the Livox driver comes up after the app)
echo "[$(basename "$0")] waiting for lidar data on ${LIDAR_TOPIC} ..."
T_LIDAR=$(date +%s)
until ros2 topic list 2>/dev/null | grep -qx "${LIDAR_TOPIC}" \
        && timeout 10 ros2 topic echo --once --qos-profile sensor_data --field header.stamp \
            ${LIDAR_TOPIC} > /dev/null 2>&1; do
    if [ $(( $(date +%s) - T_LIDAR )) -gt 300 ]; then
        echo "[$(basename "$0")] ERROR: no lidar data within 300 s (simulator / lidar driver not running?) - not taking off."
        exit 1
    fi
    sleep 1
done

ROW=$(sed -n 2p ${GOALS_CSV} | tr -d ' ')
GOAL=${NOCLIMB_GOAL:-$(echo ${ROW} | cut -d, -f1,2)}
# ROS double parameters reject integer literals ("4") - always pass x.xxx
GOAL_X=$(printf '%.3f' $(echo ${GOAL} | cut -d, -f1))
GOAL_Y=$(printf '%.3f' $(echo ${GOAL} | cut -d, -f2))
ALT=$(printf '%.3f' ${NOCLIMB_ALT:-$(echo ${ROW} | cut -d, -f3)})
echo "[$(basename "$0")] ${POLICY}: path from the engage point -> (${GOAL_X}, ${GOAL_Y}) at ${ALT} m"

# 1) PX4 pose -> /odom (ENU); AirSim lidar -> /cloud (world ENU, AirSim-registered, de-skewed)
#    NOCLIMB_MAX_TILT_RATE [rad/s] (default 0 = off, as flown in the forest): scans taken while
#    the body tilts faster are dropped (super_pf uses 0.8: near the ground, as on C-Track at
#    1.5 m, a scan registered with the lagging attitude puts the ground up as a phantom wall)
ros2 run realgazebo_fastplanner_adapters px4_lidar_to_odom_cloud --ros-args \
    -p "px4_ns:=''" \
    -p in_topic:=${LIDAR_TOPIC} \
    -p use_ros_time:=true ${ADAPTER_SRC} \
    -p synth_free_rays:=false -p deskew_rot_period_s:=0.1 \
    -p max_tilt_rate:=$(printf '%.3f' ${NOCLIMB_MAX_TILT_RATE:-0}) -p publish_empty_scans:=true \
    -p lidar_offset_x:=${LIDAR_X} -p lidar_offset_y:=${LIDAR_Y} -p lidar_offset_z:=${LIDAR_Z} \
    -p lidar_roll:=${LIDAR_ROLL} -p lidar_pitch:=${LIDAR_PITCH} -p lidar_yaw:=${LIDAR_YAW} \
    2>&1 | tee ${LOGS}/px4_lidar_to_odom_cloud.log &

# 2) PathFollowing, speed-regulated pursuit (guid 0 needs no MPPI node). vertical_priority:
#    the policy turns hard (up to a reversal at 5 m/s) and was trained with a follower whose
#    altitude hold is independent of the horizontal manoeuvre; PF's default thrust law dropped
#    1.2 m in such a reversal (nc_c).
# Per-airframe version, from the simulated airframe (envs/gazebo-classic.env SITL_AIRFRAME,
# copied into ros2.env by run.sh):
#   iris -> PF quad.yaml (1), MPPI config sim_ca_mppi.yaml, range image as sensed
#   x8   -> PF octo.yaml (2), MPPI config sim_ca_mppi_x8.yaml, range image moved 0.8 m closer
#           (prop tips 0.77 m vs the ~0.35 m class the policy was trained for, d_crash 0.6 m:
#           x8_nc_b passed 0.10 m from the canopy, 0.45 m still let the 10 kg X8 slide into it
#           when the policy flipped sides at the tree gap), path extension 5 (see below),
#           safety velocity filter at 1.2 m (noclimb_ca_node.py safety_radius_m: the policy
#           still crept into the canopy at its 0.6 m/s floor where the gap is too narrow),
#           policy speed decreases not smoothed (cmd_brake_immediate)
# Path extension: the PF path is [P0, WP, WP + k(WP-P0), WP + 2k(WP-P0)]. MPPI's 2 s rollouts
# switch to the last point 4.5 m (waypoint_switch_distance) before the third one and from there
# fly the position-hold law, i.e. brake: k = 2 (third point 9 m out) penalised speed and held the
# X8 at 4.1 m/s; k = 5 keeps the rollouts on MPPI guidance (the real PF never gets there: the
# path is re-sent every 50 ms).
# NOCLIMB_RANGE_INFLATE / NOCLIMB_GOAL_BRAKE / NOCLIMB_PATH_EXT / NOCLIMB_SAFETY_R override the
# per-airframe defaults.
# A bundle trained for the X8 (cfg.json d_crash >= 0.9, e.g. x8_onnx_r2: 1.0 m) already keeps its
# distance and was trained with the reference governor and its own height/ground conventions:
# no range-image pull, safety filter at ~0.9 m, the 360 deg governor as trained (its agent.md 0, 7).
D_CRASH=$(python3 -c "import json; m = json.load(open('${POLICY_DIR}/cfg.json')); c = dict(m['cfg']); \
c.update(m['policies'].get('${POLICY}', {}).get('overrides', {})); print(c.get('d_crash', 0.6))")
X8_TRAINED=$(python3 -c "print(1 if ${D_CRASH} >= 0.9 else 0)")
if [ "${SITL_AIRFRAME}" = "x8" ]; then
    PF_VEHICLE_TYPE=2; MPPI_YAML=sim_ca_mppi_x8.yaml
    RANGE_INFLATE=${NOCLIMB_RANGE_INFLATE:-$([ ${X8_TRAINED} = 1 ] && echo 0 || echo 0.8)}; GOAL_BRAKE=${NOCLIMB_GOAL_BRAKE:-1.5}
    PATH_EXT=${NOCLIMB_PATH_EXT:-5}
    SAFETY_R=${NOCLIMB_SAFETY_R:-$([ ${X8_TRAINED} = 1 ] && echo 0.9 || echo 1.2)}  # prop tips 0.77 m + 0.4 m (1.0: own-map minimum 0.81 m in x8_nc_mppi_e); X8-trained: 0.9
    GROUND_Z=${NOCLIMB_GROUND_Z:--0.44}   # ground in PX4's local frame (origin = the X8's centre at rest, 0.44 m up):
    AGL_OFF=${NOCLIMB_AGL_OFFSET:-0.44}   # returns below 0.25 m above it leave the image; the policy gets heights above ground
    SAFETY_K=${NOCLIMB_SAFETY_K:-2.0}     # 1/s (1: 5 m/s held to 2.6 m/s past trees 2 m away; 3: braked too late)
    BRAKE_NOW=true                        # policy speed decreases bypass NOCLIMB_CMD_TAU
    GOV_SECTOR=${NOCLIMB_GOV_SECTOR:-$([ ${X8_TRAINED} = 1 ] && echo 180 || echo 90)}  # front half only (behind: passed poles kept it crawling); X8-trained: as trained
    GOV_CORRIDOR=${NOCLIMB_GOV_CORRIDOR:-0}  # > 0: only returns in the path (+-m of the command / velocity line), with
    GOV_BRAKE=${NOCLIMB_GOV_BRAKE:-0}     # GOV_BRAKE > 0 capped by braking distance [m/s^2]. Both off: the policy heads
                                          # for a pole until ~2 m and passes it 0.6-1 m off its line - fine for the
                                          # small vehicle it learned on, not for the X8 - so only the governor's early
                                          # braking keeps the margin: corridor 1.2 put C5 0.84 m from the X8's centre
                                          # (ctrack_big_k), corridor + braking law 0.80 m at pole height (big_l)
elif [ "${SITL_AIRFRAME}" = "x500" ]; then
    # the X500 test quad (scripts/px4/x500) flies the same bundles until it has its own: the X8's
    # height/ground conventions with its own rest height (centre 0.23 m up), PathFollowing as a quad.
    # Its prop tips are at 0.39 m, so the 1 m class bundles need no range-image pull either.
    PF_VEHICLE_TYPE=1; MPPI_YAML=sim_ca_mppi.yaml
    RANGE_INFLATE=${NOCLIMB_RANGE_INFLATE:-0}; GOAL_BRAKE=${NOCLIMB_GOAL_BRAKE:-1.5}
    PATH_EXT=${NOCLIMB_PATH_EXT:-2}
    # x500_onnx (d_crash 0.6) recommends a ~0.5 m safety filter (its agent.md 3)
    SAFETY_R=${NOCLIMB_SAFETY_R:-$([ ${X8_TRAINED} = 1 ] && echo 0.9 || echo 0.5)}
    GROUND_Z=${NOCLIMB_GROUND_Z:--0.23}; AGL_OFF=${NOCLIMB_AGL_OFFSET:-0.23}
    SAFETY_K=${NOCLIMB_SAFETY_K:-2.0}; BRAKE_NOW=true
    GOV_SECTOR=${NOCLIMB_GOV_SECTOR:-180}; GOV_CORRIDOR=${NOCLIMB_GOV_CORRIDOR:-0}; GOV_BRAKE=${NOCLIMB_GOV_BRAKE:-0}
else
    PF_VEHICLE_TYPE=1; MPPI_YAML=sim_ca_mppi.yaml
    RANGE_INFLATE=${NOCLIMB_RANGE_INFLATE:-0}; GOAL_BRAKE=${NOCLIMB_GOAL_BRAKE:-1.5}
    PATH_EXT=${NOCLIMB_PATH_EXT:-2}
    SAFETY_R=${NOCLIMB_SAFETY_R:-0}; SAFETY_K=${NOCLIMB_SAFETY_K:-2.0}; BRAKE_NOW=false
    GOV_SECTOR=${NOCLIMB_GOV_SECTOR:-180}; GOV_CORRIDOR=${NOCLIMB_GOV_CORRIDOR:-0}; GOV_BRAKE=${NOCLIMB_GOV_BRAKE:-0}
    GROUND_Z=${NOCLIMB_GROUND_Z:-nan}; AGL_OFF=${NOCLIMB_AGL_OFFSET:-0}
fi
echo "[$(basename "$0")] airframe ${SITL_AIRFRAME:-iris}: PF vehicle_type ${PF_VEHICLE_TYPE}, ${MPPI_YAML}, range inflate ${RANGE_INFLATE} m, path extension ${PATH_EXT}, safety radius ${SAFETY_R} m, governor sector +-${GOV_SECTOR} deg, corridor +-${GOV_CORRIDOR} m, brake ${GOV_BRAKE} m/s^2, policy d_crash ${D_CRASH} m, ground z ${GROUND_Z}, AGL offset ${AGL_OFF} m, lidar ${LIDAR_Z} m up"
# NOCLIMB_GUID=2 flies the policy through PF's MPPI guidance (node_mppi on the GPU, config
# pathfollowing/config/sim_ca_mppi.yaml: horizon/exploration for 5 m/s, speed capped at the
# policy's speed, nose along the velocity, pursuit for sharp turns); 0 (the script default) is
# the speed-mode pursuit with sim.yaml.
NOCLIMB_GUID=${NOCLIMB_GUID:-0}
# the vehicle's thrust scale and tilt limit, the same for both PF nodes (0 = the yaml)
PF_VEHICLE_ARGS="-p hover_thrust:=$(printf '%.4f' ${NOCLIMB_HOVER_THRUST:-0}) -p max_tilt_deg:=$(printf '%.1f' ${NOCLIMB_MAX_TILT_DEG:-0})"
if [ "${SHADOW}" != 1 ]; then
    if [ "${NOCLIMB_GUID}" = "2" ]; then
        PF_ARGS="-p guid_type:=2 -p sim_yaml:=${MPPI_YAML}"
        ros2 run pathfollowing node_mppi --ros-args -p guid_type:=2 -p sim_yaml:=${MPPI_YAML} \
            -p vehicle_type:=${PF_VEHICLE_TYPE} ${PF_VEHICLE_ARGS} 2>&1 | tee ${LOGS}/node_mppi.log &
    else
        PF_ARGS="-p guid_type:=0 -p vertical_priority:=true"
    fi
    ros2 run pathfollowing node_pathfollowing --ros-args ${PF_ARGS} -p vehicle_type:=${PF_VEHICLE_TYPE} \
        ${PF_VEHICLE_ARGS} 2>&1 | tee ${LOGS}/node_pathfollowing.log &
fi

# 3) CA policy -> PF (publishes nothing until flight_manager's climb/hover phases are over)
python3 ${APP_DIR}/noclimb_ca_node.py --ros-args \
    -p policy_dir:=${POLICY_DIR} -p policy:=${POLICY} \
    -p goal_x:=${GOAL_X} -p goal_y:=${GOAL_Y} \
    -p altitude:=${ALT} -p "density:='${NOCLIMB_DENSITY:-cluster}'" \
    -p scan_memory_s:=$(printf '%.2f' ${NOCLIMB_SCAN_MEMORY:-0}) \
    -p scan_memory_min_scans:=${NOCLIMB_SCAN_MEMORY_MIN:-0} -p scan_memory_max_s:=$(printf '%.2f' ${NOCLIMB_SCAN_MEMORY_MAX:-1.5}) \
    -p cmd_tau_s:=$(printf '%.3f' ${NOCLIMB_CMD_TAU:-0}) \
    -p cmd_accel_max:=$(printf '%.3f' ${NOCLIMB_CMD_ACCEL_MAX:-0}) -p cmd_jerk_max:=$(printf '%.3f' ${NOCLIMB_CMD_JERK_MAX:-0}) \
    -p cmd_fade_near_m:=$(printf '%.2f' ${NOCLIMB_CMD_FADE_NEAR:-0}) -p cmd_fade_far_m:=$(printf '%.2f' ${NOCLIMB_CMD_FADE_FAR:-0}) \
    -p cmd_tau_near_s:=$(printf '%.3f' ${NOCLIMB_CMD_TAU_NEAR:--1}) \
    -p pf_direct:=$([ "${NOCLIMB_PF_DIRECT:-0}" = 1 ] && echo true || echo false) \
    -p pf_direct_corridor_m:=$(printf '%.2f' ${NOCLIMB_PF_DIRECT_CORRIDOR:-1.8}) \
    -p pf_direct_on_m:=$(printf '%.1f' ${NOCLIMB_PF_DIRECT_ON:-25}) -p pf_direct_off_m:=$(printf '%.1f' ${NOCLIMB_PF_DIRECT_OFF:-18}) \
    -p pf_direct_align_deg:=$(printf '%.1f' ${NOCLIMB_PF_DIRECT_ALIGN:-2.5}) \
    -p contact_dist_m:=$(printf '%.2f' ${NOCLIMB_CONTACT_DIST:-$(case "${SITL_AIRFRAME:-iris}" in x8) echo 0.77 ;; x500) echo 0.39 ;; *) echo 0.35 ;; esac)}) \
    -p contact_time_s:=$(printf '%.2f' ${NOCLIMB_CONTACT_TIME:-1.0}) \
    -p goal_brake_accel:=$(printf '%.3f' ${GOAL_BRAKE}) \
    -p range_inflate_m:=$(printf '%.3f' ${RANGE_INFLATE}) \
    -p path_extension:=$(printf '%.3f' ${PATH_EXT}) \
    -p safety_radius_m:=$(printf '%.3f' ${SAFETY_R}) -p safety_gain:=$(printf '%.3f' ${SAFETY_K}) \
    -p cmd_brake_immediate:=${BRAKE_NOW} \
    -p wp_z_rate:=$(printf '%.3f' ${NOCLIMB_WP_Z_RATE:--1}) \
    -p lidar_topic:=${LIDAR_TOPIC} \
    $([ "${GROUND_Z}" != nan ] && echo "-p ground_z_m:=$(printf '%.3f' ${GROUND_Z})") -p agl_offset_m:=$(printf '%.3f' ${AGL_OFF}) \
    -p governor_sector_deg:=$(printf '%.1f' ${GOV_SECTOR}) -p governor_corridor_m:=$(printf '%.2f' ${GOV_CORRIDOR}) \
    -p governor_brake_decel:=$(printf '%.2f' ${GOV_BRAKE}) \
    -p goal_handover_m:=$(printf '%.3f' ${SUPER_PF_GOAL_HANDOVER_M:-3.0}) \
    -p shadow:=$([ "${SHADOW}" = 1 ] && echo true || echo false) \
    $([ "${DRIVER}" = flight_driver ] && [ "${SHADOW}" != 1 ] && echo "-p goal_topic:=/flight_driver/goal") \
    -p ref_from_start:=$([ "${NOCLIMB_LIDAR:-sim}" = livox ] && echo true || echo false) \
    -p voxel_m:=$(printf '%.3f' ${NOCLIMB_VOXEL:-$([ "${NOCLIMB_LIDAR:-sim}" = livox ] && echo 0.1 || echo 0)}) \
    -p shadow_goal_ahead_m:=$(printf "%.1f" ${NOCLIMB_SHADOW_AHEAD:-30}) -p shadow_mask_rear_m:=$(printf "%.2f" ${NOCLIMB_SHADOW_MASK:-1.5}) \
    -p shadow_carry_height_m:=$(printf "%.2f" ${NOCLIMB_SHADOW_CARRY:-1.2}) \
    -p log_csv:=${LOGS}/noclimb_ca.csv -p dump_npz:=${LOGS}/noclimb_ca_dump.npz \
    2>&1 | tee ${LOGS}/noclimb_ca_node.log &
[ "${SHADOW}" = 1 ] && { wait; exit; }

# 4) Flight FSM: arm, take off to the path altitude, forward PF attitude; within
#    SUPER_PF_GOAL_HANDOVER_M (envs/ros2.env, as super_pf) of the goal it takes over with PX4
#    position control, stops at the goal and lands there.
if [ "${DRIVER}" = flight_driver ]; then
    V_MAX=$(python3 -c "import json; m = json.load(open('${POLICY_DIR}/cfg.json')); c = dict(m['cfg']); \
c.update(m['policies'].get('${POLICY}', {}).get('overrides', {})); print(c.get('v_max', 5.0))")
    GPS_ARGS=""
    if [ -n "${NOCLIMB_GOAL_GPS:-}" ]; then
        GPS_ARGS="-p goal_lat:=$(echo ${NOCLIMB_GOAL_GPS} | cut -d, -f1) -p goal_lon:=$(echo ${NOCLIMB_GOAL_GPS} | cut -d, -f2)"
    fi
    LIVOX=$([ "${NOCLIMB_LIDAR:-sim}" = livox ] && echo 1 || echo 0)
    ros2 run flight_driver flight_driver --ros-args -p mission:=ca \
        -p takeoff_altitude:=${ALT} -p goal_north_m:=${GOAL_Y} -p goal_east_m:=${GOAL_X} ${GPS_ARGS} \
        -p goal_handover_radius_m:=$(printf '%.3f' ${SUPER_PF_GOAL_HANDOVER_M:-3.0}) \
        -p require_go:=$([ "${NOCLIMB_REQUIRE_GO:-0}" = 1 ] && echo true || echo false) \
        -p ekf_settle_s:=$(printf '%.1f' ${NOCLIMB_EKF_SETTLE:-$([ ${LIVOX} = 1 ] && echo 30 || echo 5)}) \
        -p speed_limit_mps:=$(printf '%.2f' ${NOCLIMB_SPEED_LIMIT:-$(python3 -c "print(${V_MAX} + 2.5)")}) \
        -p log_dir:=${LOGS} 2>&1 | tee ${LOGS}/flight_driver.log &
    wait
    exit
fi
#    Hardcodes vehicle1/fmu/* - remap to /fmu/*. Started last so the CA node sees the arming.
sleep 3
ros2 run flight_manager flight_manager --ros-args \
    -p takeoff_altitude:=${ALT} \
    -p "goal_enu:=[${GOAL_X}, ${GOAL_Y}, ${ALT}]" -p goal_handover_radius_m:=${SUPER_PF_GOAL_HANDOVER_M:-3.0} \
    -r /vehicle1/fmu/in/offboard_control_mode:=/fmu/in/offboard_control_mode \
    -r /vehicle1/fmu/in/trajectory_setpoint:=/fmu/in/trajectory_setpoint \
    -r /vehicle1/fmu/in/vehicle_attitude_setpoint:=/fmu/in/vehicle_attitude_setpoint \
    -r /vehicle1/fmu/in/vehicle_command:=/fmu/in/vehicle_command \
    -r /vehicle1/fmu/out/vehicle_status_v1:=/fmu/out/vehicle_status_v1 \
    -r /vehicle1/fmu/out/vehicle_local_position:=/fmu/out/vehicle_local_position \
    -r /vehicle1/fmu/out/vehicle_attitude:=/fmu/out/vehicle_attitude \
    -r /vehicle1/fmu/out/sensor_combined:=/fmu/out/sensor_combined \
    2>&1 | tee ${LOGS}/flight_manager.log &

wait
