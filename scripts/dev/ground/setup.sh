#!/bin/bash
# One-time (idempotent) preparation of the UGV/USV simulation, all in ~/Documents/A4VAI-SITL/ground_sim - the
# shared PX4 tree, ros2_ws and the compose stack are only read:
#   etc/     copy of the PX4 SITL etc dir + the airframes of scripts/dev/ground/airframes (2010 UGV, 2011 USV),
#            uXRCE-DDS client pointed at 127.0.0.1 (the ROS2 container shares the SITL container's network)
#   models/  r1_rover.sdf / boat.sdf with lockstep off (and the boat's missing ground-truth plugin added): with lockstep on, the classic models stall once armed
#            (rover_differential publishes actuator_motors on a 10 ms timer, Gazebo waits for an actuator
#            message every 4 ms step, PX4's clock only advances on Gazebo's sensor messages)
#   ws/      overlay colcon workspace: src/A4VAI-PathFollowing = a git worktree (branch ugv-usv of
#            JOCIIIII/A4VAI-PathFollowing) of the PathFollowing checkout in ros2_ws, built on top of
#            ros2_ws/install by `setup.sh build` (ros2_ws itself stays on its pinned branch)
#   runs/    one directory per run (ground_run.sh)
set -e
HERE=$(dirname $(readlink -f "$0"))
REPO=$(dirname $(dirname $(dirname "$HERE")))
W=~/Documents/A4VAI-SITL
GS=$W/ground_sim
PX4_ETC=$W/PX4-Autopilot/PX4-Autopilot/build/px4_sitl_default/etc
IMAGE=$(grep '^ROS2_ENV_IMAGE=' $REPO/envs/ros2.env | cut -d= -f2 | tr -d '"')
mkdir -p $GS/runs $GS/ws/src $GS/worlds $GS/models $GS/pycuda_cache

etc() {
    rm -rf $GS/etc; cp -r $PX4_ETC $GS/etc; chmod -R u+w $GS/etc
    cp $HERE/airframes/* $GS/etc/init.d-posix/airframes/
    sed -i 's/uxrce_dds_client start -t udp -h [0-9.]* /uxrce_dds_client start -t udp -h 127.0.0.1 /' $GS/etc/init.d-posix/rcS
    grep -n "uxrce_dds_client start" $GS/etc/init.d-posix/rcS
    ls $GS/etc/init.d-posix/airframes | grep "^201"
}
models() {
    M=$W/PX4-Autopilot/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
    for m in r1_rover boat; do
        sed 's#<enable_lockstep>true</enable_lockstep>#<enable_lockstep>false</enable_lockstep>#' $M/$m/$m.sdf > $GS/models/$m.sdf
        grep -c "<enable_lockstep>false" $GS/models/$m.sdf
    done
    # calibrated IMU: no turn-on bias. The plugin's defaults (gyro 0.5 deg/s, accel 0.2 m/s^2, 1 sigma, drawn
    # anew every start) are those of an uncalibrated sensor: on a vehicle that stands still before it drives,
    # the estimator cannot tell an accel bias from a tilt, and the rover's velocity estimate ran 0.2 m/s (18 %)
    # low in steady driving and up to 1 m/s off in turns (step_ugv_7). Noise and bias random walk stay.
    for m in r1_rover boat; do
        grep -q TurnOnBiasSigma $GS/models/$m.sdf || sed -i 's#\(<imuTopic>/imu</imuTopic>\)#\1\n      <gyroscopeTurnOnBiasSigma>0</gyroscopeTurnOnBiasSigma>\n      <accelerometerTurnOnBiasSigma>0</accelerometerTurnOnBiasSigma>#' $GS/models/$m.sdf
        grep -c TurnOnBiasSigma $GS/models/$m.sdf
    done
    # rover wheel contacts: no correcting velocity (max_vel 0.01, min_depth 0) - with the world's limit on the
    # contact correcting velocity (ugv_flat.world, ground_world.py) this keeps ODE's penetration-correction
    # velocities out of the IMU: during skid turns the gyro reported 0.06-0.10 rad/s of mean roll / pitch rate
    # (0.3 rad/s rms) while the chassis stayed level, the EKF tilted up to 26 deg, rejected the GPS velocity
    # and drifted 6.9 m (e_ugv_offset_g2, g_spot_e1); with both changes 0.01-0.02 rad/s mean, 0.19 m (g_spot_w1e1)
    sed -i -e 's#<max_vel>0.0</max_vel>#<max_vel>0.01</max_vel>#' -e 's#<min_depth>0.001</min_depth>#<min_depth>0.0</min_depth>#' $GS/models/r1_rover.sdf
    grep -c "<max_vel>0.01" $GS/models/r1_rover.sdf
    # flat UGV world: the stock empty world with the contact correcting velocity limited (see above)
    sed -e 's#<contact_max_correcting_vel>100</contact_max_correcting_vel>#<contact_max_correcting_vel>0.1</contact_max_correcting_vel>#' \
        -e 's#<contact_surface_layer>0.001</contact_surface_layer>#<contact_surface_layer>0.002</contact_surface_layer>#' \
        $W/PX4-Autopilot/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world > $GS/worlds/ugv_flat.world
    grep -c "correcting_vel>0.1" $GS/worlds/ugv_flat.world
    # rover: acceleration from the change of the link velocity (the A4VAI IMU plugin option of the X8 model).
    # The plugin's default is ODE's "relative linear acceleration" (applied force / mass), which leaves out
    # the constraint forces of the wheel joints and contacts: the rover's velocity estimate then ran 0.35 m/s
    # low in straight driving, with 0.24 m/s of false lateral velocity (ekf_h0). The boat is a free body under
    # applied forces and estimates within 0.05 m/s rms with the default (step_usv_4).
    grep -q accelFromVelocity $GS/models/r1_rover.sdf || sed -i 's#\(<imuTopic>/imu</imuTopic>\)#\1\n      <accelFromVelocity>true</accelFromVelocity>\n      <accelAverageTime>0.004</accelAverageTime>\n      <accelSaturation>156.0</accelSaturation>#' $GS/models/r1_rover.sdf
    grep -c accelFromVelocity $GS/models/r1_rover.sdf
    # the boat model has no ground-truth plugin, so its magnetometer plugin never learns the position and
    # simulates the field at latitude 0, longitude 0 (declination -5 deg, inclination -30 deg) while the
    # estimator applies the declination of the world's location: a constant 8 deg yaw error and the
    # pre-arm failure "heading estimate not stable" (step_usv_1). The rover model has the plugin.
    grep -q groundtruth_plugin $GS/models/boat.sdf || sed -i "s#    <plugin name='magnetometer_plugin'#    <plugin name='groundtruth_plugin' filename='libgazebo_groundtruth_plugin.so'>\n      <robotNamespace/>\n    </plugin>\n    <plugin name='magnetometer_plugin'#" $GS/models/boat.sdf
    grep -c groundtruth_plugin $GS/models/boat.sdf
    # thrusters reversible: without <reversible> the motor model takes |force|, so a reverse command pushes
    # forward - the boat accelerated to 3.5 m/s on a stop command (step_usv_2)
    grep -q "<reversible>" $GS/models/boat.sdf || sed -i 's#\(<motorNumber>[01]</motorNumber>\)#\1\n      <reversible>true</reversible>#' $GS/models/boat.sdf
    grep -c "<reversible>true" $GS/models/boat.sdf
    # ... and with <reversible> the force follows the sign of the joint velocity = turning direction x command:
    # "cw" (-1) then drives the boat backwards on a forward command (step_usv_3) -> ccw on both thrusters
    sed -i 's#<turningDirection>cw</turningDirection>#<turningDirection>ccw</turningDirection>#' $GS/models/boat.sdf
    grep -c "<turningDirection>ccw" $GS/models/boat.sdf
}
ws() {
    PF=$W/ROS2/ros2_ws/src/A4VAI-PathFollowing
    if [ ! -d $GS/ws/src/A4VAI-PathFollowing ]; then
        # the ground-vehicle changes are the pushed branch ugv-usv (an explicit refspec: the submodule clone may
        # track only its pinned branch)
        git -C $PF fetch origin +refs/heads/ugv-usv:refs/remotes/origin/ugv-usv
        if git -C $PF show-ref --verify --quiet refs/heads/ugv-usv; then
            git -C $PF worktree add $GS/ws/src/A4VAI-PathFollowing ugv-usv
        else
            git -C $PF worktree add -b ugv-usv $GS/ws/src/A4VAI-PathFollowing origin/ugv-usv
        fi
    fi
    git -C $GS/ws/src/A4VAI-PathFollowing status --short --branch | head -5
}
build() {
    # as scripts/ros2/build.sh: the image pairs setuptools 83 with packaging 21.3, so packaging is upgraded
    # in this throw-away container first. The install holds COPIES of the Python sources and the yaml files
    # even with --symlink-install (this setuptools): rebuild after every edit. Handed back to the caller's uid.
    docker run --rm --gpus all \
        -v $W/ROS2:/home/user/workspace/ros2:ro -v $GS/ws:/home/user/workspace/ground_ws \
        --entrypoint bash "$IMAGE" -c "source /opt/ros/humble/setup.bash && source /home/user/workspace/ros2/ros2_ws/install/setup.bash \
            && pip install -q 'packaging>=24.2' 2>/dev/null; cd /home/user/workspace/ground_ws \
            && colcon build --symlink-install --packages-select pathfollowing 2>&1 | tail -5; \
            chown -R $(id -u):$(id -g) build install log"
}
case "${1:-all}" in
    etc) etc ;; models) models ;; ws) ws ;; build) build ;;
    all) etc; models; ws; build ;;
    *) sed -n 2,9p "$0"; exit 1 ;;
esac
