#!/bin/bash

# INITIAL STATEMENTS
# >>>----------------------------------------------------

# SET THE BASIC ENVIRONMENT VARIABLE
export TERM=xterm-256color

# SET THE BASE DIRECTORY
BASE_DIR=$(dirname $(readlink -f "$0"))

# SOURCE THE ENVIRONMENT AND FUNCTION DEFINITIONS
for file in ${BASE_DIR}/include/*.sh; do
    source ${file}
done
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


# MAIN STATEMENTS
# >>>----------------------------------------------------

# CHCEK IF DIRECTORY PX4-Autopilot AND gazebo-classic EXISTS
CheckDirExists "PX4-Autopilot"
CheckDirExists "PX4-Autopilot/Tools/simulation/gazebo-classic"

CustomPX4FileDir="A4VAI-Custom-PX4-File"
PX4RootDir="PX4-Autopilot"
PX4AirFrameDir="${PX4RootDir}/ROMFS/px4fmu_common/init.d-posix/airframes"
PX4GazeboSimDir="${PX4RootDir}/Tools/simulation/gazebo-classic/sitl_gazebo-classic"

# COPY THE CUSTOM PX4 FILES TO THE PX4-Autopilot DIRECTORY
# >>>----------------------------------------------------
cp ${CustomPX4FileDir}/worlds_templet/windy_test.world                                ${PX4GazeboSimDir}/worlds/windy.world

# COPY X8 AIRFRAME
cp ${CustomPX4FileDir}/airframes/sitl/2000_gazebo-classic_x8                          ${PX4AirFrameDir}/2000_gazebo-classic_x8
cp ${CustomPX4FileDir}/airframes/sitl/CMakeLists.txt                                  ${PX4AirFrameDir}/CMakeLists.txt

cp ${CustomPX4FileDir}/simulator_mavlink/sitl_targets_gazebo-classic.cmake            ${PX4RootDir}/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake
cp -r ${CustomPX4FileDir}/x8                                                          ${PX4GazeboSimDir}/models/x8
cp ${CustomPX4FileDir}/sitl_run/sitl_run.sh                                           ${PX4GazeboSimDir}/sitl_run.sh
cp ${CustomPX4FileDir}/mavlink_configs/sitl.json                                      ${PX4RootDir}/test/mavsdk_tests/configs/sitl.json
cp ${CustomPX4FileDir}/gitignore/.gitignore                                           ${PX4GazeboSimDir}/.gitignore
cp ${CustomPX4FileDir}/location/sitl_tests.yml                                        ${PX4RootDir}/.github/workflows/sitl_tests.yml
cp ${CustomPX4FileDir}/firmware_build_test/firmware_build_test.yml                    ${PX4GazeboSimDir}/.github/workflows/firmware_build_test.yml
cp ${CustomPX4FileDir}/multiple_run/sitl_multiple_run.sh                              ${PX4GazeboSimDir}/sitl_multiple_run.sh
cp ${CustomPX4FileDir}/launch/x8.launch                                               ${PX4RootDir}/launch/x8.launch

# A4VAI X8 identified from the vehicle's flight logs (scripts/px4/x8/README.md) replaces the
# Custom repo's X8 approximation: SITL airframe 2000 and the x8 Gazebo model. The PX4 build
# regenerates x8.sdf from x8.sdf.jinja and refuses to overwrite an x8.sdf that differs from its
# .last_generated copy - drop both so the new template is rendered.
cp ${BASE_DIR}/x8/2000_gazebo-classic_x8                                              ${PX4AirFrameDir}/2000_gazebo-classic_x8
cp ${BASE_DIR}/x8/x8.sdf.jinja                                                        ${PX4GazeboSimDir}/models/x8/x8.sdf.jinja
cp ${BASE_DIR}/x8/model.config                                                        ${PX4GazeboSimDir}/models/x8/model.config
rm -f ${PX4GazeboSimDir}/models/x8/x8.sdf ${PX4GazeboSimDir}/models/x8/x8.sdf.last_generated

# A4VAI X500 (scripts/px4/x500/README.md): PX4 v1.16 has it for the new Gazebo only - airframe 2001
# and a Gazebo Classic x500 model with the gz model's meshes, registered next to the X8.
cp ${BASE_DIR}/x500/2001_gazebo-classic_x500                                          ${PX4AirFrameDir}/2001_gazebo-classic_x500
grep -q "2001_gazebo-classic_x500" ${PX4AirFrameDir}/CMakeLists.txt || \
    sed -i "s/^\t2000_gazebo-classic_x8$/\t2000_gazebo-classic_x8\n\t2001_gazebo-classic_x500/" ${PX4AirFrameDir}/CMakeLists.txt
SitlTargets=${PX4RootDir}/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake
grep -qP "^\t\tx500$" ${SitlTargets} || sed -i "s/^\t\tx8$/\t\tx8\n\t\tx500/" ${SitlTargets}
mkdir -p ${PX4GazeboSimDir}/models/x500/meshes
cp ${BASE_DIR}/x500/x500.sdf.jinja ${BASE_DIR}/x500/model.config                        ${PX4GazeboSimDir}/models/x500/
cp ${PX4RootDir}/Tools/simulation/gz/models/x500_base/meshes/*                           ${PX4GazeboSimDir}/models/x500/meshes/
rm -f ${PX4GazeboSimDir}/models/x500/x500.sdf ${PX4GazeboSimDir}/models/x500/x500.sdf.last_generated
# the Gazebo build lists its *.jinja models when it configures: a build configured before the x500
# existed would not render x500.sdf - make it reconfigure once
GzBuild=${PX4RootDir}/build/px4_sitl_default/build_gazebo-classic/build.ninja
[ -f ${GzBuild} ] && ! grep -q "x500.sdf" ${GzBuild} && touch ${PX4GazeboSimDir}/CMakeLists.txt

# BUILD PX4-SITL WITHOUT LAUNCHING THE SIMULATION.
# sitl_run.sh (Tools/simulation/gazebo-classic/sitl_run.sh, invoked by the
# gazebo-classic make target) exits early when DONT_RUN is set.
# rotor spin-up reaction torque in the Gazebo motor plugin (<rotorInertia>; idempotent, see
# scripts/px4/x8/patch_motor_model.py) - the X8's yaw dynamics depend on it
python3 ${BASE_DIR}/x8/patch_motor_model.py ${PX4GazeboSimDir} || exit 1
# IMU acceleration from the link's velocity (<accelFromVelocity>; idempotent, see
# scripts/px4/x8/patch_imu_plugin.py) - the force-based reading misses touchdown impulses
python3 ${BASE_DIR}/x8/patch_imu_plugin.py ${PX4GazeboSimDir} || exit 1

(cd ${PX4RootDir} || exit 1; DONT_RUN=1 make px4_sitl gazebo-classic)

# SET THE PERMISSIONS OF THE PX4-Autopilot DIRECTORY
chmod -R o+rwx $(dirname "$BASE_DIR")

EchoGreen "[$(basename $0)] PX4-sitl BUILT SUCCESSFULLY."
EchoBoxLine

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
