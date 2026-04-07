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

# CHECK IF THE FILE sitl_run.sh EXISTS
CheckFileExists "${PX4GazeboSimDir}/sitl_run.sh"

# BACKUP THE ORIGINAL sitl_run.sh
cp \
    ${PX4GazeboSimDir}/sitl_run.sh \
    ${PX4GazeboSimDir}/sitl_run.bak

# DELETE EXCEPT THE FIRST LINE TO BUILD PX4-sitl WHILE NOT RUNNING PX4-SITL
sed -i '2,$d' ${PX4GazeboSimDir}/sitl_run.sh

# BUILD PX4-SITL
(cd ${PX4RootDir} || exit 1; make px4_sitl gazebo-classic)

# RESTORE THE ORIGINAL sitl_run.sh
mv \
    ${PX4GazeboSimDir}/sitl_run.bak \
    ${PX4GazeboSimDir}/sitl_run.sh

# SET THE PERMISSIONS OF THE PX4-Autopilot DIRECTORY
chmod -R o+rwx $(dirname "$BASE_DIR")

EchoGreen "[$(basename $0)] PX4-sitl BUILT SUCCESSFULLY."
EchoBoxLine

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
