#! /bin/bash

# SCRIPT TO BUILD ROS2 PACKAGES INSIDE THE CONTAINER
#
# USAGE (via run.sh ros2 build):
#   build                       -> build all workspaces (dirs with non-empty src/)
#   build <workspace>           -> clean-build the whole workspace
#   build <workspace> <pkg...>  -> incremental build of just those packages

# INITIAL STATEMENTS
# >>>----------------------------------------------------

# SET THE BASIC ENVIRONMENT VARIABLE
export TERM=xterm-256color

# SET THE BASE DIRECTORY
BASE_DIR=$(dirname $(readlink -f "$0"))
WORKSPACE_DIR=$(dirname $(dirname $(readlink -f "$0")))

# SOURCE THE ENVIRONMENT AND FUNCTION DEFINITIONS
for file in ${BASE_DIR}/include/*.sh; do
    source ${file}
done
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


# MAIN STATEMENTS
# >>>----------------------------------------------------

# THIS IMAGE RUNS AS root AND HAS NO sudo; USE sudo ONLY WHEN NEEDED AND AVAILABLE.
if [ "$(id -u)" -eq 0 ] || ! command -v sudo >/dev/null 2>&1; then
    SUDO=""
else
    SUDO="sudo"
fi

# UNSET MPI ENVIRONMENT VARIABLES TO PREVENT BUILD ERRORS
unset OPAL_PREFIX
unset OPENMPI_VERSION
unset OMPI_MCA_coll_hcoll_enable

# PREPARE THE BUILD ENVIRONMENT (KEYRING + EXTRA APT DEPS + rosdep).
# Only needed for full workspace builds, so it is NOT run for single-package builds.
PrepareBuildEnv() {
    ${SUDO} rm -rf /opt/hpcx

    EchoGreen "[$(basename "$0")] UPDATING ROS KEYRING..."
    ${SUDO} rm -f /usr/share/keyrings/ros-archive-keyring.gpg
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
        ${SUDO} gpg --dearmor --batch -o /usr/share/keyrings/ros-archive-keyring.gpg

    EchoGreen "[$(basename "$0")] INSTALLING ADDITIONAL ROS2 PACKAGES..."
    ${SUDO} apt-get update
    ${SUDO} apt-get install -y ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

    if command -v rosdep >/dev/null 2>&1; then
        rosdep update
    else
        EchoYellow "[$(basename "$0")] rosdep NOT FOUND - SKIPPING rosdep"
    fi
}

if [ $# -eq 0 ]; then
    # BUILD ALL WORKSPACES (DIRS WITH NON-EMPTY 'src')
    PrepareBuildEnv

    for dir in $(find ${WORKSPACE_DIR} -maxdepth 1 -mindepth 1 -type d)
    do
        # CASE 1: THE DIRECTORY HAS 'src' SUBDIRECTORY
        if [ -d "${dir}/src" ]; then
            # IF 'src' IS EMPTY, SKIP THE BUILD
            if [ -z "$(ls -A ${dir}/src)" ]; then
                EchoRed "[$(basename "$0")] ${dir}/src IS EMPTY"
                EchoRed "[$(basename "$0")] SKIPPING ${dir}"
            else
                EchoGreen "[$(basename "$0")] ${dir} SEEMS TO BE A ROS2 WORKSPACE"
                EchoGreen "[$(basename "$0")] BUILDING PACKAGES INSIDE ${dir}"

                # REMOVE THE PREVIOUS BUILDS
                rm -rf \
                    ${dir}/log \
                    ${dir}/install \
                    ${dir}/build

                # INSTALL THE DEPENDENCIES FOR THE PACKAGES
                if command -v rosdep >/dev/null 2>&1; then
                    rosdep install -i --from-paths ${dir}/src --rosdistro ${ROS_DISTRO} -y
                fi

                # BUILD THE PACKAGES
                (cd ${dir} || exit 1; colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF)
                EchoGreen "[$(basename "$0")] BUILD COMPLETED FOR PACKAGES IN ${dir}"
                EchoBoxLine
            fi
        # CASE 2: THE DIRECTORY DOES NOT HAVE 'src' SUBDIRECTORY
        else
            EchoRed "[$(basename "$0")] ${dir} DOES NOT SEEM TO BE A ROS2 WORKSPACE"
        fi
    done
else
    # FIRST ARGUMENT IS THE WORKSPACE; ANY REMAINING ARGUMENTS ARE PACKAGES TO BUILD.
    WS_DIR=${WORKSPACE_DIR}/$1
    shift

    CheckDirExists ${WS_DIR}
    CheckDirEmpty ${WS_DIR}/src

    if [ $# -eq 0 ]; then
        # NO PACKAGES GIVEN -> CLEAN-BUILD THE WHOLE WORKSPACE
        PrepareBuildEnv
        EchoGreen "[$(basename "$0")] CLEAN-BUILDING ALL PACKAGES IN ${WS_DIR}"

        rm -rf \
            ${WS_DIR}/log \
            ${WS_DIR}/install \
            ${WS_DIR}/build

        export AMENT_PREFIX_PATH=""
        export COLCON_PREFIX_PATH=""
        export CMAKE_PREFIX_PATH=""

        source /opt/ros/${ROS_DISTRO}/setup.bash

        (cd ${WS_DIR} || exit 1; colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF)
    else
        # PACKAGES GIVEN -> FAST INCREMENTAL BUILD OF JUST THOSE PACKAGES (no env prep).
        # (Their dependencies must already be built; source the existing install so colcon finds them.)
        EchoGreen "[$(basename "$0")] BUILDING SELECTED PACKAGE(S): $* IN ${WS_DIR}"

        source /opt/ros/${ROS_DISTRO}/setup.bash
        [ -f ${WS_DIR}/install/setup.bash ] && source ${WS_DIR}/install/setup.bash

        (cd ${WS_DIR} || exit 1; colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF --packages-select "$@")
    fi

    EchoGreen "[$(basename "$0")] BUILD COMPLETED FOR ${WS_DIR}"
    EchoBoxLine
fi

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
