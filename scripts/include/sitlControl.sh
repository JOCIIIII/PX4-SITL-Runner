#!/bin/bash

# FUNCTION DEFINITIONS FOR SITL SIMULATION CONTROL

BasicSetup() {
    # FUNCTION TO SET UP THE WORKSPACE AND COPY RESOURCES ON STARTUP
    # >>>---------------------------------------------------------
    EchoGreen "[$(basename "$0")] [WORKSPACE SETUP - HOST]"

    CheckDirExists ${SITL_DEPLOY_DIR} create

    CheckDirExists ${PX4_WORKSPACE} create
    CheckDirExists ${GAZEBO_CLASSIC_WORKSPACE} create
    CheckDirExists ${AIRSIM_WORKSPACE} create
    CheckDirExists ${ROS2_WORKSPACE} create
    CheckDirExists ${QGC_WORKSPACE} create

    cp ${REPO_DIR}/compose.yml ${SITL_DEPLOY_DIR}/compose.yml
    cp -r ${REPO_DIR}/envs ${SITL_DEPLOY_DIR}

    EchoBoxLine


    # SET DISPLAY AND AUDIO-RELATED ENVIRONMENT VARIABLES TO THE .env FILE
    SetComposeDisplay ${SITL_ENV_DIR}/common.env

    EchoGreen "[$(basename "$0")] [ROS_DOMAIN_ID SETUP]"

    # SET ROS_DOMAIN_ID TO THE .env FILE
    # IF ROS_DOMAIN_ID IS BLANK STRING, RANDOMLY GENERATE A VALUE BETWEEN 0 - 101
    if [ -z "${SITL_ROS_DOMAIN_ID}" ]; then
        ROS_DOMAIN_ID=$(shuf -i 0-101 -n 1)
        EchoYellow "[$(basename "$0")] ROS_DOMAIN_ID IS NOT SET. RANDOMLY GENERATING A VALUE BETWEEN 0 - 101"
        EchoYellow "[$(basename "$0")] GENERATED ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
    fi

    EchoGreen "[$(basename "$0")] SETTING ROS_DOMAIN_ID AS ${ROS_DOMAIN_ID}"
    sed -i "s~ROS_DOMAIN_ID=\"\"~ROS_DOMAIN_ID=${ROS_DOMAIN_ID}~" ${SITL_ENV_DIR}/common.env

    EchoBoxLine

    # SET WORKSPACE VARIABLES
    EchoGreen "[$(basename "$0")] [WORKSPACE SETUP - COMPOSE .env FILES]"

    # SET PX4_WORKSPACE TO THE .env FILE
    EchoGreen "[$(basename "$0")] SETTING PX4_WORKSPACE AS ${PX4_WORKSPACE}"
    sed -i "s~PX4_WORKSPACE=\"\"~PX4_WORKSPACE=${PX4_WORKSPACE}~" ${SITL_ENV_DIR}/px4.env

    # SET GAZEBO_CLASSIC_WORKSPACE TO THE .env FILE
    EchoGreen "[$(basename "$0")] SETTING GAZEBO_CLASSIC_WORKSPACE AS ${GAZEBO_CLASSIC_WORKSPACE}"
    sed -i "s~GAZEBO_CLASSIC_WORKSPACE=\"\"~GAZEBO_CLASSIC_WORKSPACE=${GAZEBO_CLASSIC_WORKSPACE}~" ${SITL_ENV_DIR}/gazebo-classic.env

    # SET AIRSIM_WORKSPACE TO THE .env FILE
    EchoGreen "[$(basename "$0")] SETTING AIRSIM_WORKSPACE AS ${AIRSIM_WORKSPACE}"
    sed -i "s~AIRSIM_WORKSPACE=\"\"~AIRSIM_WORKSPACE=${AIRSIM_WORKSPACE}~" ${SITL_ENV_DIR}/airsim.env

    # SET ROS2_WORKSPACE TO THE .env FILE
    EchoGreen "[$(basename "$0")] SETTING ROS2_WORKSPACE AS ${ROS2_WORKSPACE}"
    sed -i "s~ROS2_WORKSPACE=\"\"~ROS2_WORKSPACE=${ROS2_WORKSPACE}~" ${SITL_ENV_DIR}/ros2.env

    # SET QGC_WORKSPACE TO THE .env FILE
    EchoGreen "[$(basename "$0")] SETTING QGC_WORKSPACE AS ${QGC_WORKSPACE}"
    sed -i "s~QGC_WORKSPACE=\"\"~QGC_WORKSPACE=${QGC_WORKSPACE}~" ${SITL_ENV_DIR}/qgc.env

    EchoBoxLine


    EchoGreen "[$(basename "$0")] [WORKSPACE SETUP - SCRIPT TOOLS]"

    # CREATE SCRIPTS AND INCLUDE DIRECTORIES
    CheckDirExists ${PX4_WORKSPACE}/scripts create
    CheckDirExists ${PX4_WORKSPACE}/scripts/include create
    CheckDirExists ${GAZEBO_CLASSIC_WORKSPACE}/scripts create
    CheckDirExists ${GAZEBO_CLASSIC_WORKSPACE}/scripts/include create
    CheckDirExists ${AIRSIM_WORKSPACE}/scripts create
    CheckDirExists ${AIRSIM_WORKSPACE}/scripts/include create
    CheckDirExists ${ROS2_WORKSPACE}/scripts create
    CheckDirExists ${ROS2_WORKSPACE}/scripts/include create
    CheckDirExists ${QGC_WORKSPACE}/scripts create
    CheckDirExists ${QGC_WORKSPACE}/scripts/include create

    # COPY BUILD SCRIPT AND FUNCTION DEFINITIONS
    cp -r ${BASE_DIR}/px4/* ${PX4_WORKSPACE}/scripts/
    cp -r ${BASE_DIR}/include/* ${PX4_WORKSPACE}/scripts/include
    cp -r ${BASE_DIR}/gazebo-classic/* ${GAZEBO_CLASSIC_WORKSPACE}/scripts/
    cp -r ${BASE_DIR}/include/* ${GAZEBO_CLASSIC_WORKSPACE}/scripts/include
    cp -r ${BASE_DIR}/airsim/* ${AIRSIM_WORKSPACE}/scripts/
    cp -r ${BASE_DIR}/include/* ${AIRSIM_WORKSPACE}/scripts/include
    cp -r ${BASE_DIR}/ros2/* ${ROS2_WORKSPACE}/scripts/
    cp -r ${BASE_DIR}/include/* ${ROS2_WORKSPACE}/scripts/include
    cp -r ${BASE_DIR}/qgc/* ${QGC_WORKSPACE}/scripts/
    cp -r ${BASE_DIR}/include/* ${QGC_WORKSPACE}/scripts/include

    EchoBoxLine


    EchoGreen "[$(basename "$0")] [NETWORK SETUP]"

    # SET BASEICN NETWORK INFO
    # MODIFY SITL_NETWORK_NAME="" TO VALUE OF SITL_NETWORK_NAME
    # MODIFY SITL_NETWORK_SUBNET="" TO VALUE OF SITL_NETWORK_SUBNET
    sed -i "s~SITL_NETWORK_NAME=\"\"~SITL_NETWORK_NAME=\"${SITL_NETWORK_NAME}\"~" ${SITL_ENV_DIR}/common.env
    sed -i "s~SITL_NETWORK_SUBNET=\"\"~SITL_NETWORK_SUBNET=\"${SITL_NETWORK_SUBNET}\"~" ${SITL_ENV_DIR}/common.env

    # DETACH /16 SUBNET FROM THE NETWORK
    BASE_IP=$(echo ${SITL_NETWORK_SUBNET} | cut -d '.' -f1-3)

    # SET CONTAINER IP ADDRESSES
    PX4_IP=${BASE_IP}.2
    GAZEBO_CLASSIC_IP=${BASE_IP}.3
    AIRSIM_IP=${BASE_IP}.4
    ROS2_IP=${BASE_IP}.5
    QGC_IP=${BASE_IP}.6

    # SET CONTAINER IP ADDRESSES
    sed -i "s~PX4_IP=\"\"~PX4_IP=\"${PX4_IP}\"~" ${SITL_ENV_DIR}/px4.env
    sed -i "s~GAZEBO_CLASSIC_IP=\"\"~GAZEBO_CLASSIC_IP=\"${GAZEBO_CLASSIC_IP}\"~" ${SITL_ENV_DIR}/gazebo-classic.env
    sed -i "s~AIRSIM_IP=\"\"~AIRSIM_IP=\"${AIRSIM_IP}\"~" ${SITL_ENV_DIR}/airsim.env
    sed -i "s~ROS2_IP=\"\"~ROS2_IP=\"${ROS2_IP}\"~" ${SITL_ENV_DIR}/ros2.env
    sed -i "s~QGC_IP=\"\"~QGC_IP=\"${QGC_IP}\"~" ${SITL_ENV_DIR}/qgc.env

    EchoGreen "[$(basename "$0")] *   PX4 CONTAINER IP: ${PX4_IP}"
    EchoGreen "[$(basename "$0")] *   GAZEBO-CLASSIC CONTAINER IP: ${GAZEBO_CLASSIC_IP}"
    EchoGreen "[$(basename "$0")] *   AIRSIM CONTAINER IP: ${AIRSIM_IP}"
    EchoGreen "[$(basename "$0")] *   ROS2 CONTAINER IP: ${ROS2_IP}"
    EchoGreen "[$(basename "$0")] *   QGC CONTAINER IP: ${QGC_IP}"

    EchoBoxLine
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

SetDebugMode() {
    # FUNCTION TO SET THE CONTAINER IN DEBUG MODE
    # >>>---------------------------------------------------------
    # INPUTS:
    # $1: $0 (MAIN SCRIPT RUNNING THE FUNCTION)
    # $2: DEBUG TARGET (all, px4, gazebo-classic, airsim, ros2, qgc)
    # ------------------------------------------------------------
    # EXAMPLE:
    # SetDebugMode $0 all
    # ------------------------------------------------------------
    SCRIPT_NAME=$(basename "$1")
    DEBUG_TAGET=$2

    DEBUG_SCRIPT="debug.sh"
    
    if [ "${DEBUG_TAGET}x" == "allx" ]; then
        EchoYellow "[$(basename "$0")] SETTING ALL CONTAINERS IN DEBUG MODE"
        SetRunModePX4 $1 debug
        SetRunModeGazeboClassic $1 debug
        SetRunModeAirSim $1 debug
        SetRunModeROS2 $1 debug
        SetRunModeQGC $1 debug
    elif [ "${DEBUG_TAGET}x" == "px4x" ]; then
        EchoYellow "[${SCRIPT_NAME}] SETTING PX4-AUTOPILOT CONTAINER IN DEBUG MODE"
        SetRunModePX4 $1 debug
    elif [ "${DEBUG_TAGET}x" == "gazebo-classicx" ]; then
        EchoYellow "[${SCRIPT_NAME}] SETTING GAZEBO-CLASSIC CONTAINER IN DEBUG MODE"
        SetRunModeGazeboClassic $1 debug
    elif [ "${DEBUG_TAGET}x" == "airsimx" ]; then
        EchoYellow "[${SCRIPT_NAME}] SETTING AIRSIM CONTAINER IN DEBUG MODE"
        SetRunModeAirSim $1 debug
    elif [ "${DEBUG_TAGET}x" == "ros2x" ]; then
        EchoYellow "[${SCRIPT_NAME}] SETTING ROS2 CONTAINER IN DEBUG MODE"
        SetRunModeROS2 $1 debug
    elif [ "${DEBUG_TAGET}x" == "qgcx" ]; then
        EchoYellow "[${SCRIPT_NAME}] SETTING QGroundControl CONTAINER IN DEBUG MODE"
        SetRunModeQGC $1 debug
    else
        EchoRed "[${SCRIPT_NAME}] INVALID INPUT \"${DEBUG_TAGET}\". PLEASE CHECK THE ARGUMENT."
        EchoBoxLine
        exit 1
    fi
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

SetDebugModePX4() {
    # FUNCTION TO SET THE PX4-AUTOPILOT CONTAINER IN DEBUG MODE
    ## CONTROLLED BY THE FUNCTION SetDebugMode
    # >>>---------------------------------------------------------
    CheckFileExists ${PX4_WORKSPACE}/scripts/${DEBUG_SCRIPT}
    CheckFileExecutable ${PX4_WORKSPACE}/scripts/${DEBUG_SCRIPT}
    sed -i "s~PX4_RUN_COMMAND=\"\"~PX4_RUN_COMMAND=\'bash -c \"/home/user/workspace/scripts/${DEBUG_SCRIPT}\"\'~g" ${SITL_ENV_DIR}/px4.env
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

SetDebugModeGazeboClassic() {
    # FUNCTION TO SET THE GAZEBO-CLASSIC CONTAINER IN DEBUG MODE
    ## CONTROLLED BY THE FUNCTION SetDebugMode
    # >>>---------------------------------------------------------
    CheckFileExists ${GAZEBO_CLASSIC_WORKSPACE}/scripts/${DEBUG_SCRIPT}
    CheckFileExecutable ${GAZEBO_CLASSIC_WORKSPACE}/scripts/${DEBUG_SCRIPT}
    sed -i "s~GAZEBO_CLASSIC_RUN_COMMAND=\"\"~GAZEBO_CLASSIC_RUN_COMMAND=\'bash -c \"/home/user/workspace/gazebo/scripts/${DEBUG_SCRIPT}\"\'~g" ${SITL_ENV_DIR}/gazebo-classic.env
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

SetDebugModeAirSim() {
    # FUNCTION TO SET THE AIRSIM CONTAINER IN DEBUG MODE
    ## CONTROLLED BY THE FUNCTION SetDebugMode
    # >>>---------------------------------------------------------
    CheckFileExists ${AIRSIM_WORKSPACE}/scripts/${DEBUG_SCRIPT}
    CheckFileExecutable ${AIRSIM_WORKSPACE}/scripts/${DEBUG_SCRIPT}
    sed -i "s~AIRSIM_RUN_COMMAND=\"\"~AIRSIM_RUN_COMMAND=\'bash -c \"/home/ue4/workspace/scripts/${DEBUG_SCRIPT}\"\'~g" ${SITL_ENV_DIR}/airsim.env
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

SetDebugModeROS2() {
    # FUNCTION TO SET THE ROS2 CONTAINER IN DEBUG MODE
    ## CONTROLLED BY THE FUNCTION SetDebugMode
    # >>>---------------------------------------------------------
    CheckFileExists ${ROS2_WORKSPACE}/scripts/${DEBUG_SCRIPT}
    CheckFileExecutable ${ROS2_WORKSPACE}/scripts/${DEBUG_SCRIPT}
    sed -i "s~ROS2_RUN_COMMAND=\"\"~ROS2_RUN_COMMAND=\'bash -c \"/home/user/workspace/ros2/scripts/${DEBUG_SCRIPT}\"\'~g" ${SITL_ENV_DIR}/ros2.env
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

SetDebugModeQGC() {
    # FUNCTION TO SET THE QGroundControl CONTAINER IN DEBUG MODE
    ## CONTROLLED BY THE FUNCTION SetDebugMode
    # >>>---------------------------------------------------------
    CheckFileExists ${QGC_WORKSPACE}/scripts/${DEBUG_SCRIPT}
    CheckFileExecutable ${QGC_WORKSPACE}/scripts/${DEBUG_SCRIPT}
    sed -i "s~QGC_RUN_COMMAND=\"\"~QGC_RUN_COMMAND=\'bash -c \"/home/user/workspace/scripts/${DEBUG_SCRIPT}\"\'~g" ${SITL_ENV_DIR}/qgc.env
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

SetRunModePX4() {
    # FUNCTION TO SET THE PX4-AUTOPILOT CONTAINER IN RUN MODE
    # >>>---------------------------------------------------------
    # INPUTS:
    # $1: $0 (MAIN SCRIPT RUNNING THE FUNCTION)
    # $2: RUN MODE (gazebo-classic-sitl, airsim-sitl, clone, build, clean, sitl-gazebo-classic-standalone)
    # ------------------------------------------------------------
    # EXAMPLE:
    # SetRunModePX4 $0 gazebo-classic-sitl
    # ------------------------------------------------------------
    
    SCRIPT_NAME=$(basename "$1")
    RUN_MODE=$2

    if [ "${RUN_MODE}x" == "gazebo-classic-sitlx" ]; then
        RUN_SCRIPT="sitl-gazebo-classic.sh"
    elif [ "${RUN_MODE}x" == "gazebo-classic-airsim-sitlx" ]; then
        RUN_SCRIPT="sitl-gazebo-classic-airsim.sh"
    elif [ "${RUN_MODE}x" == "clonex" ]; then
        EchoGreen "[${SCRIPT_NAME}] CLONE PX4-AUTOPILOT REPOSITORY TO THE PX4_WORKSPACE"
        EchoGreen "[${SCRIPT_NAME}] CONTAINER WILL BE STOPPED AFTER THE PROCESS"
        RUN_SCRIPT="clone.sh"
    elif [ "${RUN_MODE}x" == "buildx" ]; then
        EchoGreen "[${SCRIPT_NAME}] BUILD PX4-AUTOPILOT INSIDE THE CONTAINER"
        EchoGreen "[${SCRIPT_NAME}] CONTAINER WILL BE STOPPED AFTER THE BUILD PROCESS."
        RUN_SCRIPT="build.sh"
    elif [ "${RUN_MODE}x" == "cleanx" ]; then
        EchoGreen "[${SCRIPT_NAME}] CLEAN PX4-AUTOPILOT BUILD INSIDE THE CONTAINER"
        EchoGreen "[${SCRIPT_NAME}] CONTAINER WILL BE STOPPED AFTER THE CLEAN PROCESS."

        EchoRed "[${SCRIPT_NAME}] [WARNING]"
        EchoRed "[${SCRIPT_NAME}] THIS WILL DELETE ALL BUILD FILES. THIS ACTION CANNOT BE UNDONE."
        WarnAction
        RUN_SCRIPT="clean.sh"
    elif [ "${RUN_MODE}x" == "sitl-gazebo-classic-standalonex" ]; then
        EchoGreen "[${SCRIPT_NAME}] RUN PX4-AUTOPILOT SITL IN GAZEBO-CLASSIC"
        RUN_SCRIPT="sitl-gazebo-classic-standalone.sh"
    elif [ "${RUN_MODE}x" == "debugx" ]; then
        EchoGreen "[${SCRIPT_NAME}] DEBUG MODE ENABLED FOR PX4-AUTOPILOT CONTAINER"
        RUN_SCRIPT="debug.sh"
    else
        EchoRed "[${SCRIPT_NAME}] INVALID INPUT \"${RUN_MODE}\". PLEASE CHECK THE ARGUMENT."
        EchoBoxLine
        exit 1
    fi

    CheckFileExists ${PX4_WORKSPACE}/scripts/${RUN_SCRIPT}
    CheckFileExecutable ${PX4_WORKSPACE}/scripts/${RUN_SCRIPT}

    sed -i "s~PX4_RUN_COMMAND=\"\"~PX4_RUN_COMMAND=\'bash -c \"/home/user/workspace/scripts/${RUN_SCRIPT}\"\'~g" ${SITL_ENV_DIR}/px4.env
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

SetRunModeGazeboClassic() {
    # FUNCTION TO SET THE GAZEBO-CLASSIC CONTAINER IN RUN MODE
    # >>>---------------------------------------------------------
    # INPUTS:
    # $1: $0 (MAIN SCRIPT RUNNING THE FUNCTION)
    # $2: RUN MODE (gazebo-classic-sitl, gazebo-classic-airsim-sitl, sitl-px4)
    # ------------------------------------------------------------
    # EXAMPLE:
    # SetRunModeGazeboClassic $0 gazebo-classic-sitl
    # ------------------------------------------------------------
    SCRIPT_NAME=$(basename "$1")
    RUN_MODE=$2

    # LOAD THE ENVIRONMENT VARIABLES
    source ${SITL_ENV_DIR}/gazebo-classic.env

    if [ "${RUN_MODE}x" == "gazebo-classic-sitlx" ]; then
        RUN_SCRIPT="sitl-px4.sh"
        # GAEZBO_HEADLESS=false

        if [ -z "${GAEZBO_HEADLESS}" ]; then
            sed -i "s~GAEZBO_HEADLESS=\"\"~GAEZBO_HEADLESS=\"false\"~g" ${SITL_ENV_DIR}/gazebo-classic.env
        fi

        if [ "${GAEZBO_HEADLESS}x" == "truex" ]; then
            EchoYellow "[${SCRIPT_NAME}] GAZEBO HEADLESS MODE ENABLED. ONLY THE GAZEBO SERVER WILL RUN."
            ENABLE_HEADLESS_MODE=1
        fi 

    elif [ "${RUN_MODE}x" == "gazebo-classic-airsim-sitlx" ]; then
        RUN_SCRIPT="sitl-px4.sh"

        if [ -z "${GAEZBO_HEADLESS}" ]; then
            sed -i "s~GAEZBO_HEADLESS=\"\"~GAEZBO_HEADLESS=\"false\"~g" ${SITL_ENV_DIR}/gazebo-classic.env
        fi

        if [ "${GAEZBO_HEADLESS}x" == "truex" ]; then
            EchoYellow "[${SCRIPT_NAME}] GAZEBO HEADLESS MODE ENABLED. ONLY THE GAZEBO SERVER WILL RUN."
            ENABLE_HEADLESS_MODE=1
        fi 

    elif [ "${RUN_MODE}x" == "sitl-px4x" ]; then
        EchoGreen "[${SCRIPT_NAME}] RUN PX4-AUTOPILOT SITL IN GAZEBO-CLASSIC"
        RUN_SCRIPT="sitl-px4.sh"
        GAEZBO_HEADLESS=false

        if [ -z "${GAEZBO_HEADLESS}" ]; then
            sed -i "s~GAEZBO_HEADLESS=\"\"~GAEZBO_HEADLESS=\"false\"~g" ${SITL_ENV_DIR}/gazebo-classic.env
        fi

        if [ "${GAEZBO_HEADLESS}x" == "truex" ]; then
            EchoYellow "[${SCRIPT_NAME}] GAZEBO HEADLESS MODE ENABLED. ONLY THE GAZEBO SERVER WILL RUN."
            ENABLE_HEADLESS_MODE=1
        fi 
        
    elif [ "${RUN_MODE}x" == "debugx" ]; then
        EchoGreen "[${SCRIPT_NAME}] DEBUG MODE ENABLED FOR GAZEBO-CLASSIC CONTAINER"
        RUN_SCRIPT="debug.sh"
    else
        EchoRed "[${SCRIPT_NAME}] INVALID INPUT \"${RUN_MODE}\". PLEASE CHECK THE ARGUMENT."
        EchoBoxLine
        exit 1
    fi

    CheckFileExists ${GAZEBO_CLASSIC_WORKSPACE}/scripts/${RUN_SCRIPT}
    CheckFileExecutable ${GAZEBO_CLASSIC_WORKSPACE}/scripts/${RUN_SCRIPT}

    if [ -z "${SITL_WORLD}" ]; then
        sed -i "s~SITL_WORLD=\"\"~SITL_WORLD=\"empty\"~g" ${SITL_ENV_DIR}/gazebo-classic.env
    fi

    if [ -z "${SITL_AIRFRAME}" ]; then
        sed -i "s~SITL_AIRFRAME=\"\"~SITL_AIRFRAME=\"iris\"~g" ${SITL_ENV_DIR}/gazebo-classic.env
    fi

    if [ -z "${ENABLE_HEADLESS_MODE}" ]; then
        sed -i "s~GAZEBO_CLASSIC_RUN_COMMAND=\"\"~GAZEBO_CLASSIC_RUN_COMMAND=\'bash -c \"/home/user/workspace/gazebo/scripts/${RUN_SCRIPT}\"\'~g" ${SITL_ENV_DIR}/gazebo-classic.env
    else
        sed -i "s~GAZEBO_CLASSIC_RUN_COMMAND=\"\"~GAZEBO_CLASSIC_RUN_COMMAND=\'bash -c \"/home/user/workspace/gazebo/scripts/${RUN_SCRIPT} ${ENABLE_HEADLESS_MODE}\"\'~g" ${SITL_ENV_DIR}/gazebo-classic.env
    fi
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

SetRunModeAirSim() {
    # FUNCTION TO SET THE AIRSIM CONTAINER IN RUN MODE
    # >>>---------------------------------------------------------
    # INPUTS:
    # $1: $0 (MAIN SCRIPT RUNNING THE FUNCTION)
    # $2: RUN MODE (gazebo-classic-airsim-sitl, auto, .sh)
    # ------------------------------------------------------------
    # EXAMPLE:
    # SetRunModeAirSim $0 gazebo-classic-airsim-sitl
    # ------------------------------------------------------------
    SCRIPT_NAME=$(basename "$1")
    RUN_MODE=$2

    if [ "${RUN_MODE}x" == "gazebo-classic-airsim-sitlx" ]; then
        RUN_SCRIPT="auto.sh"
    elif [ "${RUN_MODE}x" == "autox" ]; then
        EchoGreen "[${SCRIPT_NAME}] RUNNING AIRSIM CONTAINER IN AUTO MODE"
        EchoGreen "[${SCRIPT_NAME}] AIRSIM CONTAINER WILL FIND AND RUN .sh FILE IN /home/ue4/workspace/binary DIRECTORY"
        RUN_SCRIPT="auto.sh"
    elif [[ "${RUN_MODE}x" == *".shx" ]]; then
        EchoGreen "[${SCRIPT_NAME}] RUNNING AIRSIM CONTAINER WITH SCRIPT ${RUN_MODE}"
        RUN_SCRIPT=${RUN_MODE}
    elif [ "${RUN_MODE}x" == "debugx" ]; then
        EchoGreen "[${SCRIPT_NAME}] DEBUG MODE ENABLED FOR AIRSIM CONTAINER"
        RUN_SCRIPT="debug.sh"
    else
        EchoRed "[${SCRIPT_NAME}] INVALID INPUT \"${RUN_MODE}\". PLEASE CHECK THE ARGUMENT."
        EchoBoxLine
        exit 1
    fi

    CheckFileExists ${AIRSIM_WORKSPACE}/scripts/${RUN_SCRIPT}
    CheckFileExecutable ${AIRSIM_WORKSPACE}/scripts/${RUN_SCRIPT}

    sed -i "s~AIRSIM_RUN_COMMAND=\"\"~AIRSIM_RUN_COMMAND=\'bash -c \"/home/ue4/workspace/scripts/${RUN_SCRIPT}\"\'~g" ${SITL_ENV_DIR}/airsim.env
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

SetRunModeROS2() {
    # FUNCTION TO SET THE ROS2 CONTAINER IN RUN MODE
    # >>>---------------------------------------------------------
    # INPUTS:
    # $1: $0 (MAIN SCRIPT RUNNING THE FUNCTION)
    # $2: RUN MODE (gazebo-classic-sitlx, gazebo-classic-airsim-sitlx, build <optional: targets>, .sh)
    # ------------------------------------------------------------
    # EXAMPLE:
    # SetRunModeROS2 $0 debug
    # ------------------------------------------------------------
    SCRIPT_NAME=$(basename "$1")
    RUN_MODE=$2

    if [ "${RUN_MODE}x" == "gazebo-classic-sitlx" ]; then
        RUN_SCRIPT="sitl-px4.sh"
    elif [ "${RUN_MODE}x" == "gazebo-classic-airsim-sitlx" ]; then
        RUN_SCRIPT="sitl-px4-airsim.sh"
    elif [[ "${RUN_MODE}x" == "build"*"x" ]]; then
        EchoGreen "[${SCRIPT_NAME}] BUILDING ROS2 PACKAGES INSIDE THE CONTAINER."
        EchoGreen "[${SCRIPT_NAME}] CONTAINER WILL BE STOPPED AFTER THE BUILD PROCESS."

        INPUT_ARGUMENTS=${RUN_MODE}
        ARGUMENT_COUNT=$(echo ${INPUT_ARGUMENTS} | wc -w)

        RUN_SCRIPT="build.sh"

        if [ ${ARGUMENT_COUNT} -gt 1 ]; then
            # IF THERE ARE MORE THAN ONE ARGUMENTS
            BUILD_TARGETS=$(echo ${INPUT_ARGUMENTS} | cut -d ' ' -f2-)
        fi
    elif [[ "${RUN_MODE}x" == *".shx" ]]; then
        EchoGreen "[${SCRIPT_NAME}] RUNNING ROS2 CONTAINER WITH SCRIPT ${RUN_MODE}"
        RUN_SCRIPT=${RUN_MODE}
    elif [ "${RUN_MODE}x" == "debugx" ]; then
        EchoGreen "[${SCRIPT_NAME}] DEBUG MODE ENABLED FOR ROS2 CONTAINER"
        RUN_SCRIPT="debug.sh"
    else
        EchoRed "[${SCRIPT_NAME}] INVALID INPUT \"${RUN_MODE}\". PLEASE CHECK THE ARGUMENT."
        EchoBoxLine
        exit 1
    fi

    CheckFileExists ${ROS2_WORKSPACE}/scripts/${RUN_SCRIPT}
    CheckFileExecutable ${ROS2_WORKSPACE}/scripts/${RUN_SCRIPT}

    # IF BUILD_TARGETS IS PRESENT
    if [ -z "${BUILD_TARGETS}" ]; then
        sed -i "s~ROS2_RUN_COMMAND=\"\"~ROS2_RUN_COMMAND=\'bash -c \"/home/user/workspace/ros2/scripts/${RUN_SCRIPT}\"\'~g" ${SITL_ENV_DIR}/ros2.env
    else
        sed -i "s~ROS2_RUN_COMMAND=\"\"~ROS2_RUN_COMMAND=\'bash -c \"/home/user/workspace/ros2/scripts/${RUN_SCRIPT} ${BUILD_TARGETS}\"\'~g" ${SITL_ENV_DIR}/ros2.env
    fi
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

SetRunModeQGC() {
    # FUNCTION TO SET THE QGroundControl CONTAINER IN RUN MODE
    # >>>---------------------------------------------------------
    # INPUTS:
    # $1: $0 (MAIN SCRIPT RUNNING THE FUNCTION)
    # $2: RUN MODE (run)
    # ------------------------------------------------------------
    # EXAMPLE:
    # SetRunModeQGC $0 run
    # ------------------------------------------------------------
    SCRIPT_NAME=$(basename "$1")
    RUN_MODE=$2

    if [ "${RUN_MODE}x" == "normalx" ]; then
        EchoGreen "[${SCRIPT_NAME}] RUNNING QGroundControl CONTAINER IN NORMAL MODE"
        sed -i "s~QGC_RUN_COMMAND=\"\"~QGC_RUN_COMMAND=\'bash -c \"/usr/local/bin/entrypoint.sh\"\'~g" ${SITL_ENV_DIR}/qgc.env
    elif [ "${RUN_MODE}x" == "debugx" ]; then
        EchoGreen "[${SCRIPT_NAME}] DEBUG MODE ENABLED FOR QGroundControl CONTAINER"
        RUN_SCRIPT="debug.sh"
        CheckFileExists ${QGC_WORKSPACE}/scripts/${RUN_SCRIPT}
        CheckFileExecutable ${QGC_WORKSPACE}/scripts/${RUN_SCRIPT}
        sed -i "s~QGC_RUN_COMMAND=\"\"~QGC_RUN_COMMAND=\'bash -c \"/home/user/workspace/scripts/${RUN_SCRIPT}\"\'~g" ${SITL_ENV_DIR}/qgc.env
    else
        EchoRed "[${SCRIPT_NAME}] INVALID INPUT \"${RUN_MODE}\". PLEASE CHECK THE ARGUMENT."
        EchoBoxLine
        exit 1
    fi
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<