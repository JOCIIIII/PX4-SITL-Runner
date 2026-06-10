#!/bin/bash

# SCRIPT TO RUN CONTAINER FOR TESTING

# INITIAL STATEMENTS
# >>>----------------------------------------------------

# SET THE BASE DIRECTORY
BASE_DIR=$(dirname $(readlink -f "$0"))
REPO_DIR=$(dirname $(dirname $(readlink -f "$0")))

# SOURCE THE ENVIRONMENT AND FUNCTION DEFINITIONS
for file in ${BASE_DIR}/include/*.sh; do
    source ${file}
done
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


# INPUT STATEMENT 1 VALIDITY CHECK
# >>>----------------------------------------------------

# DECLARE DICTIONARY OF USAGE STATEMENTS 1
## KEY: ARGUMENT, CONTENT: DESCRIPTION
declare -A usageState1
usageState1["gazebo-classic-sitl"]="DEPLOY GAZEBO SITL CONTAINER"
usageState1["gazebo-classic-airsim-sitl"]="DEPLOY GAZEBO-AIRSIM SITL CONTAINER"
usageState1["px4"]="DEPLOY PX4-AUTOPILOT CONTAINER"
usageState1["gazebo-classic"]="DEPLOY GAZEBO-CLASSIC CONTAINER"
usageState1["airsim"]="DEPLOY AIRSIM CONTAINER"
usageState1["ros2"]="DEPLOY ROS2 CONTAINER"
usageState1["qgc"]="DEPLOY QGroundControl CONTAINER"

CheckValidity $0 usageState1 1 "$@"
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

# RUN PROCESS PER ARGUMENT
# >>>----------------------------------------------------
# PERFORM BASIC SETUP
## WORKSPACE DIRECTORIES, ENVIRONMENT VARIABLES, AND DISPLAY SETTINGS
BasicSetup

if [ "$1x" == "gazebo-classic-sitlx" ]; then
    # INPUT STATEMENT 2 VALIDITY CHECK
    # >>>----------------------------------------------------
    declare -A usageState2
    usageState2["run"]="RUN PX4-AUTOPILOT SITL IN GAZEBO-CLASSIC"
    usageState2["debug"]="RUN PX4-AUTOPILOT SITL IN GAZEBO-CLASSIC IN DEBUG MODE (OPTION: SERVICE(S) TO STOP)"
    usageState2["stop"]="STOP PX4-AUTOPILOT SITL IN GAZEBO-CLASSIC IF IT IS RUNNING"

    CheckValidity $0 usageState2 2 "$@"
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    # RUN FOR EACH ARGUMENT
    # >>>----------------------------------------------------
    # ACTION: stop. STOP THE CONTAINER
    if [ "$2x" == "stopx" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"

        EchoYellow "[$(basename "$0")] THIS FEATURE IS FOR CONVENIENCE."
        EchoYellow "[$(basename "$0")] IT IS RECOMMENDED TO USE stop.sh SCRIPT TO STOP THE CONTAINER."

        ${BASE_DIR}/stop.sh $1
        exit 0
    # ACTION: debug. RUN THE CONTAINER IN DEBUG MODE (sleep infinity)
    elif [ "$2x" == "debugx" ]; then
        EchoGreen "[$(basename "$0")] RUNNING GAZEBO-CLASSIC-SITL CONTAINER(S) IN DEBUG MODE."
        # WHEN NUMBER OF ARGUMENTS IS 2
        if [ $# -eq 2 ]; then
            EchoGreen "[$(basename "$0")] SETTING ALL CONTAINERS IN DEBUG MODE"
            SetRunModePX4 $0 debug
            SetRunModeGazeboClassic $0 debug
            SetRunModeROS2 $0 debug
            SetRunModeQGC $0 debug
        else
            # FOR EACH ARGUMENT STARTING FROM THE THIRD ARGUMENT, SET THE COMMAND TO THE .env FILE
            for arg in "${@:3}"; do
                if [ "${arg}x" == "px4x" ]; then
                    SetRunModePX4 $0 debug
                elif [ "${arg}x" == "gazebo-classicx" ]; then
                    SetRunModeGazeboClassic $0 debug
                elif [ "${arg}x" == "ros2x" ]; then
                    SetRunModeROS2 $0 debug
                elif [ "${arg}x" == "qgcx" ]; then
                    SetRunModeQGC $0 debug
                else
                    EchoRed "[$(basename "$0")] INVALID INPUT \"$arg\". PLEASE USE ARGUMENT AMONG"
                    EchoRed "*   \"px4\""
                    EchoRed "*   \"gazebo-classic\""
                    EchoRed "*   \"ros2\""
                    EchoRed "*   \"qgc\""
                    EchoRed "TO STOP THE CONTAINER SELECTIVELY OR LEAVE IT EMPTY TO STOP EVERYTHING."
                    exit 1
                fi
            done
        fi

        SetRunModePX4 $0 gazebo-classic-sitl
        SetRunModeGazeboClassic $0 gazebo-classic-sitl
        SetRunModeROS2 $0 gazebo-classic-sitl
        SetRunModeQGC $0 normal
    # ACTION: run. RUN THE PX4-GAZEBO-CLASSIC SITL SIMULATION
    elif [ "$2x" == "runx" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"

        SetRunModePX4 $0 gazebo-classic-sitl
        SetRunModeGazeboClassic $0 gazebo-classic-sitl
        SetRunModeROS2 $0 gazebo-classic-sitl
        SetRunModeQGC $0 normal
    fi
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

elif [ "$1x" == "gazebo-classic-airsim-sitlx" ]; then
    # INPUT STATEMENT 2 VALIDITY CHECK
    # >>>----------------------------------------------------
    declare -A usageState2
    usageState2["run"]="RUN PX4-AUTOPILOT SITL IN GAZEBO-CLASSIC"
    usageState2["debug"]="RUN PX4-AUTOPILOT SITL IN GAZEBO-CLASSIC IN DEBUG MODE (OPTION: SERVICE(S) TO STOP)"
    usageState2["test"]="UNIT TEST INDIVIDUAL ALGORITHMS IN GAEBZO-CLASSIC-AIRSIM SITL ENVORONMENT"
    usageState2["stop"]="STOP PX4-AUTOPILOT SITL IN GAZEBO-CLASSIC IF IT IS RUNNING"
    
    CheckValidity $0 usageState2 2 "$@"
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    # RUN FOR EACH ARGUMENT
    # >>>----------------------------------------------------
    # ACTION: stop. STOP THE CONTAINER
    if [ "$2x" == "stopx" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"

        EchoYellow "[$(basename "$0")] THIS FEATURE IS FOR CONVENIENCE."
        EchoYellow "[$(basename "$0")] IT IS RECOMMENDED TO USE stop.sh SCRIPT TO STOP THE CONTAINER."

        ${BASE_DIR}/stop.sh $1
        exit 0
    # ACTION: debug. SET CONTAINERS IN DEBUG MODE
    elif [ "$2x" == "debugx" ]; then
        EchoGreen "[$(basename "$0")] RUNNING GAZEBO-CLASSIC-SITL CONTAINERs IN DEBUG MODE."
        # WHEN NUMBER OF ARGUMENTS IS 2. SET ALL CONTAINERS IN DEBUG MODE
        if [ $# -eq 2 ]; then
            EchoGreen "[$(basename "$0")] SETTING ALL CONTAINERS IN DEBUG MODE"
            SetRunModePX4 $0 debug
            SetRunModeGazeboClassic $0 debug
            SetRunModeAirSim $0 debug
            SetRunModeROS2 $0 debug
            SetRunModeQGC $0 debug
        # IF NOT, SET THE DEBUG MODE FOR THE SELECTED CONTAINERS
        else
            # FOR EACH ARGUMENT STARTING FROM THE THIRD ARGUMENT, SET THE COMMAND TO THE .env FILE
            for arg in "${@:3}"; do
            echo $arg
                if [ "${arg}x" == "px4x" ]; then
                    SetRunModePX4 $0 debug
                elif [ "${arg}x" == "gazebo-classicx" ]; then
                    SetRunModeGazeboClassic $0 debug
                elif [ "${arg}x" == "airsimx" ]; then
                    SetRunModeAirSim $0 debug
                elif [ "${arg}x" == "ros2x" ]; then
                    SetRunModeROS2 $0 debug
                elif [ "${arg}x" == "qgcx" ]; then
                    SetRunModeQGC $0 debug
                else
                  CLASSIC SITL SIMULATION  EchoRed "*   \"gazebo-classic\""
                    EchoRed "*   \"airsim\""
                    EchoRed "*   \"ros2\""
                    EchoRed "*   \"qgc\""
                    EchoRed "TO STOP THE CONTAINER SELECTIVELY OR LEAVE IT EMPTY TO STOP EVERYTHING."
                    exit 1
                fi
            done
        fi

        SetRunModePX4 $0 gazebo-classic-airsim-sitl
        SetRunModeGazeboClassic $0 gazebo-classic-airsim-sitl
        SetRunModeAirSim $0 gazebo-classic-airsim-sitl
        SetRunModeROS2 $0 gazebo-classic-airsim-sitl
        SetRunModeQGC $0 normal
    # ACTION: run. RUN THE PX4-GAZEBO-CLASSIC-AIRSIM SITL SIMULATION
    elif [ "$2x" == "runx" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"

        SetRunModePX4 $0 gazebo-classic-airsim-sitl
        SetRunModeGazeboClassic $0 gazebo-classic-airsim-sitl
        SetRunModeAirSim $0 gazebo-classic-airsim-sitl
        SetRunModeROS2 $0 gazebo-classic-airsim-sitl
        SetRunModeQGC $0 normal
    # ACTION: test. UNIT TEST INDIVIDUAL ALGORITHMS IN GAZEBO-CLASSIC-AIRSIM SITL ENVIRONMENT
    elif [ "$2x" == "testx" ]; then
        # INPUT STATEMENT 3 VALIDITY CHECK
        # >>>----------------------------------------------------
        declare -A usageState3
        usageState3["path-planning"]="RUNNING PATH PLANNIG UNIT TEST"
        usageState3["path-following"]="RUNNING PATH FOLLOWING UNIT TEST"
        usageState3["collision-avoidance"]="RUNNING COLLISION AVOIDANCE UNIT TEST"
        usageState3["pf-ca-integration"]="RUNNING PATH FOLLOWING AND COLLISION AVOIDANCE INTEGRATION TEST"
        usageState3["pp-pf-integration"]="RUNNING PATH PLANNING AND PATH FOLLOWING INTEGRATION TEST"

        CheckValidity $0 usageState3 3 "$@"
        # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        # RUN FOR EACH ARGUMENT
        # >>>----------------------------------------------------
        # ACTION: path-planning. RUN PATH PLANNING UNIT TEST
        if [ "$3x" == "path-planningx" ]; then
            # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
            LimitNumArgument $0 3 "$@"
            EchoGreen "[$(basename "$0")] RUNNING PATH PLANNING UNIT TEST"
            SetRunModePX4 $0 debug
            SetRunModeGazeboClassic $0 debug
            SetRunModeAirSim $0 debug
            SetRunModeROS2 $0 path-planning-unit-test.sh
            SetRunModeQGC $0 debug

        # ACTION: path-following. RUN PATH FOLLOWING UNIT TEST
        elif [ "$3x" == "path-followingx" ]; then
            # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
            LimitNumArgument $0 3 "$@"
            EchoGreen "[$(basename "$0")] RUNNING PATH FOLLOWING UNIT TEST"
            SetRunModePX4 $0 gazebo-classic-airsim-sitl
            SetRunModeGazeboClassic $0 gazebo-classic-airsim-sitl
            SetRunModeAirSim $0 gazebo-classic-airsim-sitl
            SetRunModeROS2 $0 path-following-unit-test.sh
            SetRunModeQGC $0 normal
        
        # ACTION: collision-avoidance. RUN COLLISION AVOIDANCE UNIT TEST
        elif [ "$3x" == "collision-avoidancex" ]; then
            # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
            LimitNumArgument $0 3 "$@"
            EchoGreen "[$(basename "$0")] RUNNING COLLISION AVOIDANCE UNIT TEST"
            SetRunModePX4 $0 gazebo-classic-airsim-sitl
            SetRunModeGazeboClassic $0 gazebo-classic-airsim-sitl
            SetRunModeAirSim $0 gazebo-classic-airsim-sitl
            SetRunModeROS2 $0 collision-avoidance-unit-test.sh
            SetRunModeQGC $0 normal
        
        # ACTION: pf-ca-integation. RUN PATH FOLLOWING AND COLLISION AVOIDANCE INTEGRATION TEST
        elif [ "$3x" == "pf-ca-integrationx" ]; then
            # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
            LimitNumArgument $0 3 "$@"
            EchoGreen "[$(basename "$0")] RUNNING PATH FOLLOWING AND COLLISION AVOIDANCE INTEGRATION TEST"
            SetRunModePX4 $0 gazebo-classic-airsim-sitl
            SetRunModeGazeboClassic $0 gazebo-classic-airsim-sitl
            SetRunModeAirSim $0 gazebo-classic-airsim-sitl
            SetRunModeROS2 $0 ca-pf-integration-test.sh
            SetRunModeQGC $0 normal

        # ACTION: pp-pf-integration. RUN PATH PLANNING AND PATH FOLLOWING INTEGRATION TEST
        elif [ "$3x" == "pp-pf-integrationx" ]; then
            # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
            LimitNumArgument $0 3 "$@"
            EchoGreen "[$(basename "$0")] RUNNING PATH FOLLOWING AND COLLISION AVOIDANCE INTEGRATION TEST"
            SetRunModePX4 $0 gazebo-classic-airsim-sitl
            SetRunModeGazeboClassic $0 gazebo-classic-airsim-sitl
            SetRunModeAirSim $0 gazebo-classic-airsim-sitl
            SetRunModeROS2 $0 pp-pf-integration-test.sh
            SetRunModeQGC $0 normal
        fi
        # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    fi
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
elif [ "$1x" == "px4x" ]; then
    # INPUT STATEMENT 2 VALIDITY CHECK
    # >>>----------------------------------------------------
    declare -A usageState2
    usageState2["clone"]="CLONE PX4-AUTOPILOT REPOSITORY"
    usageState2["build"]="BUILD PX4-AUTOPILOT INSIDE THE CONTAINER"
    usageState2["clean"]="CLEAN PX4-AUTOPILOT BUILD"
    usageState2["sitl-gazebo-classic-standalone"]="RUN PX4-AUTOPILOT SITL IN GAZEBO-CLASSIC IN STANDALONE MODE (ON ONE CONTAINER)"
    usageState2["debug"]="RUN PX4-AUTOPILOT CONTAINER IN DEBUG MODE (sleep infinity)"
    usageState2["stop"]="STOP PX4-AUTOPILOT CONTAINER IF IT IS RUNNING"
    
    CheckValidity $0 usageState2 2 "$@"
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    # RUN FOR EACH ARGUMENT
    # >>>----------------------------------------------------
    # ACTION: stop. STOP THE CONTAINER
    if [ "$2x" == "stopx" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"

        EchoYellow "[$(basename "$0")] THIS FEATURE IS FOR CONVENIENCE."
        EchoYellow "[$(basename "$0")] IT IS RECOMMENDED TO USE stop.sh SCRIPT TO STOP THE CONTAINER."

        ${BASE_DIR}/stop.sh $1
        exit 0
    # ACTIONS: debug. RUN THE CONTAINER IN DEBUG MODE
    elif [ "$2x" == "debugx" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"
        SetRunModePX4 $0 debug
    elif [ "$2x" == "clonex" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"
        SetRunModePX4 $0 clone
    elif [ "$2x" == "buildx" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"
        SetRunModePX4 $0 build
    elif [ "$2x" == "cleanx" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"
        SetRunModePX4 $0 clean
    elif [ "$2x" == "sitl-gazebo-classic-standalonex" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"
        SetRunModePX4 $0 sitl-gazebo-classic-standalone
    fi
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
elif [ "$1x" == "gazebo-classicx" ]; then
    # INPUT STATEMENT 2 VALIDITY CHECK
    # >>>----------------------------------------------------
    declare -A usageState2
    usageState2["sitl-px4"]="RUN PX4-AUTOPILOT SITL IN GAZEBO-CLASSIC"
    usageState2["debug"]="RUN GAZEBO-CLASSIC CONTAINER IN DEBUG MODE (sleep infinity)"
    usageState2["stop"]="STOP GAZEBO-CLASSIC CONTAINER IF IT IS RUNNING"
    
    CheckValidity $0 usageState2 2 "$@"
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    # RUN FOR EACH ARGUMENT
    # >>>----------------------------------------------------
    # ACTION: stop. STOP THE CONTAINER
    if [ "$2x" == "stopx" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"

        EchoYellow "[$(basename "$0")] THIS FEATURE IS FOR CONVENIENCE."
        EchoYellow "[$(basename "$0")] IT IS RECOMMENDED TO USE stop.sh SCRIPT TO STOP THE CONTAINER."

        ${BASE_DIR}/stop.sh $1
        exit 0
    # ACTION: debug. RUN THE CONTAINER IN DEBUG MODE
    elif [ "$2x" == "debugx" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"
        SetRunModeGazeboClassic $0 debug
    elif [ "$2x" == "sitl-px4x" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"
        SetRunModeGazeboClassic $0 sitl-px4
    fi
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
elif [ "$1x" == "airsimx" ]; then
    # INPUT STATEMENT 2 VALIDITY CHECK
    # >>>----------------------------------------------------
    declare -A usageState2
    usageState2["debug"]="RUN AIRSIM CONTAINER IN DEBUG MODE (sleep infinity)"
    usageState2["stop"]="STOP AIRSIM CONTAINER IF IT IS RUNNING"
    usageState2["auto"]="RUN AIRSIM CONTAINER IN AUTO MODE (run .sh file in /home/ue4/workspace/binary)"
    usageState2["*.sh"]="RUN AIRSIM CONTAINER IN MANUAL MODE (run specific .sh file)"
    
    CheckValidity $0 usageState2 2 "$@"
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    # RUN FOR EACH ARGUMENT
    # >>>----------------------------------------------------
    # ACTION: stop. STOP THE CONTAINER
    if [ "$2x" == "stopx" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"

        EchoYellow "[$(basename "$0")] THIS FEATURE IS FOR CONVENIENCE."
        EchoYellow "[$(basename "$0")] IT IS RECOMMENDED TO USE stop.sh SCRIPT TO STOP THE CONTAINER."

        ${BASE_DIR}/stop.sh $1
        exit 0
    # ACTION: debug. RUN THE CONTAINER IN DEBUG MODE
    elif [ "$2x" == "debugx" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"
        SetRunModeAirSim $0 debug
    # ACTION: auto. RUN THE CONTAINER IN AUTO MODE (run .sh file in /home/ue4/workspace/binary)
    elif [ "$2x" == "autox" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"
        SetRunModeAirSim $0 auto
    # ACTION: *.sh. RUN THE CONTAINER IN MANUAL MODE (run specific .sh file)
    elif [[ "$2x" == *".shx" ]]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"
        SetRunModeAirSim $0 $2
    fi
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
elif [ "$1x" == "ros2x" ]; then
    # INPUT STATEMENT 2 VALIDITY CHECK
    # >>>----------------------------------------------------
    declare -A usageState2
    usageState2["debug"]="RUN ROS2 CONTAINER IN DEBUG MODE (sleep infinity)"
    usageState2["stop"]="STOP ROS2 CONTAINER IF IT IS RUNNING"
    usageState2["build"]="BUILD ROS2 PACKAGES (build = all workspaces | build <ws> = clean-build a workspace | build <ws> <pkg...> = build only those packages)"
    usageState2["*.sh"]="RUN ROS2 CONTAINER IN MANUAL MODE (run specific shell script)"
    
    CheckValidity $0 usageState2 2 "$@"
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    # RUN FOR EACH ARGUMENT
    # >>>----------------------------------------------------
    # ACTION: stop. STOP THE CONTAINER
    if [ "$2x" == "stopx" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"

        EchoYellow "[$(basename "$0")] THIS FEATURE IS FOR CONVENIENCE."
        EchoYellow "[$(basename "$0")] IT IS RECOMMENDED TO USE stop.sh SCRIPT TO STOP THE CONTAINER."

        ${BASE_DIR}/stop.sh $1
        exit 0
    # ACTION: debug. RUN THE CONTAINER IN DEBUG MODE
    elif [ "$2x" == "debugx" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"
        SetRunModeROS2 $0 debug
    # ACTION: build. BUILD ROS2 PACKAGES INSIDE THE CONTAINER
    elif [ "$2x" == "buildx" ]; then
        # IF ADDITIONAL DIRECTORIES ARE PROVIDED, PASS THEM TO THE BUILD SCRIPT
        if [ $# -ge 3 ]; then
            # DUE TO SED ESCAPE ISSUE, ADDITIONAL ENVIRONMENT VARIABLE IS SET
            TARGET_ROS2_WORKSPACES=${@:3}
            SetRunModeROS2 $0 "build ${TARGET_ROS2_WORKSPACES}"
        # ELSE, RUN THE BUILD SCRIPT. THIS WILL BUILD ALL PACKAGES IN THE ALL DIRECTORIES THAT HAVE NON-EMPTY 'src' SUBDIRECTORY
        else
            SetRunModeROS2 $0 "build"
        fi
    # ACTION: *.sh. RUN THE CONTAINER IN MANUAL MODE (RUN SPECIFIC SHELL SCRIPT)
    elif [[ "$2x" == *".shx" ]]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"
        SetRunModeROS2 $0 $2
    fi
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
elif [ "$1x" == "qgcx" ]; then
    # INPUT STATEMENT 2 VALIDITY CHECK
    # >>>----------------------------------------------------
    declare -A usageState2
    usageState2["debug"]="RUN QGroundControl CONTAINER IN DEBUG MODE (sleep infinity)"
    usageState2["stop"]="STOP QGroundControl CONTAINER IF IT IS RUNNING"
    usageState2["run"]="RUN QGroundControl CONTAINER IN NORMAL MODE"
    
    CheckValidity $0 usageState2 2 "$@"
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    # RUN FOR EACH ARGUMENT
    # >>>----------------------------------------------------
    # ACTION: stop. STOP THE CONTAINER
    if [ "$2x" == "stopx" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"

        EchoYellow "[$(basename "$0")] THIS FEATURE IS FOR CONVENIENCE."
        EchoYellow "[$(basename "$0")] IT IS RECOMMENDED TO USE stop.sh SCRIPT TO STOP THE CONTAINER."

        ${BASE_DIR}/stop.sh $1
        exit 0
    # ACTION: debug. RUN THE CONTAINER IN DEBUG MODE (sleep infinity)
    elif [ "$2x" == "debugx" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"
        SetRunModeQGC $0 debug
    elif [ "$2x" == "runx" ]; then
        # DO NOT ALLOW ADDITIONAL ARGUMENTS FOR THIS ACTION
        LimitNumArgument $0 2 "$@"
        SetRunModeQGC $0 normal
    fi
    # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
fi

(cd ${SITL_DEPLOY_DIR} && \
docker compose -f ${SITL_DEPLOY_DIR}/compose.yml \
    --env-file ./envs/common.env \
    --env-file ./envs/px4.env \
    --env-file ./envs/gazebo-classic.env \
    --env-file ./envs/airsim.env \
    --env-file ./envs/ros2.env \
    --env-file ./envs/qgc.env \
    --profile $1 up)

# AFTER A ros2 BUILD, build.sh EXITS AND THE CONTAINER STOPS ON ITS OWN; CLEAN IT UP.
if [ "$1x" == "ros2x" ] && [ "$2x" == "buildx" ]; then
    EchoGreen "[$(basename "$0")] BUILD FINISHED - STOPPING ROS2 CONTAINER."
    ${BASE_DIR}/stop.sh ros2
fi

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
