#!/bin/bash

# FUNCTION DEFINITIONS FOR OTHER RUN SCRIPTS

EchoGreen(){
    # FUNCTION TO ECHO INPUT IN GREEN
    # >>>---------------------------------------------------------
    # INPUTS:
    # $1: INPUT TO ECHO
    # ------------------------------------------------------------
    echo -e "\e[32m$1\e[0m"
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

EchoRed(){
    # FUNCTION TO ECHO INPUT IN RED
    # >>>---------------------------------------------------------
    # INPUTS:
    # $1: INPUT TO ECHO
    # ------------------------------------------------------------
    echo -e "\e[31m$1\e[0m"
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

EchoYellow(){
    # FUNCTION TO ECHO INPUT IN YELLOW
    # >>>---------------------------------------------------------
    # INPUTS:
    # $1: INPUT TO ECHO
    # ------------------------------------------------------------
    echo -e "\e[33m$1\e[0m"
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

EchoBoxLine(){
    # FUNCTION TO PRINT A LINE OF BOXLINES
    # >>>---------------------------------------------------------
    echo $(printf '%.s─' $(seq 1 $(tput cols)))
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

CheckDirExists(){
    # FUNCTION TO CHECK THE EXISTENCE OF THE DIRECTORY
    # >>>---------------------------------------------------------
    # INPUTS:
    # $1: DIRECTORY TO CHECK
    # $2: CREATE DIRECTORY IF NOT EXISTS (true/false, default: false)
    # ------------------------------------------------------------

    usage(){
        EchoRed "Usage: $0 [directory] (create|git) [repository] (branch)"
        EchoRed "create: Create the directory if not exists (optional)"
        EchoRed "git: Clone the repository if not exists (MANDATORY - only for git)"
        EchoRed "repository: Repository to clone (optional - only for git)"
        EchoBoxLine
        exit 1
    }

    if [ $# -eq 0 ]; then
        EchoRed "[$(basename "$0")] NO ARGUMENT PROVIDED."
        usage $0
    else
        if [ "$2x" != "createx" ] && [ "$2x" != "gitx" ] && [ "$2x" != "x" ]; then
            EchoRed "[$(basename "$0")] INVALID INPUT. PLEASE ONLY USE \"create\" OR \"git\" FOR THE SECOND ARGUMENT."
            EchoBoxLine
            exit 1
        elif [ "$2x" == "gitx" ] && [ $# -lt 3 ]; then
            EchoRed "[$(basename "$0")] REPOSITORY IS REQUIRED FOR THE git ARGUMENT."
            EchoBoxLine
            exit 1
        fi 
    fi

    # SET FLAG IF create IS PROVIDED
    if [ "$2x" == "createx" ]; then
        FLAG_CREATE=true
    else
        FLAG_CREATE=false
    fi

    # # SET FLAG IF git IS PROVIDED
    if [ "$2x" == "gitx" ]; then
        FLAG_GIT=true
        GIT_REPO=$3
        # SET BRANCH IF PROVIDED
    else
        FLAG_GIT=false
    fi

    # CHECK IF THE DIRECTORY EXISTS
    if ! [ -d "$1" ]; then
        EchoRed "[$(basename "$0")] DIRECTORY \"$1\" DOES NOT EXIST."
        # CREATE THE DIRECTORY IF FLAG_CREATE IS TRUE
        if [ "$FLAG_CREATE" == true ]; then
            EchoYellow "[$(basename "$0")] THE DIRECTORY \"$1\" WILL BE CREATED."
            mkdir -p $1
        elif [ "$FLAG_GIT" == true ]; then
            EchoYellow "[$(basename "$0")] CLONING THE REPOSITORY TO \"$1\"."
            if [ $# -eq 4 ]; then
                git clone $3 -b $4 --recursive $1
            else
                git clone $3 $1
            fi
        else
            EchoRed "[$(basename "$0")] PLEASE CHECK IF THE DIRECTORY \"$1\" EXISTS."
            exit 1
        fi
    else
        EchoYellow "[$(basename "$0")] DIRECTORY \"$1\" ALREADY EXISTS."
    fi

}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

CheckDirEmpty(){
    # FUNCTION CHECK IF THE DIRECTORY IS EMPTY
    # >>>---------------------------------------------------------
    # INPUTS:
    # $1: DIRECTORY TO CHECK
    # ------------------------------------------------------------

    usage(){
        EchoRed "Usage: $0 [directory] (delete)"
        EchoRed "create: Delete the directory if empty (optional)"
        EchoBoxLine
        exit 1
    }

    if [ $# -eq 0 ]; then
        EchoRed "[$(basename "$0")] NO ARGUMENT PROVIDED."
        usageState1 $0
    else
        if [ "$2x" != "deletex" ] && [ "$2x" != "x" ]; then
            EchoRed "[$(basename "$0")] INVALID INPUT. PLEASE ONLY USE \"delete\" FOR THE SECOND ARGUMENT."
            EchoBoxLine
            exit 1
        fi 
    fi

    # SET FLAG IF delete IS PROVIDED
    if [ "$2x" == "deletex" ]; then
        FLAG_DELETE=true
    else
        FLAG_DELETE=false
    fi


    # CHECK IF THE DIRECTORY IS EMPTY
    if [ -z "$(ls -A $1)" ]; then
        EchoRed "[$(basename "$0")] DIRECTORY \"$1\" IS EMPTY."
        if [ "$FLAG_DELETE" == true ]; then
            EchoYellow "[$(basename "$0")] THE DIRECTORY \"$1\" WILL BE DELETED."
            rm -rf $1
        else
            EchoRed "[$(basename "$0")] PLEASE CHECK IF CONTENTS OF THE DIRECTORY \"$1\" ARE REMOVED."
            EchoBoxLine
            exit 1
        fi
    fi
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

CheckFileExists(){
    # FUCNTION TO CHECK IF THE FILE EXISTS
    # >>>---------------------------------------------------------
    # INPUTS:
    # $1: FILE TO CHECK
    # ------------------------------------------------------------

    usage(){
        EchoRed "Usage: $0 [directory] (cp) [fileToCopy]"
        EchoRed "cp: Copy the file if not exists (optional)"
        EchoRed "fileToCopy: File to copy (Must be Provided - only for cp)"
        EchoBoxLine
        exit 1
    }

    if [ $# -eq 0 ]; then
        EchoRed "[$(basename "$0")] NO ARGUMENT PROVIDED."
        usageState1 $0
    else
        if [ "$2x" != "cpx" ] && [ "$2x" != "x" ]; then
            EchoRed "[$(basename "$0")] INVALID INPUT. PLEASE ONLY USE \"cp\" FOR THE SECOND ARGUMENT."
            EchoBoxLine
            exit 1
        elif [ "$2x" == "cpx" ]; then
            if [ $# -lt 3 ]; then
                EchoRed "[$(basename "$0")] FILE TO COPY IS REQUIRED FOR THE cp ARGUMENT."
                EchoBoxLine
                exit 1
            elif ! [ -f "$3" ]; then
                EchoRed "[$(basename "$0")] PLEASE CHECK IF THE FILE \"$1\" EXISTS."
                EchoBoxLine
                exit 1
            fi
        fi 
    fi

    # SET FLAG IF cp IS PROVIDED
    if [ "$2x" == "cpx" ]; then
        FLAG_COPY=true
    else
        FLAG_COPY=false
    fi

    # CHECK IF THE FILE EXISTS
    if ! [ -f "$1" ]; then
        EchoRed "[$(basename "$0")] FILE \"$1\" DOES NOT EXIST."
        if [ "$FLAG_COPY" == true ]; then
            EchoYellow "[$(basename "$0")] COPYING FILE \"$3\" TO \"$1\"."
            cp $3 $1
        else
            EchoRed "[$(basename "$0")] PLEASE CHECK IF THE FILE \"$1\" EXISTS."
            EchoBoxLine
            exit 1
        fi
    fi
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

CheckFileExecutable(){
    # FUNCTION TO CHECK IF THE FILE IS EXECUTABLE
    # >>>---------------------------------------------------------
    # INPUTS:
    # $1: FILE TO CHECK
    # $2: MAKE FILE EXECUTABLE (optional)
    # ------------------------------------------------------------
    usage(){
        EchoRed "Usage: $0 [file] (x)"
        EchoRed "x: Make file executable (optional)"
        EchoBoxLine
        exit 1
    }

    if [ $# -eq 0 ]; then
        EchoRed "[$(basename "$0")] NO ARGUMENT PROVIDED."
        usageState1 $0
    else
        if [ "$2x" != "xx" ] && [ "$2x" != "x" ]; then
            EchoRed "[$(basename "$0")] INVALID INPUT. PLEASE ONLY USE \"x\" FOR THE SECOND ARGUMENT."
            EchoBoxLine
            exit 1
        fi 
    fi

    # SET FLAG IF x IS PROVIDED
    if [ "$2x" == "xx" ]; then
        FLAG_EXEC=true
    else
        FLAG_EXEC=false
    fi

    # CHECK IF THE FILE IS EXECUTABLE
    if ! [ -x "$1" ]; then
        EchoRed "[$(basename "$0")] FILE \"$1\" IS NOT EXECUTABLE."
        if [ "$FLAG_EXEC" == true ]; then
            EchoYellow "[$(basename "$0")] MAKING FILE \"$1\" EXECUTABLE."
            chmod +x $1
        else
            EchoRed "[$(basename "$0")] PLEASE CHECK IF THE FILE \"$1\" IS EXECUTABLE."
            EchoBoxLine
            exit 1
        fi
    fi
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

SetComposeDisplay(){
    # FUNCTION TO SET DISPLAY-RELATED COMPOSE ENVIRONMENT VARIABLES
    # >>>---------------------------------------------------------
    # INPUTS:
    # $1: FILE TO MODIFY
    # ------------------------------------------------------------

    usage(){
        EchoRed "Usage: $0 [file]"
        EchoBoxLine
        exit 1
    }

    if [ $# -eq 0 ]; then
        EchoRed "[$(basename "$0")] NO ARGUMENT PROVIDED."
        usageState1 $0
    fi

    # CHECK IF THE FILE EXISTS
    CheckFileExists $1

    # READ LINE TO BE CHANGED FIRST
    DISPLAY_LINE=$(grep -E '_DISPLAY=""' $1 | grep -v "WAYLAND_DISPLAY")
    WAYLAND_DISPLAY_LINE=$(grep '_WAYLAND_DISPLAY=""' $1)
    PULSEAUDIO_LINE=$(grep '_PULSEAUDIO_DIR=""' $1)

    if [ -z "${DISPLAY_LINE}" ]; then
        EchoRed "[$(basename "$0")] *_DISPLAY LINE DOES NOT EXIST."
        exit 1
    elif [ -z "${WAYLAND_DISPLAY_LINE}" ]; then
        EchoRed "[$(basename "$0")] *_WAYLAND_DISPLAY DOES NOT EXIST."
        exit 1
    elif [ -z "${PULSEAUDIO_LINE}" ]; then
        EchoRed "[$(basename "$0")] *_PULSEAUDIO_DIR DOES NOT EXIST."
        exit 1
    fi

    EchoGreen "[$(basename "$0")] [DISPLAY SETUP]"

    EchoGreen "[$(basename "$0")] ALLOWING CONNECTION TO X SERVER FROM DOCKER."
    xhost +local:docker > /dev/null 2>&1

    # SET DISPLAY OR WAYLAND_DISPLAY. IF NO DISPLAY, KILL THE SCRIPT.
    # CASE 1: DISPLAY IS SET (X11)
    if [ ! -z "${DISPLAY}" ]; then
        EchoGreen "[$(basename "$0")] \${DISPLAY} IS SET."
        EchoGreen "[$(basename "$0")] SETTING VARIABLE \"DISPLAY\" AS \"${DISPLAY}\"."
        sed -i "s~${DISPLAY_LINE}~$(echo ${DISPLAY_LINE} | sed "s~\"\"~\"${DISPLAY}\"~g")~g" $1
    # CASE 2: WAYLAND_DISPLAY IS SET (WAYLAND)
    elif [ ! -z "${WAYLAND_DISPLAY}" ]; then
        EchoYellow "[$(basename "$0")] \${WAYLAND_DISPLAY} IS SET."
        EchoYellow "[$(basename "$0")] IT IS RECOMMENDED TO USE X11 DISPLAY."
        EchoYellow "[$(basename "$0")] SETTING VARIABLE \"WAYLAND_DISPLAY\" AS \"${WAYLAND_DISPLAY}\"."
        sed -i "s~${WAYLAND_DISPLAY_LINE}~$(echo ${WAYLAND_DISPLAY_LINE} | sed "s~\"\"~\"${WAYLAND_DISPLAY}\"~g")~g" $1
    # CASE3: NO ANY DISPLAY IS SET (NON-GRAPHICAL)
    else
        EchoRed "[$(basename "$0")] ANY DISPLAY IS NOT SET."
        EchoRed "[$(basename "$0")] ARE YOU RUNNING THIS SCRIPT IN A GRAPHICAL ENVIRONMENT?"
        EchoRed "[$(basename "$0")] IF SO, PLEASE CHECK YOUR DISPLAY SETTINGS."
        exit 1
    fi

    # SET PULSEAUDIO SOCKET
    sed -i "s~${PULSEAUDIO_LINE}~$(echo ${PULSEAUDIO_LINE} | sed "s~\"\"~\"${XDG_RUNTIME_DIR}/pulse\"~g")~g" $1

    EchoBoxLine
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

CheckValidity(){
    # FUNCTION TO CHECK THE VALIDITY OF INPUT ARGUMENTS
    # >>>---------------------------------------------------------
    # INPUTS:
    # $1: SCRIPT NAME
    # $2: DICTIONARY OF ARGUMENTS
    # $3: ORDER OF ARGUMENT TO CHECK
    # $4: INPUT ARGUMENTS
    # ------------------------------------------------------------
    # EXAMPLE:
    # CheckValidity $0 usageState 1 "$@"
    # ------------------------------------------------------------

    # FUNCTION TO PRINT USAGE STATEMENTS
    usageState(){
        EchoRed "Usage:"
        for key in "${!ARGUMENTS_DICT[@]}"; do
            EchoRed "*   ${key}: ${ARGUMENTS_DICT[${key}]}"
        done
        EchoBoxLine
        exit 1
    }

    # PARSE INPUT PARAMETERS
    ## NAME OF THE SCRIPT
    SCRIPT_NAME=$(basename "$1")
    ## DICTIONARY OF ARGUMENTS
    ### KEY: ARGUMENT, VALUE: DESCRIPTION
    local -n ARGUMENTS_DICT=$2
    ## ORDER OF ARGUMENT TO CHECK
    ARGUMENT_TO_CHECK=$3
    ## INPUT ARGUMENTS PASSSED TO THE SCRIPT
    ### CHECK FROM 3 + ARGUMENT_TO_CHECK
    INPUT_ARGUMENTS=${@:4}

    # MAIN LOGIC
    ## COUNT NUMBER OF INPUT ARGUMENTS SEPARATED BY SPACE
    NUM_INPUT_ARGUMENTS=$(echo $INPUT_ARGUMENTS | wc -w)
    NUM_INPUT_ARGUMENTS=$((${NUM_INPUT_ARGUMENTS}-${ARGUMENT_TO_CHECK}+1))
    ## GET N-th ARGUMENT
    TARGET_ARGUMET=$(echo $INPUT_ARGUMENTS | cut -d' ' -f${ARGUMENT_TO_CHECK})

    ## CHECK VALIDITY OF INPUT ARGUMENTS
    if [ ${NUM_INPUT_ARGUMENTS} -eq 0 ]; then
        ## CASE 1: NO INPUT ARGUMENT PROVIDED
        if [ ${ARGUMENT_TO_CHECK} -eq 1 ]; then
            EchoRed "[${SCRIPT_NAME}] [ARGUMENT CHECK]"
            EchoRed "[${SCRIPT_NAME}] NO INPUT ARGUMENT PROVIDED."
            usageState
        else
            EchoRed "[${SCRIPT_NAME}] [ARGUMENT CHECK]"
            EchoRed "[${SCRIPT_NAME}] NO INPUT ARGUMENT PROVIDED FOR ${TARGET_ARGUMET}."
            usageState
        fi
    else
        ## CASE 2: INPUT ARGUMENT OF help PROVIDED
        if [ "${TARGET_ARGUMET}x" == "helpx" ]; then
            usageState
        fi

        ## CASE 3: INPUT ARGUMENT PROVIDED
        ### CHECK IF THE INPUT ARGUMENT MATCHES ANY KEY IN THE DICTIONARY
        for key in "${!ARGUMENTS_DICT[@]}"; do
            if [ "${TARGET_ARGUMET}x" == "${key}x" ]; then
                ## IF MATCH, BREAK THE LOOP
                ARGUMENT_MATCHED=true
                break
            # IF KEY INCLUDES *, CHECK IF THE TARGET ARGUMENT MATCHES THE PATTERN
            elif [[ "${key}" == *"*"* ]]; then
                PATTERN=$(echo ${key} | sed 's/\*/\.\*/g')
                if [[ "${TARGET_ARGUMET}" =~ ^${PATTERN}$ ]]; then
                    ARGUMENT_MATCHED=true
                    break
                fi
            fi
            ARGUMENT_MATCHED=false
        done

        ## IF NO MATCH, PRINT ERROR MESSAGE AND KILL THE SCRIPT
        if [ "$ARGUMENT_MATCHED" == false ]; then
            EchoRed "[${SCRIPT_NAME}] [ARGUMENT CHECK]"
            EchoRed "[${SCRIPT_NAME}] INVALID INPUT \"${TARGET_ARGUMET}\". PLEASE USE ARGUMENT AMONG:"
            usageState
            EchoBoxLine
            exit 1
        else
            EchoGreen "[${SCRIPT_NAME}] [ARGUMENT CHECK]"
            EchoGreen "[${SCRIPT_NAME}] SELECTED INPUT ARGUMENT: ${TARGET_ARGUMET}"
            EchoBoxLine
        fi
    fi
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

LimitNumArgument() {
    # FUNCTION TO BLOCK ACCEPTING ADDITIONAL ARGUMENTS
    # >>>---------------------------------------------------------
    # INPUTS:
    # $1: SCRIPT NAME
    # $2: MAXIMUM NUMBER OF ARGUMENTS ALLOWED
    # $3: INPUT ARGUMENTS
    # ------------------------------------------------------------
    # EXAMPLE:
    # LimitNumArgument $0 2 "$@"
    # ------------------------------------------------------------
    SCRIPT_NAME=$(basename "$1")
    MAX_NUM_ARGUMENTS=$2
    INPUT_ARGUMENTS=${@:3}

    # GET ${MAX_NUM_ARGUMENTS}th ARGUMENT
    TARGET_ARGUMET=$(echo ${INPUT_ARGUMENTS} | cut -d' ' -f${MAX_NUM_ARGUMENTS})

    # COUNT NUMBER OF INPUT ARGUMENTS
    NUM_INPUT_ARGUMENTS=$(echo ${INPUT_ARGUMENTS} | wc -w)

    if [ ${NUM_INPUT_ARGUMENTS} -gt ${MAX_NUM_ARGUMENTS} ]; then
        # GET ${MAX_NUM_ARGUMENTS}+1th ARGUMENT
        ADDITIONAL_ARGUMENT=$(echo ${INPUT_ARGUMENTS} | cut -d' ' -f$((${MAX_NUM_ARGUMENTS}+1)))

        EchoRed "[${SCRIPT_NAME}] [ARGUMENT CHECK]"
        EchoRed "[${SCRIPT_NAME}] ADDITIONAL ARGUMENT(S) STARTING FROM \"${ADDITIONAL_ARGUMENT}\" DETECTED."
        EchoRed "[${SCRIPT_NAME}] ARGUMENT ${TARGET_ARGUMET} DOES NOT ALLOW ANY ADDITIONAL ARGUMENTS."
        EchoBoxLine
        exit 1
    fi
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

WarnAction(){
    # FUNCTION TO WARN THE USER BEFORE PROCEEDING
    # >>>---------------------------------------------------------
        read -n1 -r -p $'\e[31mARE YOU SURE? [y/n]:\e[0m: ' PROCEED

        if [ "${PROCEED}x" != "yx" ] && [ "${PROCEED}x" != "Yx" ]; then
            EchoYellow "[$(basename "$0")] ABORTED."
            EchoBoxLine
            exit 1
        fi
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

LogToFile(){
    # FUNCTION TO RE-RUN THE CALLING SCRIPT UNDER tee SO ITS WHOLE TERMINAL
    # OUTPUT IS SAVED TO ${SITL_DEPLOY_DIR}/logs/<prefix>_<timestamp>.log.
    # THE FILE IS WHAT USERS SEND US WHEN INSTALL/RUN FAILS.
    # >>>---------------------------------------------------------
    # INPUTS:
    # $1: LOG NAME PREFIX (e.g. install, run_gazebo-classic-airsim-sitl)
    # $2: CALLING SCRIPT PATH ($0)
    # $3...: CALLING SCRIPT ARGUMENTS ("$@")
    # ------------------------------------------------------------
    # EXAMPLE (put right after sourcing commonEnv.sh):
    # LogToFile install "$0" "$@"
    # ------------------------------------------------------------
    # THE RE-RUN CHILD SEES SITL_LOG_FILE SET AND RETURNS IMMEDIATELY.
    if [ -n "${SITL_LOG_FILE}" ]; then
        return 0
    fi

    local PREFIX=$1
    local SCRIPT=$2
    shift 2

    local LOG_DIR=${SITL_DEPLOY_DIR}/logs
    mkdir -p "${LOG_DIR}"
    export SITL_LOG_FILE="${LOG_DIR}/${PREFIX}_$(date +%Y%m%d_%H%M%S).log"

    EchoGreen "[$(basename "${SCRIPT}")] LOGGING THIS RUN TO ${SITL_LOG_FILE}"

    # Ctrl+C MUST STILL REACH docker compose (CHILD) FOR A CLEAN SHUTDOWN,
    # BUT NEITHER THIS SHELL NOR tee MAY DIE BEFORE THE CHILD IS DONE PRINTING.
    trap ':' INT
    bash "${SCRIPT}" "$@" 2>&1 | ( trap '' INT; exec tee "${SITL_LOG_FILE}" )
    local RC=${PIPESTATUS[0]}
    trap - INT

    EchoGreen "[$(basename "${SCRIPT}")] LOG SAVED: ${SITL_LOG_FILE} (exit code ${RC})"
    exit ${RC}
}
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
