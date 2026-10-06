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


# MAIN SCRIPT
# >>>-------------------------------------------------------------

if [ ! -f /home/ue4/workspace/log ]; then
    touch /home/ue4/workspace/log
else
    echo "" > /home/ue4/workspace/log
fi

CheckFileExists /home/ue4/workspace/settings.json
CheckDirExists /home/ue4/Documents/AirSim create

ln -s /home/ue4/workspace/settings.json /home/ue4/Documents/AirSim/settings.json

# CHECK IF DIRECTORY binary EXISTS
if [ ! -d /home/ue4/workspace/binary ]; then
    EchoRed "[$(basename "$0")] DIRECTORY binary DOES NOT EXIST. PLEASE CHECK IF THE DIRECTORY EXISTS."
    exit 1
else
    EchoGreen "[$(basename "$0")] DIRECTORY binary EXISTS."
    
    # CHECK IF .sh FILE EIXSTS
    if [ ! -f /home/ue4/workspace/binary/*.sh ]; then
        EchoRed "[$(basename "$0")] .sh FILE DOES NOT EXIST. KILLING THE SCRIPT."
        exit 1
    else
        EchoGreen "[$(basename "$0")] .sh FILE EXISTS."
        
        # GET NAME OF .sh FILE
        SH_FILE=$(ls /home/ue4/workspace/binary/*.sh)

        # CHECK IF SH FILE IS NOT AN ARRAY
        if [ $(echo $SH_FILE | wc -w) -gt 1 ]; then
            EchoRed "[$(basename "$0")] THERE ARE MORE THAN ONE .sh FILE. KILLING THE SCRIPT."
            EchoRed "[$(basename "$0")] PLEASE MAKE SURE THERE IS ONLY ONE .sh FILE IN THE BINARY DIRECTORY."
            exit 1
        else
            EchoGreen "[$(basename "$0")] TARGET .sh FILE: ${SH_FILE}"
            
            # CHECK IF THE .sh FILE IS EXECUTABLE
            if [ ! -x ${SH_FILE} ]; then
                EchoRed "[$(basename "$0")] ${SH_FILE} IS NOT EXECUTABLE. KILLING THE SCRIPT."
                EchoRed "[$(basename "$0")] PLEASE MAKE SURE ${SH_FILE} IS EXECUTABLE."
                exit 1
            else
                EchoGreen "[$(basename "$0")] ${SH_FILE} IS EXECUTABLE."
                EchoGreen "[$(basename "$0")] RUNNING ${SH_FILE}..."

                # RUN THE .sh FILE
                su -c "${SH_FILE} ${AIRSIM_RENDER_ARGS:--windowed}" ue4 > /home/ue4/workspace/log
            fi
        fi
    fi
fi

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<