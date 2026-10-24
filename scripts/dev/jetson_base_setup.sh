#!/bin/bash
# Build the onboard base image (scripts/dev/jetson_base/Dockerfile) on a fresh Jetson, then run
#   PILS_BASE_IMAGE=a4vai-base:humble scripts/dev/pils_jetson_setup.sh HOST     (workspace, PILS image, builds)
#   scripts/dev/onboard_livox_setup.sh HOST                                      (Mid-360 driver, the vehicle image)
#   scripts/dev/jetson_base_setup.sh user@host
# env: JETSON_BASE_IMAGE (default a4vai-base:humble). Log: HOST:/tmp/jetson_base_build.log
set -eu
HOST=${1:?usage: jetson_base_setup.sh user@host (the Jetson, e.g. jetson@100.64.0.18)}
IMG=${JETSON_BASE_IMAGE:-a4vai-base:humble}
REPO=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
ssh -o BatchMode=yes "$HOST" "mkdir -p /tmp/jetson_base"
rsync -a "$REPO/scripts/dev/jetson_base/" "$HOST:/tmp/jetson_base/"
ssh -o BatchMode=yes "$HOST" "docker build -t $IMG /tmp/jetson_base > /tmp/jetson_base_build.log 2>&1; rc=\$?; tail -n 5 /tmp/jetson_base_build.log; exit \$rc" \
    && echo "[jetson_base] $IMG built on $HOST" || { echo "[jetson_base] build failed, see $HOST:/tmp/jetson_base_build.log"; exit 1; }
