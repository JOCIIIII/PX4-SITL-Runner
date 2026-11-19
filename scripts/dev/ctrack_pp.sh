#!/bin/bash
# Hybrid PP on the C-Track field (scripts/dev/ctrack_pp.py) inside the ROS2 image, on the GPU (the planner's
# visibility graph is CUDA-only: cupy).
#   scripts/dev/ctrack_pp.sh OUT_DIR START,WP1 [ctrack_pp.py options, e.g. --alt 1.7 --radius 1.2]
#   scripts/dev/ctrack_pp.sh --install [WHEEL]   once: hybrid-learning-path-planning 0.3.2 + numba 0.62.1 +
#       cupy-cuda12x 13.6.0 into ~/Documents/A4VAI-SITL/ROS2/pp_libs (the image's numpy 2.2 / scipy / onnxruntime
#       stay as they are). WHEEL: the lab's wheel, default the copy in the RealGazebo hybrid_pp_ros2_port folder.
# CTRACK_LAYOUT picks another layout (default: the C-Track layout of the A4VAI-Unreal folder, as ctrack_super.sh).
set -e
REPO=$(dirname $(dirname $(dirname $(readlink -f "$0"))))
W=~/Documents/A4VAI-SITL
LIBS=$W/ROS2/pp_libs
IMAGE=$(grep '^ROS2_ENV_IMAGE=' $REPO/envs/ros2.env | cut -d= -f2 | tr -d '"')
if [ "${1:-}" = --install ]; then
    WHEEL=${2:-$HOME/Desktop/00_Project/00_A4VAI/00_Development/00_Simulator/01_RealGazebo/hybrid_pp_ros2_port_20260903_1507/hybrid_pp_ros2_port/pathplanning/vendor/hybrid_learning_path_planning-0.3.2-py3-none-any.whl}
    [ -f "$WHEEL" ] || { echo "[ctrack_pp] wheel not found: $WHEEL"; exit 1; }
    mkdir -p "$LIBS"
    docker run --rm --gpus all --user $(id -u):$(id -g) -e HOME=/tmp -v "$LIBS":/pp_libs \
        -v "$(dirname "$WHEEL")":/vendor:ro --entrypoint bash "$IMAGE" -c \
        "pip install --no-cache-dir --target /pp_libs --no-deps /vendor/$(basename "$WHEEL") numba==0.62.1 llvmlite==0.45.1 \
             cupy-cuda12x==13.6.0 fastrlock && PYTHONPATH=/pp_libs python3 -c 'import cupy, hybrid_learning_path_planning as h; \
             print(\"hybrid\", h.__version__, \"cupy\", cupy.__version__, cupy.cuda.runtime.getDeviceCount(), \"GPU\")'"
    exit 0
fi
[ $# -ge 2 ] || { sed -n 2,9p "$0"; exit 1; }
[ -d "$LIBS/hybrid_learning_path_planning" ] || { echo "[ctrack_pp] run $0 --install first"; exit 1; }
OUT=$1; shift
mkdir -p "$OUT"; OUT=$(readlink -f "$OUT")
if [ -z "${CTRACK_LAYOUT:-}" ]; then
    CTRACK_DIR=${CTRACK_DIR:-$W/A4VAI-Unreal}
    [ -d "$CTRACK_DIR" ] || CTRACK_DIR=$HOME/Desktop/00_Project/00_A4VAI/00_Development/00_Simulator/02_InhouseSim/00_Unreal/00_Map
    CTRACK_LAYOUT=$CTRACK_DIR/layout/ctrack_layout.json
fi
LAYOUT=$(readlink -f "$CTRACK_LAYOUT")
docker run --rm --gpus all --user $(id -u):$(id -g) -e HOME=/tmp -e PYTHONPATH=/pp_libs \
    -v "$LIBS":/pp_libs:ro -v "$REPO/scripts/dev":/dev_scripts:ro -v "$(dirname "$LAYOUT")":/layout:ro -v "$OUT":/out \
    --entrypoint python3 "$IMAGE" /dev_scripts/ctrack_pp.py /out "$@" --layout "/layout/$(basename "$LAYOUT")"
