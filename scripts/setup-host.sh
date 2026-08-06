#!/bin/bash

# HOST PREREQUISITE SETUP (Ubuntu 22.04) - run ONCE before ./scripts/install.sh
# Installs: Docker CE + compose plugin, NVIDIA Container Toolkit, base tools.
# Does NOT install the NVIDIA GPU driver (kernel module + reboot involved) -
# install it first via 'sudo ubuntu-drivers autoinstall' + reboot if needed.

set -e

BASE_DIR=$(dirname $(readlink -f "$0"))
source ${BASE_DIR}/include/commonFcn.sh

# ---- 0. GPU DRIVER CHECK (guide only) --------------------------------------
if ! command -v nvidia-smi >/dev/null || ! nvidia-smi >/dev/null 2>&1; then
    EchoRed "[setup-host.sh] NVIDIA 드라이버가 동작하지 않습니다."
    EchoRed "[setup-host.sh] 먼저 드라이버를 설치/재부팅 후 다시 실행하세요:"
    EchoRed "[setup-host.sh]   sudo ubuntu-drivers autoinstall && sudo reboot"
    exit 1
fi
EchoGreen "[setup-host.sh] NVIDIA 드라이버 확인: $(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)"

# ---- 1. BASE TOOLS ----------------------------------------------------------
EchoGreen "[setup-host.sh] [1/3] 기본 도구 설치"
sudo apt-get update -qq
sudo apt-get install -y -qq git wget curl unzip ca-certificates gnupg

# ---- 2. DOCKER CE + COMPOSE PLUGIN -----------------------------------------
if command -v docker >/dev/null && docker compose version >/dev/null 2>&1; then
    EchoYellow "[setup-host.sh] [2/3] Docker + compose 이미 설치됨 - 건너뜀"
else
    EchoGreen "[setup-host.sh] [2/3] Docker CE 설치 (공식 저장소)"
    sudo install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
        sudo gpg --dearmor --yes -o /etc/apt/keyrings/docker.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo ${VERSION_CODENAME}) stable" | \
        sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
    sudo apt-get update -qq
    sudo apt-get install -y -qq docker-ce docker-ce-cli containerd.io docker-compose-plugin
fi
# docker 그룹 (sudo 없이 docker 사용)
if ! id -nG "$USER" | grep -qw docker; then
    sudo usermod -aG docker "$USER"
    NEED_RELOGIN=1
fi

# ---- 3. NVIDIA CONTAINER TOOLKIT -------------------------------------------
if docker info 2>/dev/null | grep -qi nvidia || dpkg -l nvidia-container-toolkit >/dev/null 2>&1; then
    EchoYellow "[setup-host.sh] [3/3] NVIDIA Container Toolkit 이미 설치됨 - 건너뜀"
else
    EchoGreen "[setup-host.sh] [3/3] NVIDIA Container Toolkit 설치"
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
        sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -sL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
        sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
        sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
    sudo apt-get update -qq
    sudo apt-get install -y -qq nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
fi

EchoGreen "[setup-host.sh] 완료."
if [ -n "${NEED_RELOGIN}" ]; then
    EchoYellow "[setup-host.sh] docker 그룹이 추가되었습니다 - 로그아웃/로그인(또는 재부팅) 후"
    EchoYellow "[setup-host.sh] ./scripts/install.sh 를 실행하세요."
else
    EchoGreen "[setup-host.sh] 이어서 ./scripts/install.sh 를 실행하세요."
fi
