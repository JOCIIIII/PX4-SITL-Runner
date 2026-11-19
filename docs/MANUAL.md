# A4VAI 2세부 SILS·PILS 시뮬레이터 설치 및 실행 매뉴얼

> 기본 흐름(실증 시나리오 시험맵, PILS)은 `README.md` 4–5장에 있습니다. 이 문서는 설치 세부입니다.

## 요구사항

- Ubuntu 22.04 (x86_64), NVIDIA GPU + **드라이버**(사전 설치: `sudo ubuntu-drivers autoinstall` 후 재부팅), 디스플레이(그래픽 세션 필수)
- 디스크 여유 ~60GB (도커 이미지 ~35GB + PX4/ROS2 빌드)
- Docker/Compose/NVIDIA Container Toolkit은 아래 `setup-host.sh`가 설치해준다

## 설치 (명령 2개)

```bash
git clone https://github.com/JOCIIIII/PX4-SITL-Runner.git && cd PX4-SITL-Runner
./scripts/setup-host.sh     # 호스트 사전준비: Docker + NVIDIA Container Toolkit (sudo, 1회)
./scripts/install.sh        # 시뮬레이터 설치 (docker 그룹 추가됐다면 재로그인 후)
```

도커 이미지 pull(전부 Docker Hub 공개) → PX4 v1.16.0 빌드 → ROS2 소스 클론(서브모듈 포함) →
AirSim ROS2 브리지(Cosys-AirSim 5.4-v3.2) 소스 빌드 → ROS2 워크스페이스 빌드 →
리소스·AirSim 바이너리(release 자동 다운로드) → 실증 시나리오 시험맵(A4VAI-Unreal 릴리스의 Unreal 패키징 빌드,
SHA256 확인; `~/Downloads/A4VAI-Sim-v1.0`에 릴리스 파일 사본을 넣어 두면 그걸 씀)까지 전부 자동.
각 단계는 멱등이라 실패 시 재실행하면 이어서 진행된다.

## 제거

```bash
./scripts/uninstall.sh     # 컨테이너·네트워크·워크스페이스 삭제, 이미지는 선택
```

## 모니터링

- **Foxglove Studio** → `ws://localhost:8765` (`/cloud`, `/rog_map/occ`, `/fsm/path`, `/odom` 등)
- 로그: `~/Documents/A4VAI-SITL/ROS2/logs/*.log` (노드별)
- 설치/실행 터미널 전체 출력: `~/Documents/A4VAI-SITL/logs/install_*.log`, `run_*.log` (자동 저장, 문제 보고 시 첨부). 실행 종료 시 노드별 로그도 `run_*_ros2/` 로 복사됨
- QGC 창에서 기체 상태/모드 확인

## 자주 쓰는 명령

```bash
./scripts/run.sh <svc> debug              # 개별 컨테이너 쉘 (px4|ros2|airsim|...)
./scripts/run.sh px4 build                # PX4 재빌드
# ROS2 재빌드 (devel 이미지 — runtime 이미지에는 컴파일러가 없음):
docker run --rm --entrypoint bash -v ~/Documents/A4VAI-SITL/ROS2:/home/user/workspace/ros2 \
  jociiiii/a4vai:inhousesim-v0.9 -c 'source /opt/ros/humble/setup.bash && cd /home/user/workspace/ros2/ros2_ws && colcon build --symlink-install'
```

## 트러블슈팅

| 증상 | 조치 |
|---|---|
| `Pool overlaps with other one` | 다른 프로젝트가 172.70.0.0/16 점유 — 해당 네트워크 제거 또는 `scripts/include/commonEnv.sh`의 `SITL_NETWORK_SUBNET` 변경 |
| `container name already in use` | `docker rm -f px4-env gz-sim airsim-binary ros2-env qgc-app` |
| airsim_node가 `invalid allocator`로 종료 | 브리지가 구버전 빌드 — `rm -rf ~/Documents/A4VAI-SITL/ROS2/cosys-airsim/ros2/{build,install}` 후 install.sh 재실행 |
| 시험맵 `fly`가 Unreal을 못 띄움 | `~/Documents/A4VAI-SITL/logs/ctrack_unreal.log` 확인, `./scripts/ctrack.sh stop` 후 재시도 |
| 수정한 env/스크립트가 반영 안 됨 | `~/Documents/A4VAI-SITL/` 쪽은 매 실행마다 저장소 사본으로 덮어써짐 — **저장소에서 수정**할 것 |
