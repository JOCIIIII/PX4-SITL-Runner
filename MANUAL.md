# A4VAI SITL 설치 및 실행 매뉴얼

## 요구사항

- Ubuntu 22.04 (x86_64), NVIDIA GPU + 드라이버, 디스플레이(그래픽 세션 필수)
- Docker + Docker Compose 플러그인, NVIDIA Container Toolkit, git
- 디스크 여유 ~60GB (도커 이미지 ~35GB + PX4/ROS2 빌드)

## 설치 (명령 1개)

```bash
git clone <이 저장소> && cd PX4-SITL-Runner
./scripts/install.sh
```

도커 이미지 pull(전부 Docker Hub 공개) → PX4 v1.16.0 빌드 → ROS2 소스 클론(서브모듈 포함) →
AirSim ROS2 브리지(Cosys-AirSim 5.4-v3.2) 소스 빌드 → ROS2 워크스페이스 빌드 →
리소스·AirSim 바이너리(release 자동 다운로드)까지 전부 자동.
각 단계는 멱등이라 실패 시 재실행하면 이어서 진행된다.

## 실행 (명령 2개)

```bash
./scripts/run-super.sh      # ① SUPER + PathFollowing — 라이다 맵핑·충돌회피 비행
./scripts/run-pf-test.sh    # ② PathFollowing 단독 — 고정 웨이포인트 추종 (플래너 없음)
```

둘 다 PX4+Gazebo+AirSim+ROS2+QGC 전체 스택을 띄우고 자동으로 이륙→비행→착륙한다.
정지: `./scripts/stop.sh gazebo-classic-airsim-sitl`

- ① goal 변경: `~/Documents/A4VAI-SITL/ROS2/ros2_ws/src/realgazebo_fastplanner_adapters/config/goals_super_far.csv` (ENU x,y,z)
- ② 웨이포인트 변경: `~/Documents/A4VAI-SITL/ROS2/ros2_ws/src/path_following_test/wp.csv` (NED x,y,고도) — 수정 후 재실행만 하면 됨

## 제거

```bash
./scripts/uninstall.sh     # 컨테이너·네트워크·워크스페이스 삭제, 이미지는 선택
```

## 모니터링

- **Foxglove Studio** → `ws://localhost:8765` (`/cloud`, `/rog_map/occ`, `/fsm/path`, `/odom` 등)
- 로그: `~/Documents/A4VAI-SITL/ROS2/logs/*.log` (노드별)
- QGC 창에서 기체 상태/모드 확인

## 자주 쓰는 명령

```bash
./scripts/run.sh <svc> debug              # 개별 컨테이너 쉘 (px4|ros2|airsim|...)
./scripts/run.sh px4 build                # PX4 재빌드
# ROS2 재빌드 (devel 이미지 — runtime 이미지에는 컴파일러가 없음):
docker run --rm --entrypoint bash -v ~/Documents/A4VAI-SITL/ROS2:/home/user/workspace/ros2 \
  jociiiii/a4vai:devel -c 'source /opt/ros/humble/setup.bash && cd /home/user/workspace/ros2/ros2_ws && colcon build --symlink-install'
```

## 트러블슈팅

| 증상 | 조치 |
|---|---|
| `Pool overlaps with other one` | 다른 프로젝트가 172.70.0.0/16 점유 — 해당 네트워크 제거 또는 `scripts/include/commonEnv.sh`의 `SITL_NETWORK_SUBNET` 변경 |
| `container name already in use` | `docker rm -f px4-env gz-sim airsim-binary ros2-env qgc-app` |
| AirSim이 `libApex...so` 에러로 종료 | binary 패키지 불완전 — `Engine/` 포함 전체 패키지로 교체 |
| airsim_node가 `invalid allocator`로 종료 | 브리지가 구버전 빌드 — `rm -rf ~/Documents/A4VAI-SITL/ROS2/cosys-airsim/ros2/{build,install}` 후 install.sh 재실행 |
| 수정한 env/스크립트가 반영 안 됨 | `~/Documents/A4VAI-SITL/` 쪽은 매 실행마다 저장소 사본으로 덮어써짐 — **저장소에서 수정**할 것 |
