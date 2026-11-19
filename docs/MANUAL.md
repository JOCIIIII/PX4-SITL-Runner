# A4VAI 2세부 SILS·PILS 시뮬레이터 설치 및 실행 매뉴얼

> 기본 흐름(실증 시나리오 시험맵, PILS)은 `README.md` 4–5장에 있습니다. 이 문서는 설치 세부와 시험(단위·쌍·통합) 사용법입니다.

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

## 시험: 단위·쌍·통합

실증 시나리오 시험맵에서 경로계획(PP)·경로추종(PF)·충돌회피(CA)를 하나씩(단위), 둘씩(PP-PF, PF-CA), 셋 모두(통합) 시험합니다.
충돌회피는 세 가지 중에서 고르고, 경로추종은 PathFollowing(MPPI)입니다.

### 명령

```bash
TEST_CA=<회피> ./scripts/scenario.sh test <시험> <이름> [프로필]
./scripts/scenario.sh test table        # 셀마다 가장 최근 결과
```

- `<시험>`: `pp`, `pf`, `ca`, `pp-pf`, `pf-ca`, `all`
- `TEST_CA`: `inha`(인하대 학습 정책, 기본) · `jbnu`(전북대 T1000) · `super`(SUPER)
- `<프로필>`: 기체와 인하대 정책(기본 `x8_5sm2`; 목록은 `./scripts/scenario.sh profiles`)
- 시뮬레이터가 하나라 한 번에 하나만 돌립니다. Unreal은 자동으로 뜹니다.

```bash
./scripts/scenario.sh test pp t_pp
./scripts/scenario.sh test pf t_pf
TEST_CA=jbnu  ./scripts/scenario.sh test ca  t_ca_jbnu
./scripts/scenario.sh test pp-pf t_pppf
TEST_CA=super ./scripts/scenario.sh test pf-ca t_pfca_super
TEST_CA=inha  ./scripts/scenario.sh test all  t_all_inha
```

### 시험별 구성

| 시험 | 코스 | 날리는 것 |
|---|---|---|
| `pp` | 시나리오 A 경로 START → WP1 (천막 회피) | Hybrid PP 계획만, 비행 없음 |
| `pf` | 시나리오 B 직선, 바람 구역 | PathFollowing이 정해진 경로를 비행 |
| `ca` | 시나리오 C, WP3 → GOAL (기둥·RC카·나무 벽·풍선) | 회피 알고리즘의 기본 구성: 인하대·SUPER는 PathFollowing을 거치고, 전북대는 T1000이 PX4에 속도 명령을 직접 보냄 |
| `pp-pf` | 시나리오 A | Hybrid PP 경로를 PathFollowing이 비행 |
| `pf-ca` | 시나리오 C 구간 WP3 → WP5 → WP6 → GOAL | 회피 명령을 PathFollowing이 비행 |
| `all` | 시나리오 D | Hybrid PP 경로를 PathFollowing이 WP3까지(B 코스 원·사각·나선 포함), 이후 회피 알고리즘이 WP5 → WP6 → GOAL |

| `TEST_CA` | 앱 (`scripts/ros2/apps/`) | 비고 |
|---|---|---|
| `inha` | `noclimb_ca.sh` | 프로필의 정책(`NOCLIMB_POLICY_DIR`) |
| `jbnu` | `t1000.sh`(`ca`), `t1000_pf.sh`(그 밖) | 전북대 T1000 Control 445 |
| `super` | `super_pf.sh` | PathFollowing MPPI (`SUPER_PF_GUID=2`) |

`all`에서 전북대·SUPER는 `prefix_relay_node.py`가 계획 경로를 먼저 PathFollowing에 주고, 끝점 근처에서 회피 알고리즘에 넘깁니다.

### 합격 기준

| 시험 | 기준 |
|---|---|
| `pp` | 경로 찾음, 최단 경로의 120 % 이하, 계획 1초 이하 (RFP 기준은 Jetson에서 잽니다) |
| `pf`, `pp-pf` | 경로 끝에 착륙, 경로 오차 p95 1 m 이하·최대 3 m 이하 |
| `ca`, `pf-ca` | 접촉 없음(모든 장애물과 프로펠러 끝 여유 > 0), 목표 2 m 안 착륙 |
| `all` | 계획 경로 구간 p95 1 m·최대 3 m 이하, 접촉 없음, 목표 2 m 안 착륙 |

### 옵션

| 변수 | 기본 | 뜻 |
|---|---|---|
| `TEST_SPEED` | 5 m/s | 경로추종 속도 (`pf`, `pp-pf`, `all`의 계획 경로) |
| `TEST_WIND` | 5 m/s | 바람 구역 세기, 0이면 끔 (`pf`, `all`에만) |
| `TEST_ALT` | 1.7 m | 비행 높이 (지면 위) |
| `TEST_TIMEOUT` | 600 s | 한 비행의 제한 시간 |

`ca`·`pf-ca`와 `all`의 회피 구간은 각 알고리즘의 기본 속도(최대 5 m/s)로 납니다.
`scripts/dev/scenario_demo.sh`의 `DEMO_*` 손잡이와 프로필 값(`NOCLIMB_*` 등)도 명령 앞에 붙여 바꿀 수 있습니다.

### 결과 보기

- `~/Documents/A4VAI-SITL/ROS2/test_runs/<이름>/test_summary.txt`: 합격·불합격과 기준별 값
- 같은 폴더의 `demo_summary.txt`(경로 오차, 장애물별 최근접 거리), `px4_sitl.ulg`(PX4 로그), 노드 로그
- `test_runs/test_matrix.csv`: 모든 실행 기록. `./scripts/scenario.sh test table`이 셀마다 최근 결과를 표로 보여 줍니다.

### 알아 둘 점

- RC카는 계속 움직이므로 같은 시험도 카트와 마주치는 시점에 따라 결과가 달라질 수 있습니다. 성공률은 같은 시험을 여러 번 돌려서 보세요.
- `pp`·`pp-pf`·`all`은 Hybrid PP 라이브러리가 필요합니다. 휠 파일은 이 저장소에 없으니 경로계획 담당에게 받아
  `scripts/dev/scenario_pp.sh --install <휠 파일>`로 한 번 설치하세요(GPU 필요).
- 끝나면 `./scripts/scenario.sh stop`으로 시뮬레이터를 닫습니다.

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
| 시험맵 `fly`가 Unreal을 못 띄움 | `~/Documents/A4VAI-SITL/logs/scenario_unreal.log` 확인, `./scripts/scenario.sh stop` 후 재시도 |
| 수정한 env/스크립트가 반영 안 됨 | `~/Documents/A4VAI-SITL/` 쪽은 매 실행마다 저장소 사본으로 덮어써짐 — **저장소에서 수정**할 것 |
