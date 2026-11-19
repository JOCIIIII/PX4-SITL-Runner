# A4VAI 2세부 SILS·PILS 시뮬레이터 (PX4-SITL-Runner)

무인이동체 자율비행 시뮬레이터입니다. PX4(비행제어), Gazebo Classic(물리), Unreal 실증 시나리오 시험맵(센서·환경),
ROS2 Humble(자율비행 알고리즘)을 도커로 묶어 이륙부터 착륙까지 자동으로 비행합니다.

- **SILS**: 알고리즘을 이 PC에서 돌립니다.
- **PILS**: 같은 알고리즘을 기체용 동반 컴퓨터(Jetson)에서 돌리고, 시뮬레이션만 이 PC에서 합니다.
- 실기체 비행에 쓰는 스크립트와 비행 프로필도 같은 저장소에 있습니다(`scripts/dev/onboard_real.sh`).

---

## 1. 요구사항

| 항목 | 요구 |
|---|---|
| OS | Ubuntu 22.04 LTS (x86_64) |
| GPU | NVIDIA GPU + 드라이버(Vulkan 지원, `nvidia-smi` 동작). Unreal Engine 설치는 필요 없습니다 |
| 메모리 | 16 GB 이상 (32 GB 권장) |
| 디스크 | 여유 60 GB 이상 (도커 이미지 약 24 GB + PX4 빌드 약 12 GB + 그 외) |
| 기타 | 그래픽 세션(디스플레이), 인터넷, sudo 권한, `python3` |

PILS를 하려면 Jetson Orin(JetPack 6, L4T R36)이 추가로 필요합니다(5장).

## 2. 사전 준비 (1회)

```bash
sudo ubuntu-drivers autoinstall && sudo reboot      # NVIDIA 드라이버가 이미 있으면 건너뜀
git clone https://github.com/JOCIIIII/PX4-SITL-Runner.git
cd PX4-SITL-Runner
./scripts/setup-host.sh                             # Docker + NVIDIA Container Toolkit
```

`setup-host.sh`가 "docker 그룹 추가"를 안내하면 로그아웃 후 다시 로그인하세요.

## 3. 설치

```bash
./scripts/install.sh
```

설치 단계의 세부는 [docs/MANUAL.md](docs/MANUAL.md)에 있습니다.

실증 시나리오 시험맵의 Unreal 패키지(약 400 MB)는 [A4VAI-Unreal 릴리스 `A4VAI-Sim-v1.0`](https://github.com/JOCIIIII/A4VAI-Unreal/releases/tag/A4VAI-Sim-v1.0)(A4VAI Sim v1.0)에서
자동으로 받습니다. 인터넷이 안 되는 곳에서는 릴리스의 파일 3개를 미리 받아 `~/Downloads/A4VAI-Sim-v1.0/`에
넣어 두면 다운로드 대신 그걸 씁니다(다른 폴더면 `UNREAL_ASSET_DIR=<폴더>`).

| 파일 | 내용 |
|---|---|
| `A4VAISIM-Linux.tar.gz` | 실증 시나리오 시험맵·Yard 맵 패키징 빌드 A4VAI Sim v1.0 (A4VAI-Unreal d316eb2) |
| `scenario-gazebo-prebuilt.tar.gz` | GazeboDrone + Gazebo 플러그인 |
| `SHA256SUMS` | 두 파일의 체크섬 (설치가 확인함) |

도커 이미지 → PX4 빌드(X8·X500 모델 포함) → 알고리즘 소스(`A4VAI-Algorithms-ROS2` `inhousesim-v1.0`) 빌드 →
AirSim ROS2 브리지 빌드 → 실증 시나리오 시험맵(Unreal 패키징 빌드) 순서로 자동 설치합니다.
중간에 실패하면 **다시 실행하면 이어서** 진행합니다. 작업 폴더는 `~/Documents/A4VAI-SITL`입니다.

## 4. 실행: 실증 시나리오 시험맵 (SILS)

```bash
./scripts/scenario.sh profiles               # 비행 프로필 목록
./scripts/scenario.sh fly x8_5s first_flight  # X8, 학습 회피 정책, WP3 → GOAL, 높이 1.7 m
./scripts/scenario.sh fly x500_3 x500_test   # X500 3 m/s
./scripts/scenario.sh stop                   # 시뮬레이터와 컨테이너 종료
```

- `fly`는 프로필의 기체(X8/X500)에 맞는 라이다 설정으로 Unreal을 띄우고(이미 맞게 떠 있으면 그대로 씁니다),
  Gazebo/PX4와 ROS2 앱을 띄워 자동으로 이륙 → 회피 비행 → 목표 착륙까지 합니다. 한 번에 약 2–3분입니다.
- 출발·도착을 바꾸려면: `./scripts/scenario.sh fly x8_5s run2 WP3 GOAL 1.7` (레이아웃 웨이포인트 이름 또는 `X,Y` 좌표, 높이 m).
- 캠퍼스 마당 맵(남→북 코스, 10-17 검증): `SCENARIO_MAP=Yard ./scripts/scenario.sh fly x500_3 yard1 2,-78 40,1 2.0`
- 결과: `~/Documents/A4VAI-SITL/ROS2/test_runs/<이름>/` (PX4 로그 `px4_sitl.ulg`, 회피 기록 `noclimb_ca.csv`, 노드 로그)

```bash
python3 scripts/dev/scenario_clearance.py ~/Documents/A4VAI-SITL/ROS2/test_runs/first_flight  # 장애물 최소 여유
python3 scripts/dev/shake_metrics.py   ~/Documents/A4VAI-SITL/ROS2/test_runs/first_flight   # 롤·피치 흔들림
```

(분석 도구는 호스트의 `numpy`, `pyulog`가 필요합니다: `pip3 install --user numpy pyulog`)

### 비행 프로필

`scripts/dev/flight_profiles/*.env` 한 파일이 기체·정책·손잡이를 모두 정합니다. **PILS와 실기체도 같은 파일을 씁니다.**
설명과 SILS/PILS 결과는 `scripts/dev/flight_profiles/README.md`에 있습니다.
명령 앞에 `NOCLIMB_*=값`을 붙이면 프로필 값보다 우선합니다(예: `NOCLIMB_CMD_ACCEL_MAX=2.5 ./scripts/scenario.sh fly x500_3 t1`).

### 시험: 단위·쌍·통합

경로계획(PP)·경로추종(PF)·충돌회피(CA)를 하나씩, 둘씩, 셋 모두 시험합니다. 충돌회피는 `TEST_CA=inha|jbnu|super`(인하대 / 전북대 T1000 / SUPER)로 고릅니다.
명령, 시험별 구성, 합격 기준, 옵션은 [docs/MANUAL.md](docs/MANUAL.md)의 "시험" 절에 있습니다.

```bash
TEST_CA=super ./scripts/scenario.sh test pf-ca t1  # 시험: pp | pf | ca | pp-pf | pf-ca | all
./scripts/scenario.sh test table                   # 셀마다 최근 결과
```

## 5. 실행: PILS (Jetson)

알고리즘을 Jetson에서 돌리고 이 PC는 시뮬레이션만 합니다. Jetson은 JetPack 6(L4T R36)이어야 합니다.

```bash
# Jetson 준비 (1회, ssh 키 로그인 필요, 수십 분–2시간)
./scripts/dev/jetson_base_setup.sh jetson@<Jetson 주소>                                   # 기반 이미지
PILS_BASE_IMAGE=a4vai-base:humble ./scripts/dev/pils_jetson_setup.sh jetson@<Jetson 주소>  # 소스 복사·빌드
./scripts/dev/onboard_livox_setup.sh jetson@<Jetson 주소>                                 # (실기체용) Mid-360 드라이버

# 비행
ONBOARD_IP=<Jetson 주소> ONBOARD_SSH=jetson@<Jetson 주소> ./scripts/scenario.sh pils x8_5s pils1
```

- 같은 LAN이면 LAN 주소를, 원격(Tailscale 100.x)이면 그 주소를 씁니다. 100.x 주소면 원격 링크 모드로 자동 전환됩니다.
- 알고리즘이나 스크립트를 고친 뒤에는 `pils_jetson_setup.sh`를 다시 실행하세요(파이썬만 바꿨으면 `PILS_SKIP_BUILD=1`).

## 6. 제거

```bash
./scripts/uninstall.sh    # 컨테이너·작업공간 삭제 (도커 이미지 삭제는 선택)
```

## 7. 문제가 생기면

| 증상 | 조치 |
|---|---|
| `fly`가 "Unreal API (41451) not up" | `~/Documents/A4VAI-SITL/logs/scenario_unreal.log` 확인. NVIDIA 드라이버·Vulkan 문제가 대부분 |
| `Pool overlaps with other one` | `./scripts/scenario.sh stop` (시뮬레이터 네트워크 정리) 후 다시 실행 |
| 비행 결과가 `airsim_frozen` | Gazebo→Unreal 위치 전달이 멈춘 것. 같은 명령을 다시 실행 (하네스가 3번까지 자동 재시도함) |
| 고친 스크립트가 반영 안 됨 | `~/Documents/A4VAI-SITL/` 쪽 사본은 실행할 때마다 덮어써집니다. **이 저장소에서** 고치세요 |
| 설치 로그 | `~/Documents/A4VAI-SITL/logs/install_<시각>.log` |

## 구성 저장소 (모두 `inhousesim-v1.0`)

| 저장소 | 내용 |
|---|---|
| [PX4-SITL-Runner](https://github.com/JOCIIIII/PX4-SITL-Runner) | 이 저장소(A4VAI 2세부 SILS·PILS 시뮬레이터): 설치·실행·PILS·실기체 스크립트, 기체 모델, 비행 프로필 |
| [A4VAI-Algorithms-ROS2](https://github.com/JOCIIIII/A4VAI-Algorithms-ROS2) | ROS2 알고리즘: 학습 회피 정책, flight_driver, PathFollowing·SUPER(서브모듈), T1000(전북대) |
| [A4VAI-Unreal](https://github.com/JOCIIIII/A4VAI-Unreal) | 실증 시나리오 시험맵·Yard 맵 (UE 5.8 + Cosys-AirSim 3.5). 릴리스 `A4VAI-Sim-v1.0`에 패키징 빌드 |

## License

MIT — [LICENSE](./LICENSE)
