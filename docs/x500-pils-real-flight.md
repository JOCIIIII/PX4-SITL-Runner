# X500 학습 회피 — PILS와 실기체 비행 준비

2026-10-15. PX4-SITL-Runner `t1000-integration` 작업트리(미커밋). 시험 기체는 X500이고, 최종 실증은 X8에서 합니다.

## 1. 결과

### 1.1 SILS (PC 한 대)

- **조건:**
  - C-Track Unreal, WP3 → GOAL, 지면 위 1.7 m
  - X500 SITL 모델(실기체 로그로 식별, `scripts/px4/x500`)
  - Mid-360 패턴 라이다(중심 위 0.10 m)
  - 노드 설정: 누적 0.4 s/5장, 기울기 게이트 0.8 rad/s, 명령 평활 τ 0.3 s, GUID 2
  - 번들마다 1회씩 비행했습니다.

| 번들 | v_max | 프롭끝 최소 여유 | 위치 | C1 앞 0.5 m 비키기 시작 | 평균/최대 m/s | 추종비 | 안전필터 개입 |
|---|---|---|---|---|---|---|---|
| `x500_onnx` | 5 | +1.34 m | 나무벽 | 8.7 m | 3.94 / 5.05 | 0.98 | 3 |
| `x500_speed_onnx/v4` | 4 | +1.25 | 나무벽 | 8.4 | 3.33 / 3.96 | 0.91 | 1 |
| `x500_speed_onnx/v3` | 3 | +0.92 | C1 기둥 | 3.8 | 2.77 / 3.10 | 1.00 | 0 |
| `x500_speed_onnx/v2` | 2 | **−0.03** | RC카트(움직임) | 1.3 | 1.95 / 2.10 | 1.05 | 1 |
| `x500_speed_onnx/v1` | 1 | +0.43 | C1 기둥 | 1.1 | 0.98 / 1.10 | 1.00 | 0 |

- 다섯 판 모두 착륙했고, 접촉 감지는 0회입니다.
- **3 m/s 이하 판은 늦게 비킵니다.**
  - 학습 쪽 기록과 같은 현상입니다. 저속으로 이어 학습하는 동안 생긴 것입니다.
  - 학습 세션이 고치는 시험을 돌리고 있습니다(거리 기준 진로 벌점). 결과는 22:40쯤 나옵니다.
- **v2는 움직이는 RC카트에 거의 닿았습니다.**
  - 1 m 앞까지 2 m/s를 유지했습니다.
  - 움직이는 장애물은 학습하지 않았습니다.

### 1.2 PILS

- **루프백**(같은 PC, 앱은 별도 컨테이너, PX4 DDS 대상 변경, 도메인 37), `x500_onnx`: 착륙, +1.11 m(SILS +1.34 m)
- **젯슨 Orin Nano 8 GB**(JetPack 6, 25 W, LAN 192.168.219.152, 도메인 37). 같은 조건으로 판마다 1회씩, SILS → PILS입니다.

| 번들 | 프롭끝 최소 여유 | 위치 | C1 앞 비키기 시작 | 평균 m/s | 추종비 |
|---|---|---|---|---|---|
| 5 m/s | +1.34 → **+0.83** | 나무벽 | 8.7 → 4.1 m | 3.94 → 3.81 | 0.98 → 0.94 |
| v4 | +1.25 → +1.31 | 나무벽 | 8.4 → 8.5 | 3.33 → 3.27 | 0.91 → 0.88 |
| v3 | +0.92 → +0.95 | C1 / RC카트 | 3.8 → 4.7 | 2.77 → 2.75 | 1.00 → 0.98 |
| v2 | −0.03 → +0.01 | RC카트 | 1.3 → 2.6 | 1.95 → 1.93 | 1.05 → 1.05 |
| v1 | +0.43 → +0.49 | C1 | 1.1 → 1.4 | 0.98 → 0.99 | 1.00 → 1.00 |

- 다섯 판 모두 착륙했고, 접촉은 0회입니다.
- 4·3·2·1 m/s는 SILS와 같은 수준입니다.
- 5 m/s만 여유가 줄었습니다(1회).
  - 젯슨에서는 MPPI 계산이 길어지고 PF 루프가 느려집니다.
  - 그래서 롤 명령 흔들림이 SILS의 2.5배였고(롤 명령 고주파 0.81 → 2.09°), 기울기 게이트로 버린 스캔이 1 → 21장으로 늘었습니다.

**젯슨 부하(다섯 판)**

| 항목 | 젯슨 | PC(SILS) |
|---|---|---|
| CPU(6코어 평균) | 77–84 % | — |
| 메모리 | 최대 2.9 GB / 7.6 GB, 스왑 0 | — |
| 온도 / 전력 | ≤ 50.5 °C / 약 7.6 W | — |
| 정책 노드 한 스텝 계산 | 중앙 14–18 ms, p95 31–38 ms | — |
| 20 Hz 간격 | p95 55–57 ms, 60 ms 넘는 스텝 4–19회 | 최대 51 ms |
| MPPI 계산 | 중앙 8–10 ms, p95 22–30 ms, 최대 51–77 ms | 3.3 / 4.7 / 5.5 ms |
| PF 루프 | 177–204 Hz | 247 Hz |

- **프로세스별 CPU(한 코어 기준 %):**
  - 실기체에서도 도는 것: PF 58, 어댑터 44, MPPI 37, 정책 36, Agent 10, flight_manager 9. 합하면 6코어의 약 30 %입니다.
  - PILS·시험 전용: AirSim 브리지 69(라이다를 10 ms마다 받아 다시 보냄), 기록기 58, GNOME 파일 색인 43.
- **실기체에서는 PILS보다 여유가 있습니다.**
  - 단, 실기체 라이다는 10 Hz 프레임만 들어오므로 어댑터 몫은 줄어듭니다.
  - 그 대신 Livox 드라이버가 더해집니다.
- **줄일 수 있는 것:**
  - 젯슨 GNOME 색인을 끄거나 헤드리스로 부팅(시스템 설정이라 사용자 결정)
  - 브리지 라이다 주기를 늘림(PILS 전용)

## 2. PILS 구성

- **PC:**
  - Unreal(C-Track, AirSim RPC 0.0.0.0:41451)
  - `ctrack-sitl`(Gazebo + PX4 + GazeboDrone, 호스트 네트워크)
- **젯슨 `ros2-onboard` 컨테이너 하나:**
  - MicroXRCEAgent, AirSim ROS2 브리지, 라이다 어댑터
  - PF(MPPI), `noclimb_ca` 노드, `flight_manager`, 기록기
  - SILS와 같은 스크립트입니다(`scripts/ros2/sitl-px4-airsim.sh` → `apps/noclimb_ca.sh`).
- **링크는 둘뿐입니다.** 실기체와 같습니다.
  - uXRCE-DDS UDP 8888: PC의 PX4 → 젯슨 Agent. PX4는 rcS 사본에서 대상 IP를 바꿉니다. 빌드는 그대로 둡니다.
  - AirSim RPC 41451: 젯슨 브리지 → PC Unreal(라이다, 시뮬 기체 자세)
- **설치(한 번, 다시 돌려도 됨):** `scripts/dev/pils_jetson_setup.sh jetson@192.168.219.152`
  1. 이 작업트리를 `~/PX4-SITL-Runner`로 옮깁니다(미커밋 작업 포함).
  2. 앱 소스(`ros2_ws/src`, SUPER·T1000 제외)와 Cosys-AirSim 소스를 `~/Documents/A4VAI-SITL/ROS2`로 옮깁니다.
  3. 이미지 `a4vai-pils:humble`을 만듭니다.
     - 바탕은 기존 `ros2-px4-jetson:humble-a4vai-hybrid`입니다(Humble, Agent 2.4.3, onnxruntime 1.23 CPU, scipy, foxglove).
     - 브리지 의존성(image_transport, geographic_msgs, mavros_msgs)을 apt로 더합니다.
  4. 컨테이너 안에서 빌드합니다. 마운트 경로는 실행 때와 같아야 합니다(`/home/user/workspace/ros2`).
     - cv_bridge: 소스 빌드. apt판은 우분투 OpenCV 4.5를 함께 끌어와 이미지의 4.8과 섞입니다.
     - Cosys 브리지
     - px4_msgs, flight_manager, pathfollowing, 어댑터
- **비행 한 판 (PC에서):**

  ```bash
  CTRACK_MODE=pils ONBOARD_IP=192.168.219.152 ONBOARD_SSH=jetson@192.168.219.152 PILS_DOMAIN=37 \
  CTRACK_APP=noclimb_ca NOCLIMB_POLICY_DIR=x500_onnx NOCLIMB_POLICY=ca_x500 \
  NOCLIMB_MAX_TILT_RATE=0.8 NOCLIMB_SCAN_MEMORY=0.4 NOCLIMB_SCAN_MEMORY_MIN=5 SIM_LIDAR=mid360 \
  ./scripts/dev/ctrack_super.sh pils_x500_5 WP3 GOAL 1.7 5.0 480 x500
  ```

  - 하네스가 ssh로 젯슨 쪽(`scripts/dev/pils_onboard.sh`)을 켜고 끕니다.
  - 기록은 PC의 `test_runs/<이름>`으로 가져옵니다. 분석 도구는 SILS와 같습니다.
  - 젯슨의 AirSim 주소는 ONBOARD_IP로 가는 경로의 PC 주소로 자동으로 잡힙니다(LAN이면 192.168.219.177).
- **주의:**
  - **도메인:** 연구실 LAN에는 다른 ROS2 기기가 있습니다. 도메인 0을 피하고 37을 씁니다. PX4는 `ROS_DOMAIN_ID`로 `UXRCE_DDS_DOM_ID`를 맞춥니다.
  - **Agent는 2.4.3이어야 합니다.** v3.x는 Humble(Fast DDS 2.x)과 디스커버리가 안 됩니다(`~/Documents/flight-test` README).
  - **Unreal은 공용입니다.** 쓰기 전과 끝난 뒤 Unreal 세션에 알립니다.

## 3. 실기체와 PILS의 차이 — 비행 전에 할 일

| 항목 | PILS(지금) | 실기체 | 할 일 |
|---|---|---|---|
| 라이다 입력 | AirSim 브리지. 패턴 순서, 빈 샷은 NaN, 시각은 마지막 샷 | `livox_ros_driver2`(Mid-360 직결). 점마다 float64 ns 시각(라이다 시계) | **됨:** `scripts/dev/onboard_livox_setup.sh`(Livox-SDK2 이미지 + 드라이버 빌드), 설정 `scripts/ros2/livox/MID360_a4vai.json` |
| 점군 등록 자세 | AirSim 기체 자세(시뮬 진리값, 같은 시계) | PX4 EKF 자세만 있음 | **됨:** 어댑터 `deskew_mode field`. 점마다 PX4 자세로 등록하고, 기울기 게이트도 PX4 자세로 합니다. 시험 `scripts/dev/adapter_field_mode_test.py`: 5 m/s 벽 오차 0.011 m(디스큐 없이 0.5 m). **남음:** 라이다 지연(`NOCLIMB_LIDAR_LATENCY`, 지금 0.02 s 가정) 실측 |
| PX4 링크 | UDP 8888, 도메인 37 | TELEM2 시리얼 921600(지금 기체 설정: `UXRCE_DDS_CFG` 102, 도메인 0). 젯슨 쪽은 `/dev/ttyUSB0` | **됨:** `scripts/dev/onboard_real.sh agent start`. Agent는 따로 상주시키고 앱만 다시 켭니다. PX4 1.16은 Agent를 재시작하면 다시 붙지 않습니다(FC를 재부팅해야 함) |
| 젯슨 네트워크 | 유선 LAN 하나(`eno1`) | `eno1`은 Mid-360이 씁니다. Wi-Fi는 꺼져 있습니다 | 감시·ssh 경로: Wi-Fi 설정 또는 USB 이더넷 |
| 로컬 원점 | 스폰 지점 = 원점(z 0) | 실기체 로그 19개에서 이륙 전 로컬 z가 −13~−45 m(EKF2_HGT_REF GPS). 비행 중 흐름은 작음(중앙 +0.04 m, 한 번 +1.15 m, 한 번 EKF 리셋으로 333 m 점프) | **됨:** livox 모드면 노드가 켜질 때(땅 위 정지) 위치를 기준으로 지면 높이·경로 고도·`NOCLIMB_GOAL`을 잡습니다(`ref_from_start`). **남음:** 실비행 `fly`에서 flight_manager 이륙 고도·목표도 같은 기준으로 |
| 목표점 | `NOCLIMB_GOAL`(PX4 로컬 ENU) | 이륙점 기준 상대값(위 기준 적용) | — |
| 기체 값 | 질량 2.0 kg, 11인치 프롭, 라이다 중심 위 0.10 m, 다리 중심 아래 0.24 m(모두 가정) | 실측 | 사용자 확인 → 학습 세션(추종 체인·크기 재맞춤) |
| 자기 점 | 없음 | 탁상(10-15, 실제 X500 장착): 라이다 0.5 m 안 반사가 프레임당 약 2000점. 정책이 늘 0.51 m 장애물을 봄. 아래 −7~−5°는 기체에 가려짐 | **됨:** 어댑터 `self_radius_m`(기체 중심에서 수평 0.55 m, 높이 ±0.5 m 안을 지움; `NOCLIMB_SELF_RADIUS`). 탁상에서 남은 0.55 m대 점은 모니터·사람(실물)이었음. **남음:** 트인 곳에서 0.55–0.8 m에 기체 부품이 남는지 확인 |
| 라이다 방향 | — | X500의 Mid-360은 커넥터가 기수 쪽 = 라이다 +X가 뒤를 봄 | **됨:** `NOCLIMB_LIDAR_YAW_DEG` 기본 180. 탁상 확인: 뒤 파티션 1.1 m, 오른쪽 모니터 0.55 m가 맞는 방향에 나옴 |
| 시간 | 젯슨 시계 + 브리지 시각으로 닫혀 있음 | 라이다 시계 ↔ 젯슨 | 디스큐에는 프레임 안 상대 시각만 씁니다. chrony는 로그 정렬용 |

## 4. PX4 (Pixhawk 6X) 설정

- **PF는 자세 offboard로 명령합니다.** `flight_manager`가 PF 구간에서 `OffboardControlMode.attitude`를 씁니다.
  - 그래서 `MPC_XY_VEL_MAX`, `MPC_TILTMAX_AIR` 같은 위치 제어기 한계는 걸리지 않습니다.
  - 속도 상한은 정책의 `v_max`와 PF 기울기 한계 35°입니다.
  - PX4 쪽 안전장치는 지오펜스, RC 개입, 킬 스위치, 페일세이프입니다.
- 지금 값은 2026-10-02 로그(`flight-log/x500/1002/3m_최종 비행.ulg`)에서 읽었습니다.

| 파라미터 | 지금 | 권장 | 이유 |
|---|---|---|---|
| `UXRCE_DDS_CFG` / `SER_TEL2_BAUD` | 102 / 921600 | 그대로 | 젯슨 Agent 시리얼 링크 |
| `UXRCE_DDS_DOM_ID` | 0 | **37**(젯슨 `ONBOARD_DOMAIN=37`) | 연구실 LAN의 도메인 0에 다른 장비의 ROS2 노드가 있습니다(10-15 확인, `/fmu` 토픽이 잡힌 적도 있음). 젯슨이 그 망에 붙으면 남의 위치·자세가 섞입니다 |
| `COM_OF_LOSS_T` / `COM_OBL_RC_ACT` | 1.0 s / 0(Position) | 그대로 | 앱이 죽으면 1 s 뒤 Position 유지 |
| `COM_RC_OVERRIDE` / `COM_RC_STICK_OV` | 3 / 30 % | 그대로 | 스틱을 30 % 움직이면 offboard에서도 조종사가 넘겨받음 |
| `COM_RCL_EXCEPT` | 0 | 그대로 | RC가 끊기면 offboard 중에도 페일세이프 |
| `NAV_RCL_ACT` / `RTL_RETURN_ALT` | 2(Return) / 30 m | 시험장에 맞게 결정 | Return은 30 m까지 올라간 뒤 돌아옵니다. 좁은 시험장이면 Land |
| `GF_MAX_HOR_DIST` / `GF_MAX_VER_DIST` / `GF_ACTION` | 0 / 0(꺼짐) / 2(Hold) | 시험 구역 + 여유(예: 40 m / 6 m) / Hold | 지금은 지오펜스가 없습니다 |
| `RC_MAP_KILL_SW` | 7번 채널 | 비행 전 동작 확인 | — |
| `COM_FLTMODE1` / `COM_FLTMODE6` | Stabilized / Position | 그대로 | offboard에 들어갈 때 스위치를 Position에 두면, 넘겨받을 때 스위치를 한 번 움직여야 합니다(위치가 바뀔 때만 모드 전환) |
| `COM_LOW_BAT_ACT` | 0(경고만) | Return 또는 Land | 시험 중 배터리 부족 대응 |

## 5. 비행 시험 순서

**그림자 시험(비행 전):** 실제 장애물 앞에서 정책이 내는 회피 경로를 날지 않고 봅니다.
- **준비:**
  - 젯슨 `eno1`을 Mid-360에 연결하고 192.168.1.50/24로 둡니다(PC의 `livox-mid360` 프로필과 같게).
  - Pixhawk TELEM2 ↔ USB 시리얼(`/dev/ttyUSB0`)을 연결합니다.
  - 기체는 땅에 둔 채 켭니다. EKF 원점이 지면 기준이 되어야 하고, 실외에서 GPS가 잡혀야 합니다.
- **실행(젯슨):**

  ```bash
  scripts/dev/onboard_real.sh agent start        # 한 번만, 계속 켜 둠
  ONBOARD_DOMAIN=37 NOCLIMB_POLICY_DIR=x500_speed_onnx/v1 NOCLIMB_POLICY=ca_x500_v1 scripts/dev/onboard_real.sh shadow shadow_v1_a
  ```

- **보기:** 노트북 Foxglove에서 `ws://<젯슨 IP>:8765`에 붙어 `/cloud`, `/noclimb/path`, `/odom`, `/shadow/desired_speed`를 봅니다.
- **한 번 해 보기:**
  1. 기체를 들고, 기수를 15–20 m 앞 장애물 쪽으로 둡니다.
  2. `/shadow/reset`을 보냅니다. 목표가 기수 방향 30 m 앞에 놓입니다.
  3. 1 m/s 정도로 걸으면서, 경로가 몇 m 앞에서 어느 쪽으로 꺾이는지 봅니다.
  - 들고 있는 사람은 기체 뒤 1.5 m 안이라 지워집니다(`NOCLIMB_SHADOW_MASK`).
- **끝내기:** `scripts/dev/onboard_real.sh stop shadow_v1_a`. 기록은 `test_runs/`에 남습니다.
- **확인할 것:**
  - 비키기 시작 거리·방향이 PILS와 비슷한지
  - 자기 점(기체 부품)이 장애물로 잡히는지
  - 걷는 중 흔들림에서 점군이 잘 등록되는지
  - 젯슨 부하와 20 Hz 간격

0. **탁상(프롭 없이):**
   - 시리얼 링크와 `/fmu` 토픽을 확인합니다.
   - Mid-360 → 드라이버 → 어댑터 `/cloud` → 정책 출력까지 확인합니다.
   - 기록기, 젯슨 부하·온도를 봅니다.
   - 정지 점군으로 자기 점을 확인합니다.
1. **호버(Position 모드, 앱만 켜고 offboard 없이):**
   - 진동 속 라이다 점군과 자세를 봅니다.
   - 젯슨 부하를 봅니다.
2. **PF만, CA 없이:** 직선 1 m/s(`pf_test` 앱, 웨이포인트 CSV). 이륙, offboard 진입, 착륙 순서를 확인합니다.
3. **CA v1(1 m/s) + 부드러운 장애물 1개:** 폼 기둥이나 상자를 경로 정면에 둡니다. 안전 조종사가 스틱 개입과 킬 스위치를 준비합니다.
4. **v2 → v3 → v4 → 5 m/s:**
   - 다음 단계로 가는 조건: 연속 2회 착륙, 프롭끝 최소 여유 0.5 m 이상, 추종비 0.8–1.1
   - 사람이나 움직이는 물체 근처에서는 하지 않습니다(움직이는 장애물 학습 안 함).
   - 3 m/s 이하 판은 늦게 비키므로 장애물 앞에 접근 거리를 충분히 둡니다. 학습 개선판이 나오면 바꿉니다.

## 6. 남은 일 (순서대로)

1. ~~젯슨 PILS 5판~~ 끝났습니다(1.2절).
2. 기체 실측값(질량, 프롭, 라이다·다리 높이)을 학습 세션에 보냅니다.
3. 그림자 시험을 합니다(5절). 소프트웨어는 준비됐습니다.
   - 10-15 탁상 시험: 실제 Mid-360(기체 장착), 정지 상태의 가짜 PX4(`scripts/dev/fake_px4_static.py`), 도메인 37. 체인 전체가 돌았고, 노드 20 Hz·한 스텝 26 ms·젯슨 CPU 약 40 %입니다.
   - 라이다 점검: `scripts/dev/livox_bench_check.py`. 10.1 Hz, 프레임당 약 15,300점, 점별 시각 범위 0.100 s
   - 실내에서는 반사가 많아 정책 노드 메모리 점이 5만 7천 개였고, 한 스텝 72 ms(13 Hz)였습니다. 노드에 복셀 필터(`NOCLIMB_VOXEL`, livox 기본 0.1 m, 복셀마다 가장 가까운 점)를 넣어 7천 개, 18.8 ms, 20 Hz로 맞췄습니다. 정책 이미지는 6×32칸이라(1 m에서 칸 0.2 m) 영향이 없습니다.
   - 그림자 모드가 PX4의 disarm 상태를 '비행 끝'으로 보고 멈추던 문제를 고쳤습니다.
   - 젯슨 전원 모드: 지금 25W는 CPU 1.344 GHz 제한(15W는 1.498)입니다. MAXN_SUPER(`sudo nvpmodel -m 2`)는 1.728 GHz(+29 %)입니다. 사용자 결정입니다.
   - 어댑터 field 모드, 노드 그림자 모드, `onboard_real.sh`, Livox 드라이버
   - 남은 것은 젯슨 `eno1`을 라이다 쪽 IP로 바꾸는 일과 시리얼 연결입니다.
   - **10-15 야외 시도(중단):**
     - 실제 PX4 연결(CP2102 → `/dev/ttyUSB0`, Agent 세션)까지는 됐습니다. 컨테이너 사이 DDS를 위해 `--ipc host`를 추가했습니다.
     - GPS는 좋았는데도(fix 4, 위성 30개) 로컬 z가 +340 m였고 몇 분 새 11 m 흘렀습니다. 리셋도 수평 11번, 고도 8번 있었습니다. 그래서 그림자 리셋마다 지면을 다시 잡게 했습니다(드는 높이 1.2 m).
     - Pixhawk를 꽂고 뺄 때마다 휴대폰 USB 테더링이 꺼져 접속이 끊겼습니다. 다음에는 Pixhawk를 먼저 꽂고 테더링을 켜세요.
     - 전체 rosbag(`-a`)은 노드 입력을 막는 것으로 의심돼, 원본 토픽만 기록합니다.
4. 실비행용 실행 스크립트(`onboard_real.sh`에 PF·flight_manager를 켜는 `fly`)를 만듭니다. 그림자 시험 결과를 본 뒤에 합니다. PX4 EKF 리셋(z 점프)도 처리해야 합니다.
5. 4절 PX4 파라미터를 정해 기체에 넣습니다.
6. 학습 개선판(저속 늦은 회피)을 SILS → PILS로 다시 돌립니다.
