# 실기체 비행 시험 준비 (2026-10-16)

목표: X500·X8로 학습 회피(CA) 3 m/s·5 m/s 비행과 GPS 웨이포인트 비행을 합니다. 소프트웨어는 모두 준비했습니다. 아침에 라이다 장착값과 호버 추력을 재서 넣으면 됩니다.

작업트리: PX4-SITL-Runner `t1000-integration`(미커밋). 새 드라이버는 `~/Documents/A4VAI-SITL/ROS2/ros2_ws/src/flight_driver`(A4VAI-Algorithms-ROS2 작업트리, 미커밋)에 있습니다. 젯슨에는 동기화하고 빌드해 두었습니다. 네 프로필 모두 SILS와 젯슨 PILS(5G 경유)로 C-Track을 비행했습니다(5절).

## 1. 새로 만든 것

| 무엇 | 어디 | 내용 |
|---|---|---|
| 비행 드라이버 | `ros2_ws/src/flight_driver` | 실비행용 비행 FSM. flight-test의 `path_following_test`(X500 실비행 검증) 흐름을 따릅니다. 아래 2절 |
| GPS 웨이포인트 비행 앱 | `scripts/ros2/apps/mission.sh` | PF(MPPI)만, 계획기·회피 없음. 위경도 CSV를 따라 비행 |
| 비행 프로필 | `scripts/dev/flight_profiles/{x500,x8}_{3,5}.env` | 정책 번들·노드 설정·PF 실기체 값. **PILS 게이트와 실기체가 같은 파일을 읽습니다** |
| PILS 게이트 | `scripts/dev/pils_gate.sh PROFILE NAME` | 프로필 그대로 C-Track을 젯슨에서 비행 |
| 실기체 실행 | `scripts/dev/onboard_real.sh` | `fly`(CA), `mission`(GPS WP), `calib`/`tool`(보정), `state`/`go`/`land`/`stop` |
| 라이다 장착 보정 | `scripts/dev/lidar_mount_calib.py` | `shake`(지연·roll/pitch/yaw), `rest`(roll/pitch·높이), `target`(yaw 확인) |
| 호버 추력 | `scripts/dev/hover_thrust.py` | PX4 로그(`hover_thrust_estimate`) 또는 PF 로그(관측기 수직 추정)에서 |
| GPS 미션 도구 | `scripts/dev/gps_mission.py` | `check`(구간 길이·방위·거리), `from-local`/`to-local`(PX4 투영과 같음) |
| PF 실기체 값 | A4VAI-PathFollowing `node_pathfollowing.py`·`node_mppi.py` | `hover_thrust`(실측 호버 추력으로 추력 환산), `max_tilt_deg`(30°). 두 노드에 같이 |
| CA 노드 | `noclimb_ca_node.py` | `goal_topic`(드라이버가 OFFBOARD 호버 뒤 목표를 주면 시작, 높이 기준도 그때 잡음), `/noclimb/ready` |

### 1.1 PF 추력 환산 — 실기체에서 고친 것

- PF 추력 명령 = 질량 × 가속 / (모터 수 × 최대추력)입니다. 추력 명령·외란 관측기·MPPI 롤아웃에 들어가는 것은 결국 호버 추력 = m·g/(n·T_max) 하나뿐입니다.
- 지금 `quad.yaml`(2.03 kg, 7.07 N)은 호버 0.703입니다. flight-test 값(2.74 kg)으로 하면 0.951입니다. 실제 X500 호버는 **0.636–0.645**입니다(실기체 로그 24개, 10-02 로그 0.644–0.645).
- 0.951로 날면 관측기가 수직 외란을 3.3 m/s²로 추정해야 합니다. 우리 PF는 관측기를 4 m/s²로 제한하므로 여유가 거의 없습니다. flight-test PF에는 그 제한이 없었습니다.
- 그래서 `hover_thrust` 파라미터를 넣었습니다. 값을 주면 T_max를 m·g/(n·h)로 다시 잡습니다. X500 프로필은 0.645입니다. **아침 첫 비행 로그로 다시 잽니다**(라이다를 단 뒤라 무거워졌습니다).
- X8은 그대로 둡니다. `octo.yaml`이 이미 실기체 호버 PWM 1515 µs와 맞습니다.
- SITL 확인: 0.645로 날린 PF 로그의 관측기 추정은 −0.14 m/s²이고, 환산하면 호버 0.636입니다. PX4 자체 추정은 0.637로 같습니다.

## 2. 비행 드라이버 (`flight_driver`)

순서는 flight-test와 같습니다: 준비 → (운용자 go) → arm → AUTO_TAKEOFF(이륙점 지면 기준 높이) → 공중에서 OFFBOARD → 이륙 높이에서 2 s 안정 → 미션 → 착륙 → disarm. 높이와 위치는 모두 이륙점 기준입니다. PX4 로컬 원점은 실기체에서 지면에 있지 않습니다(로그상 z −13~−45 m, 10-15에는 +340 m).

미션은 세 가지입니다.

- `wp_local`: CSV `x,y,z` = 북, 동, 높이. 이륙점 기준입니다(flight-test 형식).
- `wp_gps`: CSV `lat,lon,alt`. alt는 이륙점 기준 높이입니다. PX4 기준점(`ref_lat`/`ref_lon`)과 PX4 투영식으로 바꿉니다.
- `ca`: 목표(이륙점 기준 북·동, 또는 위경도)를 정책 노드에 줍니다. 목표 3 m 안에서는 PX4 위치 제어로 목표점에 멈춘 뒤 착륙합니다.

**flight-test 드라이버에서 고친 위험 4가지:**

1. **킬 스위치로 disarm되면 다시 arm하고 이륙했습니다.** 지금은 arm을 한 번만 합니다. 그 뒤 disarm되면 끝납니다.
2. **PF가 죽어도 마지막 자세 명령을 100 ms마다 새 시각으로 계속 보냈습니다.** 지금은 PF 명령이 올 때만 전달합니다. 0.5 s 끊기면 그 자리에 착륙합니다.
3. **조종자가 넘겨받은 뒤에도 PF 자세를 계속 PX4로 보냈습니다.** 지금은 PX4가 OFFBOARD일 때만 보냅니다. 모드가 바뀌면 모든 출력을 영구히 멈춥니다(명령, setpoint, offboard 신호).
4. **이륙 중 조종자가 모드를 바꿔도 다시 OFFBOARD로 가져갔습니다.** 지금은 AUTO_TAKEOFF가 시작된 뒤 다른 모드가 되거나, 목표보다 낮은 곳에서 Hold가 되면 양보합니다.

**그 자리 착륙 조건:**

- PF 끊김 0.5 s
- CA 경로 끊김 2.5 s
- 지오펜스: 미션 최대 거리 + 20 m, 높이 + 10 m
- 속도 초과: 정책 v_max + 2.5 m/s가 0.5 s 이상
- PX4 위치 리셋: 수평 3 m 초과, 수직 1 m 초과, 기준점 변경
- 충돌: IMU 30 m/s², 0.2 s 안 2표본. 실기체 정상 비행 최대는 19 m/s², 충돌은 150–250 m/s²였습니다.
- dead reckoning(GNSS 상실)
- 운용자 `land`

**arm 조건:**

- PX4 pre-flight 통과
- PX4 위치 추정이 30 s 동안 리셋 없음
- PF heartbeat
- CA 노드 준비(`/noclimb/ready`: 자세, 라이다 드라이버, `/cloud` 모두 살아 있음)
- 운용자 `go`(실기체만, SITL/PILS는 자동)

**PX4 파라미터는 하나도 쓰지 않습니다.** 기존 `flight_manager`는 `COM_RC_IN_MODE 4`(RC 끔) 등을 쓰려 했습니다. SITL에서는 경로가 없어 동작하지 않았습니다.

### 2.1 SITL 시험 (Gazebo, X500, PX4 셸에서 고장 주입)

| 시험 | 넣은 것 | 결과 |
|---|---|---|
| go 게이트 + 로컬 사각 20 m, 3 m/s | 준비 뒤 `go` | READY → go → 이륙 4.26 m → OFFBOARD에서 5.0 m까지 올린 뒤 비행 30 s → 착륙·disarm. 횡오차 0.03–0.4 m, PF 전달 250 Hz(PF 명령 나이 최대 4 ms) |
| GPS 사각 20 m, 5 m/s | 위경도 CSV(`gps_mission.py from-local`) | 비행 23 s, 착륙 |
| 비행 중 조종자 개입 | 경로 20 m 지점에서 `commander mode auto:loiter` | 즉시 양보. 그 뒤 4 s 동안 PX4로 간 자세 0, setpoint 0, offboard 신호 0. 셸에서 착륙시키자 "disarmed in YIELDED"로 끝남 |
| 이륙 중 개입 | 2 m에서 Hold | "hold at 2.15 m during the take-off to 5.00 m"로 양보, 그 뒤 명령 0 |
| PF 강제 종료 | 비행 중 `pkill node_pathfollowing` | 0.54 s 뒤 그 자리 착륙. 마지막 명령을 되풀이하지 않음 |
| 공중 disarm | 비행 중 `commander disarm -f` | "disarmed in FOLLOW"로 즉시 끝남, 다시 arm 안 함 |
| 운용자 착륙 | `/flight_driver/land` | 그 자리 착륙 |

호버 추력: PF 로그 방식 0.636, PX4 추정 0.637로 같았습니다(SITL X500 모델).

## 3. 아침 순서

모든 명령은 젯슨(`ssh jetson@100.64.0.135`, Tailscale)의 `~/PX4-SITL-Runner`에서 실행합니다. 기체를 바꾸면 3.1부터 다시 합니다.

### 3.0 연결

```bash
scripts/dev/onboard_real.sh agent status            # a4vai-agent Up 이어야 함 (없으면 agent start)
```

- Pixhawk를 USB 시리얼로 먼저 꽂고, 그다음 휴대폰 USB 테더링을 켭니다. 꽂고 뺄 때마다 테더링이 끊깁니다.
- `eno1`은 Mid-360 쪽(`livox-mid360`, 192.168.1.50)이어야 합니다.

### 3.1 라이다 보정 (프롭 없이, 약 10분)

```bash
scripts/dev/onboard_real.sh calib cal_x500
scripts/dev/onboard_real.sh tool livox_bench_check.py 10
scripts/dev/onboard_real.sh tool lidar_mount_calib.py shake
scripts/dev/onboard_real.sh tool lidar_mount_calib.py rest --yaw <shake의 yaw> --centre-height <실측>
scripts/dev/onboard_real.sh tool lidar_mount_calib.py target --yaw <yaw> --roll <roll> --pitch <pitch>
scripts/dev/onboard_real.sh stop cal_x500
```

1. `livox_bench_check`: 10 Hz, 프레임당 약 15k점. 방위별 1 m 안 점으로 자기 기체 반사 반경을 봅니다 → `NOCLIMB_SELF_RADIUS`.
2. `shake`: 25 s 동안 손으로 roll, pitch, **yaw**를 모두 ±20°, 1 Hz 정도로 흔듭니다. 출력:
   - 지연 → `NOCLIMB_LIDAR_LATENCY`
   - 회전 → `NOCLIMB_LIDAR_ROLL_DEG`, `_PITCH_DEG`, `_YAW_DEG`
   - 축별 흔들림 값이 0.1보다 작으면 그 축을 더 흔들고 다시 합니다.
3. `rest`: 트인 땅에 가만히 둡니다. 출력:
   - Mid-360 내장 IMU 중력과 PX4 자세로 구한 roll/pitch(땅이 기울어도 됨)
   - 지면 평면 높이 → `NOCLIMB_LIDAR_Z`(라이다 높이 − 기체 중심 높이)
   - `--centre-height`: 땅에서 기체 중심까지 줄자로 잰 값
4. `target`: 기수 축 3–6 m 앞에 기둥이나 사람 하나를 둡니다. 방위가 0°에서 벗어나면 yaw 오차입니다.
5. 나온 줄은 프로필보다 우선합니다. `fly`·`mission` 명령줄 앞에 붙이세요.

자기 점: X500은 0.55 m였습니다. X8은 팔과 프롭이 0.77 m까지 나오므로 더 큽니다.
- **노드의 접촉 감지는 실기체에서 사실상 꺼집니다.** 자기 점 반경(X500 0.55, X8 약 0.9 m) 안의 반사를 지우는데, 접촉 거리(X500 0.39, X8 0.77 m)가 그보다 작기 때문입니다.
- 남는 대비책은 드라이버의 IMU 충돌 감지(30 m/s²)와 안전 조종사입니다.

### 3.2 첫 비행: GPS 웨이포인트, 3 m/s (호버 추력 재기)

```bash
python3 scripts/dev/gps_mission.py check line.csv        # 구간·거리·높이 확인
MISSION_SPEED=3 MISSION_ALT=5 scripts/dev/onboard_real.sh mission x500_3 wp_x500_a line.csv
scripts/dev/onboard_real.sh state                         # phase=WAIT_GO (READY) 확인
scripts/dev/onboard_real.sh go                            # 안전 조종사 준비 후
scripts/dev/onboard_real.sh stop wp_x500_a
```

- CSV: `lat,lon,alt` 행, alt는 이륙점 기준 높이(2 m 이상)입니다. 첫 점이 이륙점에서 1 m 넘게 떨어져 있으면 드라이버가 지금 위치를 앞에 붙입니다.
- 비행 뒤 호버 추력을 잽니다. `hover_thrust.py test_runs/wp_x500_a/pf_log.csv --assumed 0.645` 또는 Pixhawk 로그(QGC로 내려받기)에 `hover_thrust.py log.ulg`. 나온 값을 다음 비행부터 `NOCLIMB_HOVER_THRUST=`로 줍니다.

### 3.3 CA 비행 (X500 3 m/s → 5 m/s)

```bash
NOCLIMB_GOAL=0,30 NOCLIMB_ALT=3 NOCLIMB_HOVER_THRUST=<3.2 값> <3.1 줄들> \
  scripts/dev/onboard_real.sh fly x500_3 ca_x500_3_a
scripts/dev/onboard_real.sh state; scripts/dev/onboard_real.sh go
scripts/dev/onboard_real.sh stop ca_x500_3_a
```

- `NOCLIMB_GOAL "x,y"` = 동, 북 [m]이고, 이륙점 기준입니다. 위경도로 줄 때는 `NOCLIMB_GOAL_GPS="lat,lon"`.
- 기수를 목표 쪽으로 두고 이륙합니다(정책은 기수와 상관없지만 라이다 시야가 기수 기준).
- 장애물은 경로 중간에, 목표 3 m 안에는 두지 않습니다(목표 3 m 안은 PX4 위치 제어로 직진).
- 5 m/s: 프로필 `x500_5`. 같은 장애물에 3 m/s가 연속 2회 착륙했고 프롭 끝 여유가 0.5 m 이상일 때 올립니다.

### 3.4 X8

3.1(X8 자체 장착값), 3.2(호버 추력은 그대로 둬도 됨 — `octo.yaml`이 실기체와 맞음), 3.3을 프로필 `x8_3` → `x8_5`로 합니다.

## 4. 비행 전 PX4 확인 (QGC)

| 파라미터 | 확인할 값 | 이유 |
|---|---|---|
| `COM_RC_OVERRIDE` / `COM_RC_STICK_OV` | 3 / 30 % | 스틱으로 OFFBOARD·자동 모드를 넘겨받음 → 드라이버가 영구히 양보 |
| `COM_OF_LOSS_T` / `COM_OBL_RC_ACT` | 1.0 s / 0(Position) | 앱이 통째로 죽었을 때 |
| `NAV_RCL_ACT` / `RTL_RETURN_ALT` | 시험장에 맞게(Return 30 m 또는 Land) | RC 끊김 |
| `RC_MAP_KILL_SW` | 동작 확인 | 킬 스위치 → 드라이버는 다시 arm하지 않음 |
| `COM_DISARM_LAND` | 켜짐(> 0) | 착륙 뒤 PX4가 disarm해야 드라이버가 끝남(60 s 뒤 disarm 한 번 요청) |
| `MPC_TKO_SPEED` / `MIS_TAKEOFF_ALT` | 기본 | AUTO_TAKEOFF는 드라이버가 준 높이로 감 |
| `UXRCE_DDS_DOM_ID` | 0(젯슨 `ONBOARD_DOMAIN` 기본 0) | 연구실 LAN이 아니면 0 그대로 |
| `/fmu/out/sensor_combined` | 나와야 함(`ros2 topic hz`) | 충돌 감지. 없으면 그 기능만 빠짐 |

## 5. 게이트 비행 (C-Track WP3 → GOAL, 1.7 m, Mid-360 시뮬, `flight_driver` 포함)

프로필 파일 그대로(`pils_gate.sh`) 비행했습니다. 실기체에서 `onboard_real.sh fly`가 읽는 파일과 같습니다. 여유는 프롭 끝 기준 최소값입니다.

| 프로필 | 모드 | 결과 | 최소 여유 | 평균/최대 속도 | 고도(1.5 ± 0.2 m 안) | 착륙 위치 |
|---|---|---|---|---|---|---|
| `x500_3` | SILS | 착륙 | +1.04 m (C1) | 2.83 / 3.10 | 100 % | 목표 0.46 m |
| `x500_5` | SILS | 착륙 | +0.95 m (C1) | 4.07 / 5.07 | 100 % | 목표 0.19 m |
| `x8_3` | SILS | 착륙 | +0.86 m (C1) | 2.93 / 3.17 | 100 % | 목표 0.59 m |
| `x8_5` | SILS | 착륙 | +1.44 m (C2) | 3.91 / 5.01 | 87 % (최고 1.77 m) | 목표 0.60 m |

- 드라이버 순서는 모두 정상이었습니다. 이륙(PX4가 0.7 m에서 끝냄) → OFFBOARD에서 1.47 m까지 올림 → 2 s → 정책 비행 → 목표 2.8 m에서 PX4 위치 제어 → 목표점 착륙.
- 여유는 이전(flight_manager, 기울기 35°)과 비슷합니다.
  - X500 3 m/s는 +0.92 → +1.04, 5 m/s는 +1.34 → +0.95입니다.
  - X8 3 m/s는 +0.74 → +0.86입니다.
  - 모두 한 번씩만 비행했고, 같은 설정에서도 비행마다 차이가 큽니다.
- **정정:** 처음에 X500 여유를 X8 치수(프롭 끝 0.77 m)로 계산해 0.38 m 작게 적었습니다. 그래서 "30° 기울기로 여유가 줄었다"고 했는데, 그 해석은 틀렸습니다. `ctrack_clearance.py`는 이제 실행 기록에서 기체를 읽어 X500이면 X500 치수(0.386 m)를 씁니다.

**젯슨 PILS — 5G 경유 모드를 만들어서 했습니다.**
- 젯슨은 오늘 밤 5G 휴대폰 테더링 위 Tailscale(왕복 30–45 ms, 약 12 Mbit/s)로만 연결돼 있습니다. `eno1`은 라이다용입니다.
- 처음에는 이 링크에서 두 가지가 막혔습니다.
  - **PX4 uXRCE가 토픽을 하나도 만들지 않았습니다.** 로그는 `RTT too high for timesync: 28 ms`였습니다. PX4는 왕복 10 ms가 넘는 시간 동기 표본을 버리는데, 동기가 수렴해야 토픽을 만듭니다.
  - **AirSim 브리지가 시작 RPC(`simGetInstanceSegmentationColorMap`)에서 60 s 시간 초과로 죽었습니다.**
- 그래서 `PILS_LINK=wan` 모드를 만들었습니다(`pils_gate.sh`는 100.x 주소면 자동으로 켭니다).
  - PILS용 PX4 시작 스크립트 사본에 `param set UXRCE_DDS_SYNCT 0`을 넣습니다. 그러면 PX4가 들어오는 명령을 받은 시각으로 찍습니다(생성 코드 `min(timestamp, now)`).
  - AirSim 브리지는 PC에서 Unreal 옆에 둡니다(`pils-airsim` 컨테이너). 라이다와 자세 토픽만 TCP 하나로 젯슨에 넘깁니다(`scripts/ros2/tools/topic_relay.py`).
  - 라이다는 점마다 순서를 지킨 채 1 cm int16 + zlib로 보냅니다. 빈 샷 NaN도 그대로이고, 어댑터가 이 순서로 점 시각을 셉니다. 19.8 → 1.2–2.2 Mbit/s로 줄었고, 오차는 최대 0.005 m입니다.
  - 큰 메시지는 최신 것만 기다립니다. 처음에 원본을 그대로 보냈을 때는 링크가 꽉 차서 PX4 쪽 응답도 늦어졌습니다. 그 결과 PF가 1.7 s 동안 반영되지 않은 명령을 계속 내며 외란 추정이 상한까지 쌓였고, 이어 기체가 8.1 m/s까지 폭주했습니다. **드라이버가 속도 제한으로 그 자리 착륙시켰습니다.**
- 실비행에는 이 네트워크가 끼지 않습니다. PX4는 시리얼, 라이다는 이더넷 직결입니다. 링크 경유 PILS는 실기체보다 한 바퀴 지연이 40 ms 정도 더 깁니다.
- 이 모드의 변경(`UXRCE_DDS_SYNCT 0`, PC 쪽 브리지, 중계기)은 `pils_gate.sh`·`pils_onboard.sh`·`sitl-px4-airsim.sh`·`ctrack_super.sh`(PILS 분기)에만 있습니다. 실기체 실행 경로(`onboard_real.sh`, `onboard-real.sh`, 앱)에는 없음을 grep으로 확인했습니다. 실기체 PX4의 시간 동기는 켜진 그대로입니다.

| 프로필 | 모드 | 결과 | 최소 여유 | 평균/최대 속도 | 고도(1.5 ± 0.2 m 안) | 착륙 위치 |
|---|---|---|---|---|---|---|
| `x500_3` | **PILS(젯슨, 5G)** | 착륙 | +0.93 m (C1) | 2.79 / 3.20 | 100 % | 목표 0.49 m |
| `x500_5` | **PILS(젯슨, 5G) 1회** | **목재 벽 접촉** → 노드 접촉 감지로 그 자리 착륙 | **−0.18 m** (목재 벽, 중심이 벽 0.21 m 앞까지) | 3.87 / 5.02 | 100 % | 목표 30.7 m 앞 |
| `x500_5` | **PILS(젯슨, 5G) 2회** | 착륙 | +1.02 m (목재 벽) | 3.93 / 5.08 | 99 % | 목표 0.18 m |
| `x8_3` | **PILS(젯슨, 5G)** | 착륙 | +0.82 m (C1) | 2.91 / 3.24 | 100 % | 목표 0.58 m |
| `x8_5` | **PILS(젯슨, 5G)** | 착륙 | +1.60 m (목재 벽) | 3.99 / 5.10 | 100 % | 목표 0.64 m |

**5 m/s에서 접촉한 이유:**
- 정책은 벽 4.4 m 앞부터 크게 꺾고(a0 −1) 속도를 3.3 → 0.6 m/s로 낮추라고 명령했습니다.
- 기체는 그 명령을 약 0.7–0.8 s 늦게 따라갔습니다. 0.9 m 앞에서도 2.2 m/s였고, 그대로 벽 끝에 닿았습니다.
- 명령 대비 기체 속도의 지연은 SILS 0.70 s, PILS(5G) 0.80 s입니다. 링크가 더한 것은 약 0.1 s입니다. 감속하는 동안 기체가 명령보다 빠른 정도도 중앙 0.54 / 0.60 m/s로 비슷했습니다.
- 필요한 감속은 1.5–2 m/s²로 30° 기울기 한계(5.7 m/s²)보다 훨씬 작습니다. 기울기 제한 탓이 아니라 **PF 속도 추종이 느린 탓**입니다.
- 학습 쪽은 추종 τ 0.2–0.5 s와 지연 50–150 ms를 가정했습니다. 우리 PF+기체는 그보다 느립니다. 그래서 X500 5 m/s는 벽처럼 넓은 장애물 앞에서 여유가 작습니다.
- 그 밖의 비행(SILS 1, PILS 1)은 +0.95 / +1.02 m였습니다.
- **권장:**
  - 3 m/s에서 연속 2회 성공한 뒤 5 m/s로 갑니다.
  - 5 m/s에서는 장애물 사이를 넓게, 벽처럼 넓은 장애물은 피합니다.
  - 안전 조종사는 벽 앞에서 개입할 준비를 합니다.
  - 노드의 접촉 감지는 실기체에서 꺼집니다(3.1절). 시뮬에서는 접촉 뒤 1.0 s 만에 그 자리 착륙시켰지만, 실기체에서는 이 대비책이 없습니다.
  - 안전 필터 반경을 키우면(`NOCLIMB_SAFETY_R` 0.5 → 0.8 m) 더 일찍 감속합니다. 다만 오늘 밤 시험하지 않았습니다.

- 젯슨 부하(x500_3): CPU 중앙 80 %, 최대 93 %(25 W 모드). 정책 노드 한 스텝 중앙 14.8 ms, p95 26.2 ms. 20 Hz 간격 p95 54 ms. PF 명령 나이 중앙 2 ms, 최대 13 ms. 라이다 10 Hz 전량, 자세 100 Hz 전량 도착.
- LAN이 있으면 예전 방식이 더 실기체에 가깝습니다. `eno1`을 연구실 LAN에 꽂고 `ONBOARD_IP=192.168.219.152 ONBOARD_SSH=jetson@192.168.219.152 scripts/dev/pils_gate.sh x500_3 이름`으로 실행합니다.

## 6. 미룬 것 (알고 날 것)

- **CA는 목표 하나**입니다. 여러 점 CA(웨이포인트마다 회피)는 아직 없습니다.
- **EKF 리셋은 착륙으로만 대응합니다.** 보정은 하지 않습니다. 실기체 로그 24개에서 비행 중 z 리셋은 1번(충돌 때)뿐이었습니다.
- flight-test PF의 `safety_guard.py`는 옮기지 않았습니다. 드라이버의 감시(PF·경로 끊김, 펜스, 속도, 충돌)로 대신합니다.
- 움직이는 장애물은 학습하지 않았습니다. 사람 근처에서는 하지 않습니다.
- 젯슨 전원 모드(MAXN_SUPER), GNOME tracker 끄기는 사용자 결정입니다.
