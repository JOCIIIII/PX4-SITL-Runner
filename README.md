# A4VAI SITL Simulator (InhouseSim v0.9)

무인이동체 자율비행 SITL 시뮬레이터.
PX4(비행제어) + Gazebo Classic(물리) + AirSim(센서/환경) + ROS2 Humble(자율비행 알고리즘)을
도커 컨테이너로 통합 실행하며, 이륙부터 착륙까지 자동으로 수행됩니다.

---

## 1. 시스템 요구사항

| 항목 | 요구 |
|---|---|
| OS | Ubuntu 22.04 LTS (x86_64) |
| GPU | NVIDIA GPU (RTX 20xx 이상 권장) |
| 디스크 | 여유 공간 60GB 이상 |
| 기타 | 물리 디스플레이(그래픽 세션), 인터넷 연결, sudo 권한 |

## 2. 사전 준비 (1회)

**① NVIDIA 드라이버** — 이미 설치되어 있다면(`nvidia-smi` 동작) 건너뜁니다.

```bash
sudo ubuntu-drivers autoinstall
sudo reboot
nvidia-smi        # 재부팅 후 GPU 정보가 출력되면 정상
```

**② Docker + NVIDIA Container Toolkit** — 아래 스크립트가 자동 설치합니다.

```bash
git clone -b runtime https://github.com/JOCIIIII/PX4-SITL-Runner.git
cd PX4-SITL-Runner
./scripts/setup-host.sh
```

> 스크립트가 "docker 그룹 추가" 안내를 출력하면 **로그아웃 후 다시 로그인**(또는 재부팅)하고 다음 단계로 진행하세요.
> 수동 설치를 원하면: [Docker 공식 문서](https://docs.docker.com/engine/install/ubuntu/), [NVIDIA Container Toolkit 문서](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## 3. 설치

```bash
./scripts/install.sh
```

도커 이미지 다운로드(24GB) → PX4 빌드 → 알고리즘 소스 클론/빌드 → AirSim 환경 다운로드까지
전 과정이 자동입니다 (네트워크 속도에 따라 30분에서 1시간).
중간에 실패해도 **다시 실행하면 이어서 진행**됩니다.

## 4. 실행

```bash
./scripts/run-integrated-algorithm.sh   # 경로추종 + 충돌회피
./scripts/run-pf-test.sh                # 경로추종 단독 테스트
```

실행하면 Gazebo·AirSim·QGroundControl 창이 뜨고, 기체가 **자동으로 이륙 → 비행 → 목표점 착륙**까지 수행합니다.
(첫 실행은 컨테이너 초기화로 1-2분 정도 더 걸릴 수 있습니다)

## 5. 종료

- 실행한 터미널에서 **`Ctrl+C`** — 전체 컨테이너가 정상 종료됩니다.
- 창이 남아 있거나 다른 터미널에서 종료하려면:

```bash
./scripts/stop.sh gazebo-classic-airsim-sitl
```

## 6. 제거

```bash
./scripts/uninstall.sh    # 컨테이너·작업공간 삭제 (도커 이미지 삭제는 선택)
```

## 7. 문제가 생기면 (로그 보내기)

설치와 실행의 터미널 출력은 자동으로 파일에 저장됩니다. 문제가 생기면 아래를 그대로 보내주세요.

| 파일 | 내용 |
|---|---|
| `~/Documents/A4VAI-SITL/logs/install_<시각>.log` | `install.sh` 전체 출력 |
| `~/Documents/A4VAI-SITL/logs/run_<프로필>_<시각>.log` | 실행 스크립트 전체 출력 (PX4·Gazebo·AirSim·ROS2·QGC 컨테이너 로그 포함) |
| `~/Documents/A4VAI-SITL/logs/run_<프로필>_<시각>_ros2/` | 그 실행의 ROS2 노드별 로그 (Ctrl+C 로 종료했을 때 복사됨) |

```bash
# 한 번에 묶기
tar -czf ~/a4vai-logs.tar.gz -C ~/Documents/A4VAI-SITL logs
```

> `uninstall.sh` 는 `~/Documents/A4VAI-SITL` 전체(로그 포함)를 지우므로, 로그는 제거 전에 먼저 보내주세요.

## License

MIT — [LICENSE](./LICENSE)
