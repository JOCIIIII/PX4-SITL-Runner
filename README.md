# A4VAI SITL Simulator (InhouseSim v0.9)

무인이동체 자율비행 SITL 시뮬레이터.
PX4(비행제어) + Gazebo Classic(물리) + AirSim(센서/환경) + ROS2 Humble(자율비행 알고리즘)을
도커 컨테이너로 통합 실행하며, 라이다 맵핑 → SUPER 플래너 충돌회피 → 경로추종(PathFollowing)의
전체 파이프라인이 이륙부터 착륙까지 자동으로 수행됩니다.

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

도커 이미지 다운로드(~24GB) → PX4 빌드 → 알고리즘 소스 클론/빌드 → AirSim 환경 다운로드까지
전 과정이 자동입니다 (약 30분~1시간, 네트워크 속도에 따라 다름).
중간에 실패해도 **다시 실행하면 이어서 진행**됩니다.

## 4. 실행

```bash
./scripts/run-integrated-algorithm.sh   # 통합 알고리즘: 라이다 맵핑 + SUPER 충돌회피 + 경로추종
./scripts/run-pf-test.sh                # 경로추종 단독 테스트 (고정 웨이포인트, 플래너 없음)
```

실행하면 Gazebo·AirSim·QGroundControl 창이 뜨고, 기체가 **자동으로 이륙 → 비행 → 목표점 착륙**까지 수행합니다.
(첫 실행은 컨테이너 초기화로 1~2분 정도 더 걸릴 수 있습니다)

## 5. 종료

- 실행한 터미널에서 **`Ctrl+C`** — 전체 컨테이너가 정상 종료됩니다.
- 창이 남아 있거나 다른 터미널에서 종료하려면:

```bash
./scripts/stop.sh gazebo-classic-airsim-sitl
```

## 6. 시나리오 변경

| 대상 | 파일 (수정 후 재실행만 하면 반영) |
|---|---|
| 통합 알고리즘 목표점 | `~/Documents/A4VAI-SITL/ROS2/ros2_ws/src/realgazebo_fastplanner_adapters/config/goals_super_far.csv` (ENU x,y,z[m]) |
| 경로추종 웨이포인트 | `~/Documents/A4VAI-SITL/ROS2/ros2_ws/src/path_following_test/wp.csv` (NED x,y,고도[m]) |

## 7. 모니터링

- **Foxglove Studio**([다운로드](https://foxglove.dev/download))를 `ws://localhost:8765`에 연결 —
  포인트클라우드(`/cloud`), 점유맵(`/rog_map/occ`), 경로(`/fsm/path`), 기체 위치(`/odom`) 시각화
- **QGroundControl** 창 — 기체 상태·모드·궤적
- 노드별 로그: `~/Documents/A4VAI-SITL/ROS2/logs/*.log`

## 8. 제거

```bash
./scripts/uninstall.sh    # 컨테이너·작업공간 삭제 (도커 이미지 삭제는 선택)
```

## 9. 문제 해결

| 증상 | 조치 |
|---|---|
| `permission denied` (docker) | docker 그룹 미적용 — 로그아웃/로그인 후 재시도 |
| `container name already in use` | `docker rm -f px4-env gz-sim airsim-binary ros2-env qgc-app` 후 재실행 |
| `Pool overlaps with other one` | 다른 프로젝트가 172.70.0.0/16 네트워크 점유 — 해당 네트워크 제거 |
| 설치 중단/실패 | `./scripts/install.sh` 재실행 (멱등 — 완료된 단계는 건너뜀) |

그 외 상세 내용은 [MANUAL.md](./MANUAL.md) 참조.

## 문의

사용 중 문의사항은 jociiiii@inha.edu 로 연락 주세요.

## License

MIT — [LICENSE](./LICENSE)
