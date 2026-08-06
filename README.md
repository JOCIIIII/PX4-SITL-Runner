# A4VAI SITL Simulator (InhouseSim v0.9)

PX4 + Gazebo Classic + AirSim(Cosys) + ROS2(Humble) 기반 자율비행 드론 SITL 시뮬레이터.
라이다 맵핑(rog_map) → SUPER 플래너 충돌회피 → PathFollowing 추종의 전체 파이프라인을 도커로 실행한다.

## 요구사항

Ubuntu 22.04 (x86_64) · NVIDIA GPU + 드라이버 · 그래픽 세션 · 디스크 ~60GB

## 설치

```bash
git clone -b runtime https://github.com/JOCIIIII/PX4-SITL-Runner.git && cd PX4-SITL-Runner
./scripts/setup-host.sh     # Docker + NVIDIA Container Toolkit (sudo, 1회. docker 그룹 추가 시 재로그인)
./scripts/install.sh        # 이미지·소스·빌드·AirSim 바이너리 전자동 (~30분)
```

## 실행

```bash
./scripts/run-integrated-algorithm.sh      # SUPER + PathFollowing (충돌회피 비행)
./scripts/run-pf-test.sh    # PathFollowing 단독 (고정 웨이포인트)
```

이륙→비행→착륙까지 자동 수행. 정지는 `./scripts/stop.sh gazebo-classic-airsim-sitl`,
모니터링은 Foxglove Studio → `ws://localhost:8765`.

## 제거

```bash
./scripts/uninstall.sh
```

## 상세 문서

목표점/웨이포인트 변경, 재빌드, 트러블슈팅 → [MANUAL.md](./MANUAL.md)

## License

MIT — [LICENSE](./LICENSE)
