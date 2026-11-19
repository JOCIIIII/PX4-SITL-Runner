# 컨테이너 기반의 PX4 SITL 시뮬레이션 및 개발 스크립트 도구

## 설명

[![en](https://img.shields.io/badge/lang-en-red.svg)](../README.md)
[![kr](https://img.shields.io/badge/lang-kr-blue.svg)](./README-KR.md)

- 본 저장소는 컨테이너 기반의 PX4 SITL 시뮬레이션 및 개발을 위한 스크립트 및 도구를 제공합니다.
- 저장소에서 제공하는 스크립트 도구를 사용하면 PX4 SITL 시뮬레이션 수행 및 개발 과정을 보다 편리하게 수행할 수 있습니다.

## 문서

- 자세한 문서는 [저장소 위키](https://github.com/kestr31/PX4-SITL-Runner/wiki)를 참조해주세요.

## 요구 사항

### HW & OS 관련 요구 사항

- AMD64(x64) 기반 리눅스 시스템 (Ubuntu 22.04 LTS 권장)
- Nvidia GPU (≥ RTX 20 Ampere)
- 512 GB 이상의 저장 공간 (NVMe SSD 권장)
- 네트워크 접속 (환경 구성 파일들을 받기 위해 필요)
- 리눅스 데스크톱 환경 (=물리 디스플레이 또는 동등한 디스플레이 수단)

### SW 요구 사항

- `sudo` 권한
- Docker & Docker Compsoe
- Nvidia 독점 드라이버
- Nvidia 컨테이너 툴킷

## License

- 본 저장소는 MIT 라이선스 하에 배포됩니다. 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조해주세요.
