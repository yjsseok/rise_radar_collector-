# How to Use

본 문서는 프로젝트 루트(`sensr_lidar_recorder/`) 기준으로 실행 가능한 주요 명령을 정리한 것입니다. 모든 예시는 UTF-8 인코딩 환경을 전제로 하며, Windows의 경우 `chcp 65001`이 자동으로 설정되는 배치 스크립트를 제공합니다.

## 1. Python CLI 실행

### 1.1 `main.py` (전체 파이프라인)
- 전체 파이프라인 기본 실행
  ```bash
  python main.py
  ```
- 설정 파일 지정
  ```bash
  python main.py --config config/config.yaml
  ```
- 출력 디렉터리 및 로그 레벨 변경
  ```bash
  python main.py --output-dir ./output --verbose
  ```
- 포인트클라우드/출력 데이터 샘플링 간격 조정
  ```bash
  python main.py --pointcloud-interval 0.5 --output-data-interval 1.0
  ```
- 포인트클라우드만 기록 (다른 데이터 무시)
  ```bash
  python main.py --pointcloud-only
  ```
- 빠른 시작 (안정화 대기 시간 생략)
  ```bash
  python main.py --quick-start
  ```
- 특정 호스트 선택 및 호스트 목록 확인
  ```bash
  python main.py --host site_b
  python main.py --host all
  python main.py --list-hosts
  ```
- `--host all`을 사용하면 등록된 모든 호스트별 실행 예시를 출력합니다.

> **주요 인자 정리**: `--config`, `--output-dir`, `--verbose`, `--pointcloud-interval`, `--output-data-interval`, `--pointcloud-only`, `--quick-start`, `--host`, `--host all`, `--list-hosts`.

### 1.2 `simple_pointcloud_recorder.py` (경량 포인트클라우드 수집)
- 기본 실행 (0.5초 간격, 무기한)
  ```bash
  python simple_pointcloud_recorder.py --interval 0.5
  ```
- 특정 시간만 수집
  ```bash
  python simple_pointcloud_recorder.py --interval 0.5 --duration 600
  ```
- 출력 경로 및 대상 SENSR 호스트 변경
  ```bash
  python simple_pointcloud_recorder.py --interval 1.0 --output-dir ./simple_output --host 192.168.0.10
  ```

### 1.3 관리/유틸리티 스크립트
- 포인트클라우드 퍼블리시 강제 활성화
  ```bash
  python enable_pointcloud.py                 # 기본 호스트(112.133.37.122)
  python enable_pointcloud.py 192.168.0.10 --rest 9081
  ```
- REST 기반 시스템 관리 CLI
  ```bash
  python sensr_manager.py status                     # 전체 상태 리포트
  python sensr_manager.py sensor list                # 센서 목록
  python sensr_manager.py sensor detail <sensor_id>
  python sensr_manager.py zone list                  # 존 목록
  python sensr_manager.py zone status <zone_id>
  python sensr_manager.py project list               # 프로젝트 목록
  python sensr_manager.py project load <name> [--path <dir>]
  python sensr_manager.py apply                      # 보류 중 변경사항 적용
  ```
- 연결/환경 점검
  ```bash
  python test_connection.py                 # config/config.yaml 사용
  python test_connection.py custom_config.yaml
  ```

## 2. Windows 배치 스크립트
- 의존성 설치 및 디렉터리 준비
  ```bat
  setup.bat
  ```
- 포인트클라우드 스트리밍 활성화 + 연결 테스트 + 상태 확인 일괄 수행
  ```bat
  setup_pointcloud.bat
  ```
- 전체 파이프라인 실행 (기본 샘플링 0.5/1.0초)
  ```bat
  run.bat
  ```
- 포인트클라우드 전용 실행
  ```bat
  run_simple.bat
  ```
- 장기 모니터링(1초 간격)
  ```bat
  run_long_term.bat
  ```
- 1분 테스트 수집
  ```bat
  run_test.bat
  ```

## 3. Ubuntu/Linux 쉘 스크립트
- ROS 2 Humble + 의존성 설치 자동화
  ```bash
  ./setup_ubuntu.sh
  ```
- 포인트클라우드 활성화 + 연결 테스트 + 상태 리포트 일괄 실행
  ```bash
  ./setup_pointcloud_ubuntu.sh
  ```
- 전체 파이프라인 실행 (ROS 2 환경 필요)
  ```bash
  ./run_ubuntu.sh
  # 또는 원하는 인자를 직접 전달
  ./run_ubuntu.sh --pointcloud-interval 0.3 --output-data-interval 0.5 --host site_b
  ```
- 포인트클라우드 전용 간편 실행
  ```bash
  ./run_simple_ubuntu.sh
  ```

## 4. 기타 참고 사항
- 모든 스크립트는 UTF-8 인코딩으로 저장되어 있으며, Windows 배치 파일은 `chcp 65001`을 통해 UTF-8 코드 페이지를 사용합니다.
- 실행 전 Python 가상환경을 활성화한 뒤 명령을 실행하면 의존성 충돌을 줄일 수 있습니다.
- 명령 실행 중 문제가 발생하면 `enable_pointcloud.py`, `test_connection.py`, `sensr_manager.py status`를 순서대로 실행하며 진단하세요.

