# SENSR LiDAR Data Recorder

## 개요
- Seoul Robotics SENSR 플랫폼에서 전송하는 객체·이벤트·포인트클라우드 데이터를 수집해 ROS 2 bag으로 저장하는 Python 기반 도구 모음입니다.
- WebSocket을 통해 실시간 메시지를 받고, REST API로 존/헬스/센서 메타데이터를 보강하여 데이터 품질을 확보합니다.
- ROS 2 `rosbag2_py`를 이용해 지정한 길이(기본 60초)로 bag 파일을 순환 생성하며, Windows 환경에서도 실행은 가능하지만 ROS 2 기능은 Linux/ROS 2 환경에서만 활성화됩니다.

## 주요 구성 요소
| 경로 | 설명 |
| --- | --- |
| `main.py` | 전체 파이프라인 실행 CLI. 설정 로드 → 모듈 초기화 → WebSocket 수신 → ROS bag 기록을 순차 처리합니다. |
| `simple_pointcloud_recorder.py` | 포인트클라우드만 빠르게 기록하는 경량 스크립트로 REST 호출을 최소화합니다. |
| `sensr_manager.py` | REST API 기반으로 센서/존/프로젝트를 조회·생성·수정·삭제할 수 있는 관리 CLI입니다. |
| `enable_pointcloud.py` | REST API를 이용해 `publish_level_point_cloud=2` 등 포인트클라우드 퍼블리시 옵션을 활성화합니다. |
| `src/sensr_client.py` | Output(5050)·Point Cloud(5051) WebSocket 채널을 관리하고 메시지를 큐와 콜백으로 전달합니다. |
| `src/data_processor.py` | SENSR protobuf 메시지를 ROS 메시지로 변환하고 REST 결과(존·헬스 등)를 캐싱해 병합합니다. |
| `src/bag_recorder.py` | `rosbag2_py.SequentialWriter`를 사용해 bag 파일을 생성, 토픽 메타데이터를 등록하고 큐 기반으로 메시지를 기록합니다. |
| `src/utils.py` | 설정 로드/검증, 로깅/시그널 처리, 디스크 사용량 점검, 호스트 프로필 관리, 상태 모니터링을 담당합니다. |
| `src/track_logger.py` | 추적 객체 수 변동을 감지해 로그로 기록합니다. |
| `config/config.yaml` | 기본 실행 구성을 담는 YAML 파일로 CLI 인자로 일부 값을 덮어쓸 수 있습니다. |
| `requirements.txt` / `requirements_ubuntu.txt` | Python 및 Ubuntu 의존성 목록입니다. |

## 설치 가이드
### 공통 준비
1. Python 3.8 이상이 설치되어 있는지 확인합니다.
2. 가상환경을 사용하는 것을 권장합니다(`python -m venv venv`).
3. 패키지를 설치합니다.
   ```bash
   pip install -r requirements.txt
   ```

### Ubuntu + ROS 2 환경
- ROS 2 (Foxy 이상) 및 `rosbag2_py`, `rclpy` 패키지가 필요합니다.
- Protobuf 3.11.4 이상과 Seoul Robotics SDK/Proto가 필요한 경우 기존 문서 절차를 참고해 설치합니다.
- `setup_ubuntu.sh`, `run_ubuntu.sh`를 통해 의존성 설치와 실행을 자동화할 수 있습니다.

### Windows 환경
- WSL2 또는 PowerShell에서 가상환경을 만든 뒤 동일하게 패키지를 설치합니다.
- ROS 2가 없으면 `BagRecorder` 초기화에서 `ImportError`가 발생하며 녹화 기능이 제한됩니다. 포인트클라우드 가공만 필요하면 `simple_pointcloud_recorder.py`를 사용하세요.
- 제공합니다 (`setup.bat`, `run_simple.bat`, `run.bat`) 스크립트는 PATH와 로그 폴더를 자동으로 정리합니다.

## 빠른 시작
### 전체 파이프라인 (객체 + 이벤트 + 포인트클라우드)
```bash
python main.py \
  --config config/config.yaml \
  --output-dir ./output \
  --verbose
```

### 포인트클라우드 전용 기록
```bash
python simple_pointcloud_recorder.py --interval 0.5
```

### 특정 호스트로 실행
```bash
python main.py --host site_b
python main.py --host all
python main.py --list-hosts
```
- `--host all`을 사용하면 등록된 모든 호스트별 실행 예시를 출력합니다.
- `--list-hosts`로 등록된 호스트 목록을 확인할 수 있습니다.
- `--pointcloud-only`와 `--pointcloud-interval`, `--output-data-interval` 옵션으로 샘플링 주기를 바꿀 수 있습니다.

## 설정 요약 (`config/config.yaml`)
```yaml
sensr:
  host: "112.133.37.122"
  active_host_label: "kimnyeong"
  hosts:
    - { id: "kimnyeong", address: "112.133.37.122", description: "김녕 센서" }
    - { id: "samyang", address: "122.202.187.5", description: "삼양 센서" }
  ports:
    output_data: 5050
    point_cloud: 5051
    logs: 5057
    rest: 9080
  reconnect_interval: 5
  api:
    cache_timeout: 5
    request_timeout: 10
    endpoints:
      health: "/results/health"
      zones: "/results/zone"
      sensors: "/settings/sensor-ext"
      nodes: "/settings/node"
      projects: "/commands/project"
recording:
  duration: 60
  output_directory: "./output"
  filename_format: "sensr_data_{timestamp}.bag"
  pointcloud_interval: 0.5
  output_data_interval: 0.5
  pointcloud_only: false
  skip_empty_data: false
  track_log_enabled: true   # 추적 객체 수 변동 시 로그
ros:
  topics:
    pointcloud: "/sensr/pointcloud"
    objects: "/sensr/objects"
    events: "/sensr/events"
    health: "/sensr/health"
    zones: "/sensr/zones"
logging:
  level: "INFO"
  file: "./logs/sensr_recorder.log"
```
- CLI 인자로 전달되는 값은 실행 시 `runtime_config`가 덮어쓰며 원본 파일은 수정하지 않습니다.
- `--output-dir`, `--pointcloud-interval`, `--output-data-interval`, `--pointcloud-only`, `--quick-start`, `--verbose`, `--host`, `--list-hosts` 옵션을 지원합니다.

## 실행 동작 개요
1. 설정을 읽고(`load_config`) 필수 키를 검증합니다(`validate_config`).
2. 로깅과 시그널 핸들러를 초기화하고 `StatusMonitor`를 준비합니다.
3. `SensrClient`가 WebSocket을 연결하고 메시지를 큐에 적재합니다.
4. `_on_message_received` 콜백이 메시지 타입을 확인하고 샘플링 간격을 적용합니다.
5. `DataProcessor`가 protobuf를 ROS 메시지로 변환하고 REST API 캐시를 활용해 존/헬스 데이터를 보강합니다.
6. `BagRecorder`가 메시지 큐를 읽어 `rosbag2_py` writer로 기록하며, duration이 지나면 새 bag을 생성합니다.
7. 종료 시 WebSocket과 bag writer를 안전하게 닫고, 디스크 용량이 부족하면 경고를 남깁니다.

## 문제 해결 가이드
| 증상 | 확인 사항 | 조치 |
| --- | --- | --- |
| SENSR 연결 실패 | `ping <host>`, `test_connection.py` | 네트워크 라우팅과 방화벽을 확인하고, `--host`로 올바른 주소를 지정합니다. |
| Bag 파일 용량이 비정상적으로 작음 | 포인트클라우드 퍼블리시 여부 | `python enable_pointcloud.py`로 `publish_level_point_cloud=2`를 설정하고, 첫 메시지 수신 시까지 대기합니다. |
| 큐가 빠르게 가득 참 | 샘플링 간격, 디스크 I/O | `pointcloud_interval`을 늘리고, SSD 디스크 사용을 권장합니다. |
| Windows에서 ROS 모듈 ImportError | ROS 2 미설치 | 포인트클라우드만 필요한 경우 `simple_pointcloud_recorder.py`, 전체 기능은 Linux/ROS 2에서 실행하세요. |

로그는 `logs/sensr_recorder.log`에 저장되며 `--verbose`로 DEBUG 레벨을 활성화할 수 있습니다.
- `track_log_enabled`를 활성화하면 사람이 추적 구역에 출입할 때 객체 수 변동이 동일 로그에 기록됩니다.

## 개발 & 테스트 메모
- 현재 자동화 테스트는 제공되지 않습니다(`tests/` 폴더 비어 있음). 기능 변경 시 수동으로 bag 파일을 생성해 검증하세요.
- 새로운 메시지 타입을 추가하려면 `DataProcessor`에 ROS 메시지 생성 로직을 구현하고 `BagRecorder`에 토픽 메타데이터를 등록해야 합니다.
- 장기 운영 시 `BagRecorder.cleanup_old_files()`를 주기적으로 호출하거나 cron/Task Scheduler로 디스크 용량을 관리하세요.

## 문서 및 인코딩
- 본 README를 포함한 프로젝트 문서는 UTF-8 인코딩을 사용합니다.
- 추가 문서는 `docs/` 디렉터리가 아닌 README 하나로 통합했으며, 필요 시 본 문서만 갱신하면 됩니다.

## 라이선스
- 라이선스 정보가 별도로 제공되지 않았습니다. 사내 배포 정책을 따르거나 라이선스 파일을 추가하세요.

