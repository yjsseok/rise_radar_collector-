# SENSR Radar Collector v3 (Simplified Structure)

## 📁 디렉토리 구조

```
radar_v3/
├── main_multiprocessing.py       # 메인 실행 파일
├── run_multiprocessing_wsl.sh    # WSL/Linux 실행 스크립트
├── README.md                      # 이 파일
├── config/
│   └── config.yaml.example       # 설정 파일 예제
├── src/
│   ├── __init__.py
│   ├── sensr_client.py           # SENSR WebSocket 클라이언트
│   ├── data_processor.py         # 데이터 프로세서
│   ├── data_processor_multiprocessing.py  # 멀티프로세싱 프로세서
│   ├── utils.py                  # 유틸리티 함수
│   ├── track_logger.py           # 추적 로거
│   └── bag_recorder_optimized.py # 최적화된 Bag 레코더
├── sensr_proto/                  # Protobuf 정의
├── logs/                         # 로그 파일 저장 디렉토리
└── output/                       # ROS2 bag 파일 출력 디렉토리
```

## 🚀 v2.1.0 주요 기능

### 1. Deterministic Backpressure (백프레셔 관리)
- 데이터 타입별 독립 큐 (`output_data`, `point_cloud`)
- High watermark 기반 경고 (한 번만 출력)
- 설정 가능한 큐 크기 및 드롭 정책

### 2. Adaptive Worker Pool (적응형 워커 풀)
- 부하에 따라 워커 수 자동 조절 (4개 → 최대 8개)
- 입력 큐 크기 기반 scale up/down
- 워커별 처리 시간 통계 수집

### 3. Graceful Shutdown (정상 종료)
- `multiprocessing.Event` 기반 종료
- CTRL+C 시 Exception 없이 정상 종료
- 최종 리포트 출력 보장

## ⚙️ 설정

### 1. config.yaml 생성

```bash
cd /home/user/rise_radar_collector-/radar_v3
cp config/config.yaml.example config/config.yaml
```

### 2. config.yaml 편집

```yaml
sensr:
  host: "YOUR_HOST_IP"  # SENSR 서버 IP 주소
  ports:
    output_data: 5050
    point_cloud: 5051

multiprocessing:
  num_workers: 4          # 초기 워커 수
  max_workers: 8          # 최대 워커 수

queue:
  max_items: 200          # 큐 최대 크기
  high_watermark_pct: 80  # 경고 임계값 (%)
  drop_policy: "oldest"   # 드롭 정책

shutdown:
  timeout_s: 10           # 종료 타임아웃 (초)
```

## 🏃 실행 방법

### 방법 1: 실행 스크립트 사용 (권장)

```bash
cd /home/user/rise_radar_collector-/radar_v3
chmod +x run_multiprocessing_wsl.sh
./run_multiprocessing_wsl.sh
```

스크립트 내에서 다음 항목을 수정 가능:
- `HOST`: SENSR 호스트 ID (config.yaml에 정의된 것)
- `DURATION`: 테스트 시간 (초)
- `WORKERS`: 워커 프로세스 수

### 방법 2: Python 직접 실행

```bash
cd /home/user/rise_radar_collector-/radar_v3

# ROS2 환경 로드 (필요 시)
source /opt/ros/humble/setup.bash

# Python 실행
python3 main_multiprocessing.py --host samyang --duration 60 --workers 4
```

### 옵션

- `--config, -c`: 설정 파일 경로 (기본: `./config/config.yaml`)
- `--host, -H`: SENSR 호스트 ID
- `--duration, -d`: 테스트 시간 (초, 기본: 300)
- `--workers, -w`: 워커 프로세스 수 (기본: 4)

## 📊 출력 결과

### 실행 중 로그
- 메시지 수신/처리 통계
- 워커 풀 스케일링 이벤트
- 메모리 사용량
- 큐 상태

### 최종 리포트
```
📊 최종 테스트 리포트 (멀티프로세싱)
======================================================================

⏱️ 총 실행 시간: 60.00초

📨 메시지 수신:
  - 총 수신: 684
  - 포인트클라우드: 30
  - Output Data: 30

⚙️ 멀티프로세싱 통계:
  - 워커 수: 4개
  - 총 처리: 480
  - 드롭: 0
  - 평균 처리 시간: 25.5ms

💾 디스크 쓰기:
  - 총 쓰기: 480
  - 드롭된 메시지: 0

🚀 처리량:
  - 수신: 11.4 msg/s
  - 처리: 8.0 msg/s
  - 쓰기: 8.0 msg/s

💻 시스템 리소스:
  - 메모리 (평균): 350.0 MB
  - 메모리 (최소): 230.0 MB
  - 메모리 (최대): 470.0 MB
```

### 출력 파일
- ROS2 bag 파일: `./output/sensr_data_YYYYMMDD_HHMMSS/`
- 로그 파일: `./logs/sensr_recorder.log`

## 🔍 문제 해결

### "설정 파일 없음" 오류
```bash
cp config/config.yaml.example config/config.yaml
# config.yaml 편집 후 다시 실행
```

### "호스트 연결 실패" 오류
1. config.yaml의 `sensr.host` 확인
2. SENSR 서버가 실행 중인지 확인
3. 방화벽/네트워크 설정 확인

### "ROS2 없음" 경고 (Windows에서)
- Windows에서는 정상적인 동작입니다
- ROS2 bag 파일은 Linux/WSL에서만 생성됩니다

### 큐 포화 경고
```
⚠️ 큐 high watermark 도달: 160/200 (80%)
```
- `config.yaml`에서 `queue.max_items` 증가
- 또는 `multiprocessing.max_workers` 증가

## 📈 성능 목표

| 지표 | 목표 |
|------|------|
| 처리 속도 | ≥8 msg/s |
| 큐 경고 | 1회 이하 |
| 종료 | Exception 없음 |

## 📝 버전 정보

- **Version**: v2.1.0
- **Release Date**: 2025-11-12
- **Changes**: CHANGELOG_v2.1.0.md 참조

## 🔗 관련 문서

- [SENSR_MULTIPROCESSING_FIX_PLAN.md](../SENSR_MULTIPROCESSING_FIX_PLAN.md)
- [radar_v2/CHANGELOG_v2.1.0.md](../radar_v2/CHANGELOG_v2.1.0.md)
