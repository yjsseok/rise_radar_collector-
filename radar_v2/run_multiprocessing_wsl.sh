#!/bin/bash
# SENSR Multiprocessing Recorder - WSL/Linux Version
# 시간 지정 실행 모드

set -e

echo "================================================"
echo "🚀 SENSR Multiprocessing Recorder (WSL)"
echo "================================================"

# Python 확인
if ! command -v python3 &> /dev/null; then
    echo "❌ Python3이 설치되어 있지 않습니다"
    exit 1
fi

echo "[INFO] Python3 버전: $(python3 --version)"

# 현재 디렉토리 확인
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "[INFO] 작업 디렉토리: $SCRIPT_DIR"

# config.yaml 파일 확인
if [ ! -f "config/config.yaml" ]; then
    echo "❌ config.yaml 파일이 없습니다: config/config.yaml"
    echo "[INFO] config.yaml.example을 복사하여 config.yaml을 생성하세요"
    echo "cp config/config.yaml.example config/config.yaml"
    exit 1
fi

# 필요한 디렉토리 생성
echo "[INFO] 필요한 디렉토리 생성 중..."
mkdir -p logs
mkdir -p simple_output

# 메모리 최적화
echo "[INFO] 메모리 캐시 정리 중..."
sync
python3 -c "import gc; gc.collect()" 2>/dev/null || true

echo ""
echo "[INFO] SENSR Multiprocessing Recorder 시작"
echo "[INFO] 모드: 시간 지정 실행"
echo "[INFO] 출력: simple_output/ 디렉토리"
echo "[INFO] 데이터: 포인트클라우드 + 객체 데이터"
echo ""

# 기본 설정값
DURATION=${SENSR_DURATION:-300}
WORKERS=${SENSR_WORKERS:-4}

# 명령줄 인자 처리
while [[ $# -gt 0 ]]; do
    case $1 in
        --duration|-d)
            DURATION="$2"
            shift 2
            ;;
        --workers|-w)
            WORKERS="$2"
            shift 2
            ;;
        --config|-c)
            CONFIG_PATH="$2"
            shift 2
            ;;
        *)
            echo "알 수 없는 옵션: $1"
            echo "사용법: $0 [--duration 초] [--workers 수] [--config 경로]"
            exit 1
            ;;
    esac
done

CONFIG_PATH="${CONFIG_PATH:-../config/config.yaml}"

echo "[INFO] 실행 시간: ${DURATION}초"
echo "[INFO] 워커 수: ${WORKERS}개"
echo "[INFO] 설정 파일: $CONFIG_PATH"
echo ""

# test 디렉토리로 이동
cd test

echo "[$(date '+%Y-%m-%d %H:%M:%S')] Multiprocessing Recorder 시작..."

# Python 스크립트 실행
python3 main_multiprocessing.py \
    --config "$CONFIG_PATH" \
    --duration "$DURATION" \
    --workers "$WORKERS"

EXIT_CODE=$?

cd ..

echo ""
if [ $EXIT_CODE -eq 0 ]; then
    echo "[INFO] ✅ 정상 종료 (종료 코드: $EXIT_CODE)"
else
    echo "[경고] ⚠️ 오류 발생 (종료 코드: $EXIT_CODE)"
fi

echo "[INFO] Multiprocessing Recorder 종료"
