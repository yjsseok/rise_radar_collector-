#!/bin/bash
# SENSR Simple PointCloud Recorder - WSL/Linux Version
# 무한 실행 모드 (Ctrl+C로 종료)

set -e

echo "================================================"
echo "🚀 SENSR Simple PointCloud Recorder (WSL)"
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

# 필요한 디렉토리 생성
echo "[INFO] 필요한 디렉토리 생성 중..."
mkdir -p simple_output
mkdir -p logs

# 메모리 최적화
echo "[INFO] 메모리 캐시 정리 중..."
sync
python3 -c "import gc; gc.collect()" 2>/dev/null || true

echo ""
echo "[INFO] SENSR Simple PointCloud Recorder 시작"
echo "[INFO] 모드: 무한 실행 (Ctrl+C로 종료)"
echo "[INFO] 출력: simple_output/ 디렉토리"
echo "[INFO] 데이터: 포인트클라우드만"
echo ""

# 기본 설정값
HOST="${SENSR_HOST:-122.202.187.5}"
INTERVAL="${SENSR_INTERVAL:-1.0}"

# 명령줄 인자 처리
while [[ $# -gt 0 ]]; do
    case $1 in
        --host|-h)
            HOST="$2"
            shift 2
            ;;
        --interval|-i)
            INTERVAL="$2"
            shift 2
            ;;
        *)
            echo "알 수 없는 옵션: $1"
            echo "사용법: $0 [--host IP] [--interval 초]"
            exit 1
            ;;
    esac
done

echo "[INFO] SENSR 호스트: $HOST"
echo "[INFO] 수집 간격: ${INTERVAL}초"
echo ""

# 무한 루프로 자동 재시작
while true; do
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Simple Recorder 시작..."
    
    # Python 스크립트 실행
    python3 simple_pointcloud_recorder.py \
        --host "$HOST" \
        --interval "$INTERVAL" || {
        echo ""
        echo "[경고] 프로세스가 종료되었습니다 (종료 코드: $?)"
        echo "[INFO] 10초 후 자동 재시작..."
        sleep 10
        continue
    }
    
    echo ""
    echo "[INFO] 정상 종료. 재시작하지 않습니다."
    break
done

echo "[INFO] Simple Recorder 종료"
