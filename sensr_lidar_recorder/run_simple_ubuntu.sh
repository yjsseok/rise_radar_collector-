#!/bin/bash

# SENSR Simple Point Cloud Recorder - Ubuntu 스크립트
# 경량 포인트클라우드 수집기 (서버 부하 최소)

set -e

echo "================================================"
echo "🥇 SENSR Simple Point Cloud Recorder (Ubuntu)"
echo "================================================"

# Python 확인
if ! command -v python3 &> /dev/null; then
    echo "[ERROR] Python3가 설치되지 않았습니다"
    exit 1
fi

echo "[INFO] Python $(python3 --version) 확인됨"

# 디렉토리 생성
mkdir -p logs simple_output

# cleanup 함수
cleanup() {
    echo ""
    echo "[INFO] Simple Point Cloud Recorder 종료 중..."
    echo "[INFO] 종료 완료"
    exit 0
}

# 시그널 핸들러 설정
trap cleanup SIGINT SIGTERM

echo "[INFO] Simple Point Cloud Recorder 시작..."
echo "[INFO] Features: Point cloud only, minimal server load"
echo "[INFO] Output: simple_output/ directory"
echo "[INFO] Ctrl+C로 중단할 수 있습니다"
echo ""

# Default arguments if none provided
if [ $# -eq 0 ]; then
    echo "[INFO] 권장 설정 사용: --interval 0.5"
    python3 simple_pointcloud_recorder.py --interval 0.5
else
    python3 simple_pointcloud_recorder.py "$@"
fi

# 정상 종료 시에도 cleanup 호출
cleanup