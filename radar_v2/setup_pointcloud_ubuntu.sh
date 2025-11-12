#!/bin/bash

# SENSR Point Cloud Setup & Management - Ubuntu 스크립트

set -e

echo "================================================"
echo "🔧 SENSR Point Cloud Setup & Management (Ubuntu)"
echo "================================================"

# Python 확인
if ! command -v python3 &> /dev/null; then
    echo "[ERROR] Python3가 설치되지 않았습니다"
    exit 1
fi

echo "[INFO] Python $(python3 --version) 확인됨"

echo ""
echo "[1/3] 포인트클라우드 스트리밍 활성화 중..."
python3 enable_pointcloud.py
if [ $? -ne 0 ]; then
    echo "[ERROR] 포인트클라우드 설정 실패"
    exit 1
fi

echo ""
echo "[2/3] 연결 테스트 중..."
python3 test_connection.py
if [ $? -ne 0 ]; then
    echo "[WARNING] 연결 테스트 실패 - 네트워크 설정을 확인해주세요"
fi

echo ""
echo "[3/3] 시스템 상태 확인 중..."
python3 sensr_manager.py status

echo ""
echo "[SUCCESS] 포인트클라우드 설정 완료!"
echo ""
echo "다음 단계:"
echo "  ./run_simple_ubuntu.sh         # 경량 수집 시작"
echo "  ./run_ubuntu.sh                # 완전 기능 수집 시작"
echo "  python3 simple_pointcloud_recorder.py --interval 1.0  # 장기 모니터링"
echo ""