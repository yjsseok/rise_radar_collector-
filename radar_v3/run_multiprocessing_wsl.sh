#!/bin/bash
# SENSR 멀티프로세싱 테스트 - WSL/Linux 환경

echo "======================================================================"
echo "SENSR 멀티프로세싱 고속 처리 테스트 (WSL/Linux)"
echo "======================================================================"
echo ""

# 호스트와 워커 수 설정
HOST="samyang"
DURATION=60
WORKERS=4

echo "호스트: $HOST"
echo "테스트 시간: ${DURATION}초"
echo "워커 수: ${WORKERS}개"
echo ""
echo "WSL/Linux 환경에서 실행 중"
echo ""

# ROS2 환경 설정 (있는 경우)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "ROS2 Humble 환경 로드 중..."
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    echo "ROS2 Foxy 환경 로드 중..."
    source /opt/ros/foxy/setup.bash
else
    echo "ROS2 없이 실행 (Windows Mock 모드)"
fi

# PYTHONPATH 설정 (radar_v3 구조)
export PYTHONPATH="$(pwd):$(pwd)/src:$PYTHONPATH"

# Python 실행
python3 main_multiprocessing.py --host $HOST --duration $DURATION --workers $WORKERS

echo ""
echo "테스트 완료!"
