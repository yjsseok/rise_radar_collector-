#!/bin/bash

# SENSR LiDAR Data Recorder - Ubuntu 실행 스크립트
# ROS2 환경에서 SENSR 데이터를 수집하는 스크립트

set -e

echo "================================================"
echo "SENSR LiDAR Data Recorder - Ubuntu ROS2"
echo "================================================"

# ROS2 환경 확인
if [ -z "$ROS_DISTRO" ]; then
    echo "[ERROR] ROS2 환경이 설정되지 않았습니다"
    echo "다음 명령을 실행하세요: source /opt/ros/humble/setup.bash"
    exit 1
fi

# ROS2 Humble 확인
if [ "$ROS_DISTRO" != "humble" ]; then
    echo "[WARNING] ROS2 Humble이 아닌 $ROS_DISTRO가 설정되어 있습니다"
    echo "[INFO] ROS2 Humble 사용을 권장합니다"
fi

echo "[INFO] ROS2 $ROS_DISTRO 환경 확인됨"

# Python 확인
if ! command -v python3 &> /dev/null; then
    echo "[ERROR] Python3가 설치되지 않았습니다"
    exit 1
fi

echo "[INFO] Python $(python3 --version) 확인됨"

# 설정 파일 확인
if [ ! -f "config/config.yaml" ]; then
    echo "[ERROR] 설정 파일을 찾을 수 없습니다: config/config.yaml"
    exit 1
fi

echo "[INFO] 설정 파일 확인됨"

# 디렉토리 생성
mkdir -p logs output simple_output

# ROS2 daemon 확인 (ROS2는 roscore가 필요 없음)
echo "[INFO] ROS2는 별도의 core 프로세스가 필요하지 않습니다"
if ! ros2 daemon status > /dev/null 2>&1; then
    echo "[INFO] ROS2 daemon 시작 중..."
    ros2 daemon start
fi
echo "[INFO] ROS2 daemon 실행 중 확인됨"

# cleanup 함수
cleanup() {
    echo ""
    echo "[INFO] 프로그램 종료 중..."
    echo "[INFO] 종료 완료"
    exit 0
}

# 시그널 핸들러 설정
trap cleanup SIGINT SIGTERM

echo "[INFO] SENSR LiDAR Data Recorder 시작..."
echo "[INFO] Features: Full ROS2 integration + Zone/Health API"
echo "[INFO] Output: output/ directory"
echo "[INFO] Ctrl+C로 중단할 수 있습니다"
echo ""

# Default arguments if none provided  
if [ $# -eq 0 ]; then
    echo "[INFO] 기본 설정 사용: --pointcloud-interval 0.5 --output-data-interval 1.0"
    python3 main.py --pointcloud-interval 0.5 --output-data-interval 1.0
else
    python3 main.py "$@"
fi

# 정상 종료 시에도 cleanup 호출
cleanup
