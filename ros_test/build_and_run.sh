#!/bin/bash
# SENSR ROS2 Driver Build and Run Script

set -e

echo "================================================"
echo "SENSR ROS2 Driver - Build and Run"
echo "================================================"

# ROS2 환경 설정
echo "[INFO] ROS2 환경 설정..."
source /opt/ros/humble/setup.bash

# Workspace 디렉토리로 이동
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "[INFO] 현재 디렉토리: $(pwd)"

# sensr_proto 복사 (필요한 경우)
PROTO_SOURCE="../sensr_lidar_recorder/sensr_proto"
PROTO_DEST="./src/sensr_ros2_driver/sensr_proto"

if [ -d "$PROTO_SOURCE" ]; then
    echo "[INFO] sensr_proto 복사 중..."
    cp -r "$PROTO_SOURCE" "$PROTO_DEST"
    echo "[INFO] sensr_proto 복사 완료"
else
    echo "[WARNING] sensr_proto 폴더를 찾을 수 없습니다: $PROTO_SOURCE"
    echo "[WARNING] protobuf 기능이 작동하지 않을 수 있습니다"
fi

# Python 의존성 설치
echo "[INFO] Python 의존성 설치..."
pip3 install websocket-client pyyaml

# 빌드
echo "[INFO] ROS2 패키지 빌드 중..."
colcon build --packages-select sensr_ros2_driver

# 환경 설정
echo "[INFO] 빌드된 패키지 환경 설정..."
source install/setup.bash

# 실행 권한 설정
chmod +x src/sensr_ros2_driver/scripts/*.py

echo "[INFO] 빌드 완료!"
echo ""
echo "사용 방법:"
echo "1. 드라이버만 실행:"
echo "   ros2 launch sensr_ros2_driver sensr_driver.launch.py"
echo ""
echo "2. 드라이버 + Bag 레코더 실행:"
echo "   ros2 launch sensr_ros2_driver sensr_system.launch.py"
echo ""
echo "3. 특정 IP로 실행:"
echo "   ros2 launch sensr_ros2_driver sensr_system.launch.py host:=192.168.1.100"
echo ""
echo "4. Bag 파일 확인:"
echo "   ros2 bag info ./sensr_bags/sensr_data_YYYYMMDD_HHMMSS"
echo ""
echo "================================================"