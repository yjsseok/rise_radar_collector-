#!/bin/bash
# RViz2 실행 스크립트

echo "================================================"
echo "SENSR ROS2 Driver - RViz2 Visualization"
echo "================================================"

# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 워크스페이스 환경 설정 (빌드가 완료된 경우)
if [ -f "install/setup.bash" ]; then
    echo "[INFO] 빌드된 워크스페이스 환경 로드 중..."
    source install/setup.bash
fi

# RViz 설정 파일 경로
RVIZ_CONFIG="src/sensr_ros2_driver/config/sensr.rviz"

if [ -f "$RVIZ_CONFIG" ]; then
    echo "[INFO] RViz2 실행 중 (설정 파일 포함)..."
    rviz2 -d "$RVIZ_CONFIG"
else
    echo "[INFO] RViz2 실행 중 (기본 설정)..."
    rviz2
fi