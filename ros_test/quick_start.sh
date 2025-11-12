#!/bin/bash
# SENSR ROS2 전체 시스템 빠른 시작 스크립트

set -e

echo "================================================"
echo "SENSR ROS2 Driver - Quick Start"
echo "================================================"

# ROS2 환경 확인
if ! command -v ros2 &> /dev/null; then
    echo "[ERROR] ROS2가 설치되지 않았습니다!"
    echo "ROS2 Humble을 설치한 후 다시 실행해주세요."
    exit 1
fi

# ROS2 환경 설정
echo "[INFO] ROS2 환경 설정 중..."
source /opt/ros/humble/setup.bash

# 현재 디렉토리 확인
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 빌드 여부 확인
if [ ! -f "install/setup.bash" ]; then
    echo "[INFO] 패키지가 빌드되지 않았습니다. 빌드를 시작합니다..."
    ./build_and_run.sh
fi

# 워크스페이스 환경 설정
echo "[INFO] 워크스페이스 환경 로드 중..."
source install/setup.bash

# 사용자에게 옵션 선택하게 하기
echo ""
echo "실행할 모드를 선택하세요:"
echo "1) 드라이버만 실행 (토픽 발행만)"
echo "2) 드라이버 + Bag 레코더 실행 (권장)"
echo "3) RViz2 시각화만 실행"
echo "4) 전체 시스템 + RViz2 실행"
echo ""
read -p "선택 (1-4): " choice

case $choice in
    1)
        echo "[INFO] SENSR 드라이버만 실행합니다..."
        ros2 launch sensr_ros2_driver sensr_driver.launch.py
        ;;
    2)
        echo "[INFO] SENSR 드라이버 + Bag 레코더를 실행합니다..."
        echo "[INFO] Bag 파일은 ./sensr_bags/ 폴더에 저장됩니다."
        ros2 launch sensr_ros2_driver sensr_system.launch.py
        ;;
    3)
        echo "[INFO] RViz2 시각화를 실행합니다..."
        echo "[INFO] 먼저 다른 터미널에서 드라이버를 실행해야 합니다."
        ./run_rviz.sh
        ;;
    4)
        echo "[INFO] 전체 시스템을 실행합니다..."
        echo "[INFO] 5초 후 RViz2도 함께 실행됩니다..."
        
        # 백그라운드에서 드라이버 + 레코더 실행
        ros2 launch sensr_ros2_driver sensr_system.launch.py &
        LAUNCH_PID=$!
        
        # 5초 대기 후 RViz2 실행
        sleep 5
        ./run_rviz.sh &
        RVIZ_PID=$!
        
        # Ctrl+C 트랩 설정
        trap 'echo "[INFO] 프로그램 종료 중..."; kill $LAUNCH_PID $RVIZ_PID 2>/dev/null; exit' INT
        
        echo "[INFO] 전체 시스템이 실행 중입니다."
        echo "[INFO] Ctrl+C로 종료할 수 있습니다."
        
        # 백그라운드 프로세스들이 종료될 때까지 대기
        wait $LAUNCH_PID $RVIZ_PID
        ;;
    *)
        echo "[ERROR] 잘못된 선택입니다."
        exit 1
        ;;
esac

echo ""
echo "================================================"
echo "실행 완료!"
echo "================================================"