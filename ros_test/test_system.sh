#!/bin/bash
# SENSR ROS2 시스템 테스트 스크립트

echo "================================================"
echo "SENSR ROS2 Driver - System Test"
echo "================================================"

# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 워크스페이스가 빌드되었는지 확인
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "[ERROR] 워크스페이스가 빌드되지 않았습니다. 먼저 빌드해주세요."
    echo "./build_and_run.sh"
    exit 1
fi

echo "[INFO] 시스템 테스트 시작..."

# 1. 네트워크 연결 테스트
echo ""
echo "1. 네트워크 연결 테스트..."
HOST="112.133.37.122"
if ping -c 1 $HOST &> /dev/null; then
    echo "✅ $HOST 연결 성공"
else
    echo "❌ $HOST 연결 실패"
    echo "   네트워크 연결을 확인해주세요."
fi

# 2. 포트 연결 테스트
echo ""
echo "2. 포트 연결 테스트..."
for port in 5050 5051; do
    if timeout 3 bash -c "</dev/tcp/$HOST/$port" 2>/dev/null; then
        echo "✅ 포트 $port 연결 성공"
    else
        echo "❌ 포트 $port 연결 실패"
    fi
done

# 3. ROS2 패키지 확인
echo ""
echo "3. ROS2 패키지 확인..."
if ros2 pkg list | grep sensr_ros2_driver > /dev/null; then
    echo "✅ sensr_ros2_driver 패키지 확인됨"
else
    echo "❌ sensr_ros2_driver 패키지 없음"
fi

# 4. Launch 파일 확인
echo ""
echo "4. Launch 파일 확인..."
LAUNCH_FILES=(
    "sensr_driver.launch.py"
    "sensr_system.launch.py"
)

for launch_file in "${LAUNCH_FILES[@]}"; do
    if ros2 launch sensr_ros2_driver $launch_file --show-args &> /dev/null; then
        echo "✅ $launch_file 확인됨"
    else
        echo "❌ $launch_file 오류"
    fi
done

# 5. Python 의존성 확인
echo ""
echo "5. Python 의존성 확인..."
PYTHON_DEPS=("websocket" "yaml")

for dep in "${PYTHON_DEPS[@]}"; do
    if python3 -c "import $dep" 2>/dev/null; then
        echo "✅ Python $dep 모듈 확인됨"
    else
        echo "❌ Python $dep 모듈 없음"
        echo "   pip3 install websocket-client pyyaml"
    fi
done

# 6. 프로토버프 파일 확인
echo ""
echo "6. Protobuf 파일 확인..."
PROTO_DIR="src/sensr_ros2_driver/sensr_proto"
if [ -d "$PROTO_DIR" ]; then
    PROTO_FILES=$(find "$PROTO_DIR" -name "*_pb2.py" | wc -l)
    if [ $PROTO_FILES -gt 0 ]; then
        echo "✅ Protobuf 파일 확인됨 ($PROTO_FILES 개)"
    else
        echo "❌ Protobuf 파일이 생성되지 않음"
    fi
else
    echo "❌ sensr_proto 디렉토리 없음"
    echo "   cp -r ../sensr_lidar_recorder/sensr_proto src/sensr_ros2_driver/"
fi

# 7. 간단한 노드 실행 테스트 (5초간)
echo ""
echo "7. 노드 실행 테스트 (5초간)..."
echo "   드라이버 노드를 5초간 실행합니다..."

timeout 5s ros2 launch sensr_ros2_driver sensr_driver.launch.py &> /tmp/sensr_test.log &
TEST_PID=$!

sleep 6  # 5초 + 여유시간

if wait $TEST_PID; then
    echo "✅ 노드 실행 테스트 완료"
else
    echo "❌ 노드 실행 중 오류 발생"
    echo "   로그 확인: cat /tmp/sensr_test.log"
fi

# 8. 출력 디렉토리 권한 확인
echo ""
echo "8. 출력 디렉토리 확인..."
OUTPUT_DIR="./sensr_bags"
if mkdir -p "$OUTPUT_DIR" 2>/dev/null; then
    echo "✅ $OUTPUT_DIR 생성 가능"
    if [ -w "$OUTPUT_DIR" ]; then
        echo "✅ $OUTPUT_DIR 쓰기 가능"
    else
        echo "❌ $OUTPUT_DIR 쓰기 권한 없음"
    fi
else
    echo "❌ $OUTPUT_DIR 생성 실패"
fi

echo ""
echo "================================================"
echo "시스템 테스트 완료!"
echo "================================================"
echo ""
echo "테스트 결과를 확인하고 문제가 있으면 다음을 시도해보세요:"
echo ""
echo "• 네트워크 문제: ping $HOST"
echo "• 포트 문제: telnet $HOST 5050"
echo "• 패키지 문제: ./build_and_run.sh"
echo "• 의존성 문제: pip3 install websocket-client pyyaml"
echo "• 권한 문제: chmod +x *.sh src/sensr_ros2_driver/scripts/*.py"
echo ""
echo "실행 방법:"
echo "• 간단 실행: ./quick_start.sh"
echo "• 수동 실행: ros2 launch sensr_ros2_driver sensr_system.launch.py"
echo "• Docker 실행: docker-compose up"