#!/bin/bash

# SENSR LiDAR Data Recorder - Ubuntu 자동 설치 스크립트
# ROS2 Humble 환경 구축 및 의존성 설치

set -e

echo "================================================"
echo "🚀 SENSR LiDAR Data Recorder - Ubuntu Setup"
echo "================================================"

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Ubuntu 버전 확인
log_info "Ubuntu 버전 확인 중..."
UBUNTU_VERSION=$(lsb_release -rs)
log_success "Ubuntu $UBUNTU_VERSION 확인됨"

# ROS2 설치 확인
if [ -n "$ROS_DISTRO" ]; then
    if [ "$ROS_DISTRO" == "humble" ]; then
        log_success "ROS2 Humble이 이미 설치되어 있습니다"
    else
        log_warning "ROS $ROS_DISTRO가 설치되어 있습니다. ROS2 Humble 권장"
    fi
else
    log_info "ROS2 Humble 설치 시작..."
    
    # 시스템 업데이트
    log_info "시스템 업데이트 중..."
    sudo apt update && sudo apt upgrade -y
    
    # 필수 도구 설치
    sudo apt install -y curl gnupg2 lsb-release build-essential
    
    # ROS2 키 추가
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # ROS2 설치
    sudo apt update
    sudo apt install -y ros-humble-desktop ros-humble-rosbag2
    
    # rosdep 설정
    sudo apt install -y python3-rosdep
    sudo rosdep init || true
    rosdep update
    
    # 환경 설정
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source /opt/ros/humble/setup.bash
    
    # ROS2 개발 도구
    sudo apt install -y python3-colcon-common-extensions
    
    log_success "ROS2 Humble 설치 완료"
fi

# Python ROS2 패키지 설치
log_info "Python ROS2 패키지 설치 중..."
sudo apt install -y \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-visualization-msgs \
    ros-humble-diagnostic-msgs \
    python3-pip \
    python3-dev \
    python3-yaml \
    python3-rclpy

log_success "Python ROS2 패키지 설치 완료"

# Python 의존성 설치
log_info "Python 의존성 설치 중..."
python3 -m pip install --upgrade pip
pip3 install websocket-client protobuf>=3.20.3,<4.0.0 PyYAML numpy requests aiohttp

log_success "Python 의존성 설치 완료"

# 디렉토리 설정
log_info "프로젝트 디렉토리 설정 중..."
mkdir -p logs output simple_output proto
chmod +x run_ubuntu.sh run_simple_ubuntu.sh setup_pointcloud_ubuntu.sh

log_success "디렉토리 설정 완료"

# 설정 파일 확인
if [ ! -f "config/config.yaml" ]; then
    log_warning "설정 파일이 없습니다. config/config.yaml을 생성하세요"
else
    log_success "설정 파일 확인됨"
fi

# SENSR 포인트클라우드 설정
log_info "SENSR 포인트클라우드 설정 확인 중..."
read -p "SENSR 포인트클라우드 스트리밍을 활성화하시겠습니까? (y/n): " setup_pointcloud
if [[ $setup_pointcloud =~ ^[Yy]$ ]]; then
    log_info "포인트클라우드 스트리밍 설정 중..."
    python3 enable_pointcloud.py
    if [ $? -eq 0 ]; then
        log_success "포인트클라우드 설정 완료!"
    else
        log_warning "포인트클라우드 설정 실패 - 서버 연결을 확인해주세요"
    fi
fi

# 연결 테스트
read -p "연결 테스트를 실행하시겠습니까? (y/n): " run_test
if [[ $run_test =~ ^[Yy]$ ]]; then
    log_info "연결 테스트 중..."
    python3 test_connection.py
    if [ $? -eq 0 ]; then
        log_success "연결 테스트 통과!"
    else
        log_warning "연결 테스트 실패 - 설정을 확인해주세요"
    fi
fi

# ROS2 환경 테스트
if [ -n "$ROS_DISTRO" ] && [ "$ROS_DISTRO" == "humble" ]; then
    log_info "ROS2 환경 테스트 중..."
    source /opt/ros/humble/setup.bash
    
    python3 -c "
try:
    import rclpy
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    print('✅ ROS2 Python 패키지 정상 작동')
except ImportError as e:
    print(f'⚠️ ROS2 패키지 로딩 실패: {e}')
    print('⚠️ Windows 환경에서는 Mock 모드로 동작합니다')
"
    
    log_success "ROS2 환경 테스트 완료"
fi

echo ""
log_success "Ubuntu 설정이 완료되었습니다!"
echo ""
echo "============ 사용법 ============"
echo ""
echo "🥇 권장 (경량 모드):"
echo "  ./run_simple_ubuntu.sh              # 포인트클라우드만, 서버 부하 최소"
echo "  python3 simple_pointcloud_recorder.py --interval 0.5"
echo ""
echo "🔧 완전 기능:"
echo "  ./run_ubuntu.sh                     # ROS2 완전 기능"
echo "  python3 main.py --pointcloud-interval 0.5 --pointcloud-only"
echo ""
echo "🛠️ 관리 도구:"
echo "  python3 sensr_manager.py status     # 시스템 상태"
echo "  python3 sensr_manager.py sensor list # 센서 목록"
echo "  ./setup_pointcloud_ubuntu.sh        # 포인트클라우드 설정"
echo ""
echo "📊 간격 옵션 (서버 부하 제어):"
echo "  --interval 0.5     # 균형 (권장)"
echo "  --interval 1.0     # 장기 모니터링"
echo "  --interval 2.0     # 다중 사용자 환경"
echo ""
echo "🧪 테스트:"
echo "  python3 test_connection.py          # 연결 테스트"
echo "  python3 enable_pointcloud.py        # 포인트클라우드 설정"
echo ""
if [ -n "$ROS_DISTRO" ] && [ "$ROS_DISTRO" == "humble" ]; then
    echo "ROS2 환경을 활성화하려면:"
    echo "  source /opt/ros/humble/setup.bash"
fi
echo ""

log_info "설치 완료! 프로그램을 실행해보세요."