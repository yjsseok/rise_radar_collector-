#!/bin/bash

# SENSR LiDAR Data Recorder 설치 스크립트
# Seoul Robotics SENSR 시스템을 위한 데이터 수집 도구 설치

set -e  # 오류 발생 시 스크립트 중단

echo "================================================"
echo "SENSR LiDAR Data Recorder 설치 스크립트"
echo "================================================"

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 로그 함수들
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

# 루트 권한 확인
check_root() {
    if [[ $EUID -eq 0 ]]; then
        log_warning "루트 권한으로 실행하지 마세요. 일반 사용자로 실행하세요."
        exit 1
    fi
}

# 운영체제 확인
check_os() {
    log_info "운영체제 확인 중..."
    
    if [[ "$OSTYPE" != "linux-gnu"* ]]; then
        log_error "이 스크립트는 Linux에서만 실행됩니다"
        exit 1
    fi
    
    # Ubuntu 확인
    if ! command -v lsb_release &> /dev/null; then
        log_error "Ubuntu 시스템이 아닙니다"
        exit 1
    fi
    
    UBUNTU_VERSION=$(lsb_release -rs)
    log_success "Ubuntu $UBUNTU_VERSION 확인됨"
    
    # Ubuntu 18.04+ 확인
    if (( $(echo "$UBUNTU_VERSION 18.04" | awk '{print ($1 < $2)}') )); then
        log_error "Ubuntu 18.04 이상이 필요합니다"
        exit 1
    fi
}

# ROS 설치 확인
check_ros() {
    log_info "ROS 설치 확인 중..."
    
    if [ -z "$ROS_DISTRO" ]; then
        log_error "ROS가 설치되지 않았거나 환경이 설정되지 않았습니다"
        log_info "ROS Noetic 설치 가이드: http://wiki.ros.org/noetic/Installation/Ubuntu"
        exit 1
    fi
    
    log_success "ROS $ROS_DISTRO 확인됨"
}

# Python 확인
check_python() {
    log_info "Python 버전 확인 중..."
    
    if ! command -v python3 &> /dev/null; then
        log_error "Python3가 설치되지 않았습니다"
        exit 1
    fi
    
    PYTHON_VERSION=$(python3 --version | grep -oP '\d+\.\d+')
    log_success "Python $PYTHON_VERSION 확인됨"
    
    # Python 3.6+ 확인
    if (( $(echo "$PYTHON_VERSION 3.6" | awk '{print ($1 < $2)}') )); then
        log_error "Python 3.6 이상이 필요합니다"
        exit 1
    fi
}

# 시스템 패키지 설치
install_system_packages() {
    log_info "시스템 패키지 설치 중..."
    
    sudo apt-get update
    sudo apt-get install -y \
        build-essential \
        cmake \
        git \
        wget \
        curl \
        autoconf \
        automake \
        libtool \
        make \
        g++ \
        unzip \
        python3-pip \
        python3-dev
    
    log_success "시스템 패키지 설치 완료"
}

# Protobuf 설치
install_protobuf() {
    log_info "Protobuf 3.11.4 설치 중..."
    
    # 이미 설치된 경우 확인
    if command -v protoc &> /dev/null; then
        PROTOC_VERSION=$(protoc --version | grep -oP '\d+\.\d+\.\d+')
        if [[ "$PROTOC_VERSION" == "3.11.4" ]]; then
            log_success "Protobuf 3.11.4가 이미 설치되어 있습니다"
            return
        fi
    fi
    
    # 임시 디렉토리 생성
    TEMP_DIR=$(mktemp -d)
    cd "$TEMP_DIR"
    
    # Protobuf 소스 다운로드
    log_info "Protobuf 소스 다운로드 중..."
    curl -OL https://github.com/protocolbuffers/protobuf/releases/download/v3.11.4/protobuf-cpp-3.11.4.zip
    unzip protobuf-cpp-3.11.4.zip
    rm protobuf-cpp-3.11.4.zip
    
    cd protobuf-3.11.4
    
    # 빌드 및 설치
    log_info "Protobuf 빌드 중... (시간이 오래 걸릴 수 있습니다)"
    ./autogen.sh
    ./configure CXXFLAGS=-fPIC
    make -j$(nproc)
    
    log_info "Protobuf 테스트 중..."
    make check
    
    log_info "Protobuf 설치 중..."
    sudo make install
    sudo ldconfig
    
    # 임시 디렉토리 정리
    cd /
    rm -rf "$TEMP_DIR"
    
    log_success "Protobuf 3.11.4 설치 완료"
}

# Python 패키지 설치
install_python_packages() {
    log_info "Python 패키지 설치 중..."
    
    # pip 업그레이드
    python3 -m pip install --upgrade pip
    
    # requirements.txt에서 패키지 설치
    if [ -f "requirements.txt" ]; then
        python3 -m pip install -r requirements.txt
        log_success "Python 패키지 설치 완료"
    else
        log_warning "requirements.txt 파일을 찾을 수 없습니다"
        
        # 직접 설치
        python3 -m pip install \
            websocket-client \
            protobuf==3.11.4 \
            PyYAML \
            numpy
        log_success "필수 Python 패키지 설치 완료"
    fi
}

# Seoul Robotics SDK 설치 (선택사항)
install_sensr_sdk() {
    log_info "Seoul Robotics SDK 설치 여부를 선택하세요"
    echo "Seoul Robotics SENSR SDK와 Proto 정의를 다운로드하시겠습니까? (y/n)"
    read -r response
    
    if [[ "$response" =~ ^[Yy]$ ]]; then
        log_info "Seoul Robotics SDK 다운로드 중..."
        
        # SENSR SDK
        if [ ! -d "sensr_sdk" ]; then
            git clone https://github.com/seoulrobotics/sensr_sdk.git
            cd sensr_sdk/python
            ./configure.sh || log_warning "SDK 설정 실패 (무시하고 계속)"
            cd ../..
        else
            log_info "SENSR SDK가 이미 존재합니다"
        fi
        
        # SENSR Proto
        if [ ! -d "sensr_proto" ]; then
            git clone https://github.com/seoulrobotics/sensr_proto.git
        else
            log_info "SENSR Proto가 이미 존재합니다"
        fi
        
        log_success "Seoul Robotics SDK 설치 완료"
    else
        log_info "Seoul Robotics SDK 설치를 건너뜁니다"
    fi
}

# 권한 설정
setup_permissions() {
    log_info "파일 권한 설정 중..."
    
    # 실행 파일 권한 설정
    chmod +x main.py
    chmod +x test_connection.py
    
    # 디렉토리 생성 및 권한 설정
    mkdir -p logs output
    
    log_success "권한 설정 완료"
}

# 설정 파일 확인
check_config() {
    log_info "설정 파일 확인 중..."
    
    if [ ! -f "config/config.yaml" ]; then
        log_error "설정 파일이 없습니다: config/config.yaml"
        log_info "config/ 디렉토리에 config.yaml 파일을 생성하세요"
        exit 1
    fi
    
    log_success "설정 파일 확인됨"
}

# 연결 테스트
run_connection_test() {
    log_info "연결 테스트를 실행하시겠습니까? (y/n)"
    read -r response
    
    if [[ "$response" =~ ^[Yy]$ ]]; then
        log_info "연결 테스트 실행 중..."
        
        if python3 test_connection.py; then
            log_success "연결 테스트 성공!"
        else
            log_warning "연결 테스트 실패. 설정을 확인하세요"
        fi
    else
        log_info "연결 테스트를 건너뜁니다"
    fi
}

# 사용법 안내
show_usage() {
    log_success "설치가 완료되었습니다!"
    echo ""
    echo "사용법:"
    echo "  python3 main.py                    # 기본 실행"
    echo "  python3 main.py -c config.yaml    # 설정 파일 지정"
    echo "  python3 main.py -o /path/output   # 출력 디렉토리 지정"
    echo "  python3 main.py -v                # 상세 로그"
    echo ""
    echo "테스트:"
    echo "  python3 test_connection.py         # 연결 테스트"
    echo ""
    echo "문제 해결:"
    echo "  - 로그 파일: logs/sensr_recorder.log"
    echo "  - 설정 파일: config/config.yaml"
    echo "  - 출력 파일: output/"
}

# 메인 설치 프로세스
main() {
    log_info "SENSR LiDAR Data Recorder 설치를 시작합니다"
    
    # 사전 확인
    check_root
    check_os
    check_python
    check_ros
    
    # 설치 진행
    install_system_packages
    install_protobuf
    install_python_packages
    install_sensr_sdk
    
    # 설정
    setup_permissions
    check_config
    
    # 테스트
    run_connection_test
    
    # 완료
    show_usage
    
    log_success "설치가 성공적으로 완료되었습니다!"
}

# 스크립트 실행
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi