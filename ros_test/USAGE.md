# SENSR ROS2 Driver 사용 가이드

## 🚀 빠른 시작

### 1단계: 시스템 테스트
```bash
./test_system.sh
```

### 2단계: 빌드
```bash
./build_and_run.sh
```

### 3단계: 실행
```bash
./quick_start.sh
```

## 📁 프로젝트 구조

```
ros_test/
├── 📄 README.md                    # 상세 문서
├── 📄 USAGE.md                     # 이 파일 (사용 가이드)
├── 🔧 build_and_run.sh            # 빌드 스크립트
├── 🚀 quick_start.sh               # 빠른 시작 스크립트  
├── 🧪 test_system.sh               # 시스템 테스트
├── 🎨 run_rviz.sh                  # RViz2 실행
├── 🐳 Dockerfile                   # Docker 이미지
├── 🐳 docker-compose.yml           # Docker Compose
└── src/sensr_ros2_driver/
    ├── 📄 package.xml              # ROS2 패키지 정의
    ├── 📄 CMakeLists.txt           # 빌드 설정
    ├── scripts/
    │   ├── 🤖 sensr_driver_node.py     # 메인 드라이버 노드
    │   └── 💾 sensr_bag_recorder.py    # Bag 레코더 노드
    ├── launch/
    │   ├── 🚀 sensr_driver.launch.py   # 드라이버 실행
    │   └── 🚀 sensr_system.launch.py   # 전체 시스템 실행
    ├── config/
    │   ├── ⚙️ sensr_params.yaml        # 파라미터 설정
    │   └── 🎨 sensr.rviz               # RViz2 설정
    └── sensr_proto/                # Protobuf 파일들
```

## 🛠️ 실행 방법들

### 방법 1: 자동 스크립트 (추천)
```bash
./quick_start.sh
# 1) 드라이버만
# 2) 드라이버 + Bag 레코더  ⭐ 추천
# 3) RViz2만
# 4) 전체 시스템 + RViz2
```

### 방법 2: 수동 실행
```bash
# 환경 설정
source /opt/ros/humble/setup.bash
source install/setup.bash

# 드라이버 + Bag 레코더
ros2 launch sensr_ros2_driver sensr_system.launch.py

# 별도 터미널에서 RViz2
rviz2 -d src/sensr_ros2_driver/config/sensr.rviz
```

### 방법 3: Docker 실행
```bash
# 드라이버만
docker-compose up sensr_driver

# 드라이버 + RViz2
docker-compose --profile visualization up
```

### 방법 4: 개별 노드 실행
```bash
# 드라이버 노드만
ros2 run sensr_ros2_driver sensr_driver_node.py

# Bag 레코더만  
ros2 run sensr_ros2_driver sensr_bag_recorder.py
```

## 📊 데이터 확인

### ROS2 토픽 확인
```bash
# 토픽 목록
ros2 topic list

# 포인트클라우드 데이터 확인
ros2 topic echo /sensr/pointcloud

# 객체 데이터 확인
ros2 topic echo /sensr/objects

# 토픽 주파수 확인
ros2 topic hz /sensr/pointcloud
```

### Bag 파일 확인
```bash
# Bag 파일 정보
ros2 bag info ./sensr_bags/sensr_data_20250902_140530

# Bag 파일 재생
ros2 bag play ./sensr_bags/sensr_data_20250902_140530
```

## ⚙️ 설정 변경

### IP 주소 변경
```bash
ros2 launch sensr_ros2_driver sensr_system.launch.py host:=192.168.1.100
```

### Bag 파일 설정
```bash
# 저장 위치 변경
ros2 launch sensr_ros2_driver sensr_system.launch.py output_directory:=/home/user/data

# 분할 시간 변경 (초)
ros2 launch sensr_ros2_driver sensr_system.launch.py bag_duration:=120
```

### 파라미터 파일 사용
```bash
# config/sensr_params.yaml 편집 후
ros2 launch sensr_ros2_driver sensr_driver.launch.py --ros-args --params-file src/sensr_ros2_driver/config/sensr_params.yaml
```

## 🔍 문제 해결

### 1. 연결 문제
```bash
# 네트워크 확인
ping 112.133.37.122
telnet 112.133.37.122 5050
telnet 112.133.37.122 5051
```

### 2. 빌드 문제
```bash
# 의존성 재설치
sudo apt update
sudo apt install ros-humble-sensor-msgs ros-humble-visualization-msgs
pip3 install websocket-client pyyaml

# 클린 빌드
rm -rf build install log
./build_and_run.sh
```

### 3. 권한 문제
```bash
chmod +x *.sh
chmod +x src/sensr_ros2_driver/scripts/*.py
sudo chown -R $USER:$USER ./sensr_bags
```

### 4. Protobuf 문제
```bash
# sensr_proto 파일 확인 및 복사
ls -la src/sensr_ros2_driver/sensr_proto/
cp -r ../sensr_lidar_recorder/sensr_proto src/sensr_ros2_driver/
```

## 💡 팁

### 성능 최적화
- `publish_rate` 파라미터 조정 (기본값: 10Hz)
- Bag 파일 압축 활성화 (zstd 압축 사용)
- 큰 파일용 `max_bag_size` 증가

### 모니터링
```bash
# 시스템 리소스 모니터링
htop

# 네트워크 트래픽 모니터링  
sudo netstat -i

# ROS2 노드 상태 확인
ros2 node list
ros2 node info /sensr_driver
```

### 디버깅
```bash
# 상세 로그 확인
ros2 launch sensr_ros2_driver sensr_system.launch.py --ros-args --log-level DEBUG

# 특정 노드 로그
ros2 run sensr_ros2_driver sensr_driver_node.py --ros-args --log-level DEBUG
```

## 🆘 지원

문제가 발생하면:
1. `./test_system.sh` 실행하여 시스템 상태 확인
2. README.md의 문제 해결 섹션 참조
3. 로그 파일 확인 (`/tmp/sensr_test.log`)
4. GitHub Issues 생성