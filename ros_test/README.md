# SENSR ROS2 Driver

Seoul Robotics SENSR 시스템을 위한 ROS2 드라이버 및 Bag 레코더

## 특징

- ✅ **ROS2 Native**: ROS2 Humble 지원
- ✅ **실시간 데이터**: WebSocket을 통한 실시간 스트리밍
- ✅ **표준 메시지**: sensor_msgs/PointCloud2, visualization_msgs/MarkerArray 사용
- ✅ **자동 Bag 기록**: 설정 가능한 시간 간격으로 자동 분할
- ✅ **멀티스레드**: 안정적인 데이터 처리

## 제공하는 ROS2 토픽

| 토픽 | 메시지 타입 | 설명 |
|------|-------------|------|
| `/sensr/pointcloud` | sensor_msgs/PointCloud2 | 라이다 포인트클라우드 데이터 |
| `/sensr/objects` | visualization_msgs/MarkerArray | 검출된 객체들 (3D 바운딩 박스) |
| `/sensr/events` | std_msgs/String | 시스템 이벤트 |
| `/sensr/diagnostics` | diagnostic_msgs/DiagnosticStatus | 시스템 상태 |

## 설치 및 빌드

### 1. 의존성 설치
```bash
# ROS2 Humble 환경에서
sudo apt update
sudo apt install ros-humble-sensor-msgs ros-humble-visualization-msgs
pip3 install websocket-client pyyaml
```

### 2. sensr_proto 준비
```bash
# 기존 sensr_lidar_recorder의 protobuf 파일들을 복사
cp -r ../sensr_lidar_recorder/sensr_proto ./src/sensr_ros2_driver/
```

### 3. 빌드
```bash
# 자동 빌드 스크립트 실행
chmod +x build_and_run.sh
./build_and_run.sh

# 또는 수동 빌드
source /opt/ros/humble/setup.bash
colcon build --packages-select sensr_ros2_driver
source install/setup.bash
```

## 사용 방법

### 1. 기본 드라이버 실행
```bash
ros2 launch sensr_ros2_driver sensr_driver.launch.py
```

### 2. 드라이버 + 자동 Bag 레코더 실행
```bash
ros2 launch sensr_ros2_driver sensr_system.launch.py
```

### 3. 커스텀 설정으로 실행
```bash
# IP 주소 변경
ros2 launch sensr_ros2_driver sensr_system.launch.py host:=192.168.1.100

# Bag 파일 저장 위치 변경
ros2 launch sensr_ros2_driver sensr_system.launch.py output_directory:=/home/user/sensr_data

# Bag 파일 분할 시간 변경 (초)
ros2 launch sensr_ros2_driver sensr_system.launch.py bag_duration:=120
```

### 4. 파라미터 파일로 실행
```bash
ros2 launch sensr_ros2_driver sensr_driver.launch.py --ros-args --params-file src/sensr_ros2_driver/config/sensr_params.yaml
```

## 데이터 확인

### 1. 토픽 확인
```bash
# 발행 중인 토픽 목록
ros2 topic list

# 포인트클라우드 데이터 확인
ros2 topic echo /sensr/pointcloud

# 객체 데이터 확인  
ros2 topic echo /sensr/objects
```

### 2. Bag 파일 확인
```bash
# Bag 파일 정보 확인
ros2 bag info ./sensr_bags/sensr_data_20250902_140530

# Bag 파일 재생
ros2 bag play ./sensr_bags/sensr_data_20250902_140530
```

### 3. RViz2로 시각화
```bash
# RViz2 실행
rviz2

# 또는 설정 파일과 함께
rviz2 -d src/sensr_ros2_driver/config/sensr.rviz
```

## 설정 파라미터

### sensr_driver 노드
- `host`: SENSR 서버 IP 주소 (기본값: "112.133.37.122")
- `output_port`: 객체 데이터 포트 (기본값: 5050)
- `pointcloud_port`: 포인트클라우드 포트 (기본값: 5051)
- `frame_id`: 센서 프레임 ID (기본값: "sensr")
- `publish_rate`: 발행 주기 Hz (기본값: 10.0)

### sensr_bag_recorder 노드
- `output_directory`: Bag 파일 저장 디렉토리 (기본값: "./sensr_bags")
- `bag_duration`: Bag 파일 분할 시간(초) (기본값: 60)
- `max_bag_size`: 최대 Bag 파일 크기 MB (기본값: 1024)
- `topics`: 기록할 토픽 목록

## 문제 해결

### 1. 연결 실패
```bash
# 네트워크 연결 확인
ping 112.133.37.122

# 포트 확인
telnet 112.133.37.122 5050
telnet 112.133.37.122 5051
```

### 2. Protobuf 오류
```bash
# sensr_proto 폴더가 있는지 확인
ls src/sensr_ros2_driver/sensr_proto/

# Python 경로 확인
python3 -c "import sys; print(sys.path)"
```

### 3. 포인트클라우드 데이터 없음
- SENSR 서버에서 포트 5051 포인트클라우드 스트림 활성화 확인
- 로그에서 "포인트클라우드 데이터 수신" 메시지 확인

### 4. Bag 파일 권한 오류
```bash
# 출력 디렉토리 권한 확인
sudo chown -R $USER:$USER ./sensr_bags
chmod -R 755 ./sensr_bags
```

## 아키텍처

```
SENSR Server (112.133.37.122)
├── Port 5050 (WebSocket) → Objects/Events
└── Port 5051 (WebSocket) → Point Cloud

↓

sensr_driver_node
├── WebSocket Client (Output)
├── WebSocket Client (Point Cloud)  
├── Protobuf Decoder
└── ROS2 Publishers

↓

ROS2 Topics
├── /sensr/pointcloud
├── /sensr/objects
├── /sensr/events
└── /sensr/diagnostics

↓

sensr_bag_recorder_node
└── ros2 bag record (자동 분할)
```

## 라이센스

MIT License

## 문의

이슈가 있으시면 GitHub Issues를 이용해주세요.