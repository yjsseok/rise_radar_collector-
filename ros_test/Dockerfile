# SENSR ROS2 Driver Docker Image
FROM ros:humble-desktop

# 작업 디렉토리 설정
WORKDIR /workspace

# 시스템 의존성 설치
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-websocket \
    python3-yaml \
    net-tools \
    iputils-ping \
    telnet \
    && rm -rf /var/lib/apt/lists/*

# Python 의존성 설치
RUN pip3 install websocket-client pyyaml

# 워크스페이스 복사
COPY . /workspace/

# 실행 권한 설정
RUN chmod +x /workspace/*.sh \
    && chmod +x /workspace/src/sensr_ros2_driver/scripts/*.py \
    && chmod +x /workspace/src/sensr_ros2_driver/launch/*.py

# ROS2 환경 설정
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# 빌드
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select sensr_ros2_driver"

# 포트 노출 (ROS2 DDS)
EXPOSE 11311 7400-7500

# 기본 명령어
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch sensr_ros2_driver sensr_system.launch.py"]