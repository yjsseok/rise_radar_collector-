#!/usr/bin/env python3
"""
SENSR ROS2 Bag Recorder
ROS2 토픽들을 자동으로 bag 파일로 기록하는 노드
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import subprocess
import signal
import time
from datetime import datetime


class SensrBagRecorderNode(Node):
    """SENSR ROS2 Bag 레코더 노드"""
    
    def __init__(self):
        super().__init__('sensr_bag_recorder')
        
        # 파라미터 선언
        self.declare_parameter('output_directory', './sensr_bags')
        self.declare_parameter('bag_duration', 60)  # 60초마다 새 파일
        self.declare_parameter('max_bag_size', 1024)  # MB
        self.declare_parameter('topics', [
            '/sensr/pointcloud',
            '/sensr/objects',
            '/sensr/events',
            '/sensr/diagnostics'
        ])
        
        # 파라미터 가져오기
        self.output_directory = self.get_parameter('output_directory').get_parameter_value().string_value
        self.bag_duration = self.get_parameter('bag_duration').get_parameter_value().integer_value
        self.max_bag_size = self.get_parameter('max_bag_size').get_parameter_value().integer_value
        self.topics = self.get_parameter('topics').get_parameter_value().string_array_value
        
        # 출력 디렉토리 생성
        os.makedirs(self.output_directory, exist_ok=True)
        
        # Bag 레코딩 프로세스
        self.bag_process = None
        self.current_bag_path = None
        
        # 타이머 생성 (bag 파일 교체용)
        self.create_timer(float(self.bag_duration), self.rotate_bag_file)
        
        # 시그널 핸들러 등록
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.get_logger().info('SENSR Bag 레코더 시작됨')
        self.get_logger().info(f'출력 디렉토리: {self.output_directory}')
        self.get_logger().info(f'기록할 토픽: {self.topics}')
        
        # 첫 번째 bag 파일 시작
        self.start_new_bag()
    
    def start_new_bag(self):
        """새로운 bag 파일 시작"""
        try:
            # 기존 bag 파일 중지
            if self.bag_process:
                self.stop_current_bag()
            
            # 새 파일명 생성
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            bag_name = f"sensr_data_{timestamp}"
            self.current_bag_path = os.path.join(self.output_directory, bag_name)
            
            # ros2 bag record 명령 생성
            cmd = [
                'ros2', 'bag', 'record',
                '--output', self.current_bag_path,
                '--max-bag-size', str(self.max_bag_size * 1024 * 1024),  # MB to bytes
                '--compression-mode', 'file',
                '--compression-format', 'zstd'
            ]
            
            # 토픽 추가
            for topic in self.topics:
                cmd.extend([topic])
            
            self.get_logger().info(f'새 bag 파일 시작: {bag_name}')
            self.get_logger().info(f'명령: {" ".join(cmd)}')
            
            # 프로세스 시작
            self.bag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid if os.name != 'nt' else None
            )
            
        except Exception as e:
            self.get_logger().error(f'Bag 파일 시작 오류: {e}')
    
    def stop_current_bag(self):
        """현재 bag 파일 중지"""
        if self.bag_process:
            try:
                self.get_logger().info('현재 bag 파일 중지 중...')
                
                if os.name != 'nt':  # Linux/Mac
                    os.killpg(os.getpgid(self.bag_process.pid), signal.SIGTERM)
                else:  # Windows
                    self.bag_process.terminate()
                
                # 프로세스 종료 대기
                self.bag_process.wait(timeout=10)
                self.get_logger().info(f'Bag 파일 저장 완료: {self.current_bag_path}')
                
            except subprocess.TimeoutExpired:
                self.get_logger().warning('Bag 프로세스 강제 종료')
                if os.name != 'nt':
                    os.killpg(os.getpgid(self.bag_process.pid), signal.SIGKILL)
                else:
                    self.bag_process.kill()
            except Exception as e:
                self.get_logger().error(f'Bag 파일 중지 오류: {e}')
            finally:
                self.bag_process = None
    
    def rotate_bag_file(self):
        """Bag 파일 교체 (타이머 콜백)"""
        self.get_logger().info('Bag 파일 교체 중...')
        self.start_new_bag()
    
    def signal_handler(self, signum, frame):
        """시그널 핸들러"""
        self.get_logger().info(f'시그널 {signum} 수신됨. 종료 중...')
        self.cleanup()
    
    def cleanup(self):
        """정리 작업"""
        self.stop_current_bag()
        self.get_logger().info('Bag 레코더 종료됨')
    
    def destroy_node(self):
        """노드 종료"""
        self.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SensrBagRecorderNode()
        
        # 멀티스레드 실행기 사용
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        
    except Exception as e:
        print(f'노드 실행 오류: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()