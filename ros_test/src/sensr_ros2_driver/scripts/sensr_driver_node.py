#!/usr/bin/env python3
"""
SENSR ROS2 Driver Node
Seoul Robotics SENSR 시스템을 위한 ROS2 드라이버 노드
"""

import os
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import websocket
import threading
import time
import logging
import queue
import struct
from typing import Callable, Optional, Dict, Any

from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, String
from geometry_msgs.msg import Point, Vector3, Quaternion
from diagnostic_msgs.msg import DiagnosticStatus

# Proto 파일 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
proto_path = os.path.join(current_dir, '..', '..', '..', '..', 'sensr_lidar_recorder', 'sensr_proto')
sys.path.append(proto_path)

try:
    from sensr_proto import output_pb2, point_cloud_pb2
    PROTOBUF_AVAILABLE = True
except ImportError:
    PROTOBUF_AVAILABLE = False
    print("Warning: SENSR protobuf files not found. Please copy sensr_proto folder.")


class SensrDriverNode(Node):
    """SENSR 드라이버 ROS2 노드"""
    
    def __init__(self):
        super().__init__('sensr_driver')
        
        # 파라미터 선언
        self.declare_parameter('host', '112.133.37.122')
        self.declare_parameter('output_port', 5050)
        self.declare_parameter('pointcloud_port', 5051)
        self.declare_parameter('frame_id', 'sensr')
        self.declare_parameter('publish_rate', 10.0)
        
        # 파라미터 가져오기
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.output_port = self.get_parameter('output_port').get_parameter_value().integer_value
        self.pointcloud_port = self.get_parameter('pointcloud_port').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers 생성
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, 
            '/sensr/pointcloud', 
            qos_profile
        )
        self.objects_pub = self.create_publisher(
            MarkerArray, 
            '/sensr/objects', 
            qos_profile
        )
        self.events_pub = self.create_publisher(
            String, 
            '/sensr/events', 
            qos_profile
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticStatus, 
            '/sensr/diagnostics', 
            qos_profile
        )
        
        # WebSocket 관련
        self.ws_output = None
        self.ws_pointcloud = None
        self.message_queue = queue.Queue()
        self.is_connected = False
        self.should_reconnect = True
        
        # 스레드 관리
        self.connection_threads = {}
        
        # 타이머 생성
        self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        self.get_logger().info(f'SENSR 드라이버 노드 시작됨')
        self.get_logger().info(f'연결 대상: {self.host}:{self.output_port}, {self.pointcloud_port}')
        
        # 연결 시작
        self.start_connection()
    
    def start_connection(self):
        """WebSocket 연결 시작"""
        if not PROTOBUF_AVAILABLE:
            self.get_logger().error('Protobuf 파일을 찾을 수 없습니다!')
            return
        
        try:
            # Output data WebSocket 연결
            output_url = f"ws://{self.host}:{self.output_port}"
            self.ws_output = websocket.WebSocketApp(
                output_url,
                on_message=self._on_output_message,
                on_error=self._on_error,
                on_close=self._on_close,
                on_open=self._on_open
            )
            
            # Point cloud WebSocket 연결
            pointcloud_url = f"ws://{self.host}:{self.pointcloud_port}"
            self.ws_pointcloud = websocket.WebSocketApp(
                pointcloud_url,
                on_message=self._on_pointcloud_message,
                on_error=self._on_error,
                on_close=self._on_close,
                on_open=self._on_open
            )
            
            # 스레드 시작
            self.should_reconnect = True
            
            # Output data 수신 스레드
            output_thread = threading.Thread(
                target=self._run_websocket,
                args=(self.ws_output, "output"),
                daemon=True
            )
            output_thread.start()
            self.connection_threads["output"] = output_thread
            
            # Point cloud 수신 스레드
            pointcloud_thread = threading.Thread(
                target=self._run_websocket,
                args=(self.ws_pointcloud, "pointcloud"),
                daemon=True
            )
            pointcloud_thread.start()
            self.connection_threads["pointcloud"] = pointcloud_thread
            
            self.get_logger().info('WebSocket 연결 스레드 시작됨')
            
        except Exception as e:
            self.get_logger().error(f'연결 시작 실패: {e}')
    
    def _run_websocket(self, ws: websocket.WebSocketApp, connection_type: str):
        """WebSocket 실행"""
        while self.should_reconnect and rclpy.ok():
            try:
                self.get_logger().info(f'{connection_type} WebSocket 연결 시작')
                ws.run_forever(ping_interval=30, ping_timeout=10)
            except Exception as e:
                self.get_logger().error(f'{connection_type} WebSocket 오류: {e}')
                
            if self.should_reconnect and rclpy.ok():
                self.get_logger().info(f'5초 후 {connection_type} 재연결 시도...')
                time.sleep(5)
    
    def _on_open(self, ws):
        """WebSocket 연결 성공"""
        self.is_connected = True
        if ws == self.ws_output:
            self.get_logger().info('Output WebSocket 연결 성공')
        elif ws == self.ws_pointcloud:
            self.get_logger().info('포인트클라우드 WebSocket 연결 성공')
    
    def _on_output_message(self, ws, message):
        """Output 메시지 수신"""
        try:
            self.get_logger().debug(f'Output 데이터 수신: {len(message)} bytes')
            self._process_message(message, "output_data")
        except Exception as e:
            self.get_logger().error(f'Output 메시지 처리 오류: {e}')
    
    def _on_pointcloud_message(self, ws, message):
        """포인트클라우드 메시지 수신"""
        try:
            self.get_logger().info(f'포인트클라우드 데이터 수신: {len(message)} bytes')
            self._process_message(message, "point_cloud")
        except Exception as e:
            self.get_logger().error(f'포인트클라우드 메시지 처리 오류: {e}')
    
    def _process_message(self, message, data_type: str):
        """수신된 메시지 처리"""
        message_data = {
            'data': message,
            'type': data_type,
            'timestamp': time.time()
        }
        
        try:
            self.message_queue.put_nowait(message_data)
        except queue.Full:
            self.get_logger().warning('메시지 큐가 가득참')
    
    def _on_error(self, ws, error):
        """WebSocket 오류"""
        self.get_logger().error(f'WebSocket 오류: {error}')
        self.is_connected = False
    
    def _on_close(self, ws, close_status_code=None, close_msg=None):
        """WebSocket 연결 종료"""
        self.is_connected = False
        self.get_logger().info(f'WebSocket 연결 종료: {close_status_code}')
    
    def timer_callback(self):
        """주기적으로 호출되는 타이머 콜백"""
        # 큐에서 메시지 처리
        messages_processed = 0
        while not self.message_queue.empty() and messages_processed < 100:
            try:
                message_data = self.message_queue.get_nowait()
                self.process_sensr_message(message_data)
                messages_processed += 1
            except queue.Empty:
                break
        
        if messages_processed > 0:
            self.get_logger().debug(f'처리된 메시지: {messages_processed}개')
    
    def process_sensr_message(self, message_data: Dict[str, Any]):
        """SENSR 메시지를 ROS 메시지로 변환하여 발행"""
        try:
            data_type = message_data['type']
            raw_data = message_data['data']
            timestamp = message_data['timestamp']
            
            if data_type == 'point_cloud':
                self.process_pointcloud_data(raw_data, timestamp)
            elif data_type == 'output_data':
                self.process_output_data(raw_data, timestamp)
                
        except Exception as e:
            self.get_logger().error(f'메시지 처리 오류: {e}')
    
    def process_pointcloud_data(self, raw_data: bytes, timestamp: float):
        """포인트클라우드 데이터 처리"""
        try:
            # Protobuf 파싱
            point_result = point_cloud_pb2.PointResult()
            point_result.ParseFromString(raw_data)
            
            self.get_logger().info(f'포인트클라우드 파싱: {len(point_result.points)}개')
            
            # PointCloud2 메시지 생성
            pc_msg = PointCloud2()
            pc_msg.header.stamp = self.get_clock().now().to_msg()
            pc_msg.header.frame_id = self.frame_id
            
            # 포인트 데이터 수집
            all_points = []
            all_intensities = []
            
            for point_cloud in point_result.points:
                points_data = point_cloud.points
                intensities_data = point_cloud.intensities
                
                # Vector3f 파싱 (12 bytes per point)
                num_points = len(points_data) // 12
                for i in range(num_points):
                    offset = i * 12
                    x, y, z = struct.unpack('<fff', points_data[offset:offset + 12])
                    all_points.append([x, y, z])
                
                # Intensities 파싱 (4 bytes per intensity)
                intensity_count = len(intensities_data) // 4
                for i in range(intensity_count):
                    offset = i * 4
                    intensity = struct.unpack('<f', intensities_data[offset:offset + 4])[0]
                    all_intensities.append(intensity)
            
            if all_points:
                # PointCloud2 필드 정의
                pc_msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                ]
                
                pc_msg.is_bigendian = False
                pc_msg.point_step = 16  # 4 * 4 bytes
                pc_msg.width = len(all_points)
                pc_msg.height = 1
                pc_msg.row_step = pc_msg.point_step * pc_msg.width
                pc_msg.is_dense = True
                
                # 포인트 데이터 직렬화
                point_data = []
                for i, (point, intensity) in enumerate(zip(all_points, all_intensities[:len(all_points)])):
                    point_data.extend(struct.pack('<ffff', point[0], point[1], point[2], intensity))
                
                pc_msg.data = bytes(point_data)
                
                # 발행
                self.pointcloud_pub.publish(pc_msg)
                self.get_logger().info(f'포인트클라우드 발행: {len(all_points)}개 포인트')
            
        except Exception as e:
            self.get_logger().error(f'포인트클라우드 처리 오류: {e}')
    
    def process_output_data(self, raw_data: bytes, timestamp: float):
        """출력 데이터 (객체) 처리"""
        try:
            # Protobuf 파싱
            output_message = output_pb2.OutputMessage()
            output_message.ParseFromString(raw_data)
            
            # StreamMessage 처리
            if output_message.HasField('stream'):
                self.process_stream_message(output_message.stream, timestamp)
            
            # EventMessage 처리
            if output_message.HasField('event'):
                self.process_event_message(output_message.event, timestamp)
                
        except Exception as e:
            self.get_logger().error(f'출력 데이터 처리 오류: {e}')
    
    def process_stream_message(self, stream, timestamp: float):
        """스트림 메시지 (객체) 처리"""
        try:
            # MarkerArray 생성
            marker_array = MarkerArray()
            marker_array.markers = []
            
            for i, obj in enumerate(stream.objects):
                marker = Marker()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = self.frame_id
                marker.ns = "sensr_objects"
                marker.id = obj.id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                # 위치 설정
                marker.pose.position.x = obj.bbox.position.x
                marker.pose.position.y = obj.bbox.position.y
                marker.pose.position.z = obj.bbox.position.z
                
                # 크기 설정
                marker.scale.x = obj.bbox.size.x
                marker.scale.y = obj.bbox.size.y
                marker.scale.z = obj.bbox.size.z
                
                # 색상 설정 (라벨에 따라)
                if obj.label == 1:  # CAR
                    marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
                elif obj.label == 2:  # PEDESTRIAN
                    marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
                elif obj.label == 3:  # CYCLIST
                    marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0
                else:  # MISC
                    marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0
                
                marker.color.a = 0.7
                marker.lifetime.sec = 1  # 1초 후 사라짐
                
                marker_array.markers.append(marker)
            
            if marker_array.markers:
                self.objects_pub.publish(marker_array)
                self.get_logger().debug(f'객체 발행: {len(marker_array.markers)}개')
                
        except Exception as e:
            self.get_logger().error(f'스트림 메시지 처리 오류: {e}')
    
    def process_event_message(self, event, timestamp: float):
        """이벤트 메시지 처리"""
        try:
            # 간단한 문자열로 발행
            event_str = f"Events at {timestamp}"
            msg = String()
            msg.data = event_str
            self.events_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'이벤트 메시지 처리 오류: {e}')
    
    def destroy_node(self):
        """노드 종료"""
        self.should_reconnect = False
        if self.ws_output:
            self.ws_output.close()
        if self.ws_pointcloud:
            self.ws_pointcloud.close()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SensrDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()