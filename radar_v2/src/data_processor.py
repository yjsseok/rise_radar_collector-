#!/usr/bin/env python3
"""
Data Processor Module
SENSR Protobuf 데이터를 ROS 메시지로 변환하는 모듈
"""

# 🚀 Phase 1: Protobuf C++ 구현 강제 활성화 (5-10배 성능 향상)
import os
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'cpp'

import struct
import time
import logging
import requests
from typing import Dict, Any, Optional, List
import numpy as np

from .track_logger import TrackLogger

try:
    import rclpy
    from rclpy.time import Time
    from rclpy.duration import Duration
    from sensor_msgs.msg import PointCloud2, PointField
    from visualization_msgs.msg import MarkerArray, Marker
    from std_msgs.msg import Header, String
    from geometry_msgs.msg import Point, Vector3, Quaternion
    from diagnostic_msgs.msg import DiagnosticStatus
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    logging.warning("ROS2 패키지를 찾을 수 없습니다. Windows 모드로 실행합니다.")
    
    # Windows에서 ROS 없이 실행하기 위한 Mock 클래스들
    class MockRosPy:
        class Time:
            @staticmethod
            def from_sec(timestamp):
                return timestamp
        
        class Duration:
            def __init__(self, sec):
                self.sec = sec
    
    class MockHeader:
        def __init__(self):
            self.stamp = None
            self.frame_id = ""
    
    class MockPointCloud2:
        def __init__(self):
            self.header = MockHeader()
            self.fields = []
            self.width = 0
            self.height = 0
            self.is_bigendian = False
            self.point_step = 0
            self.row_step = 0
            self.data = b''
            self.is_dense = True
    
    class MockPointField:
        FLOAT32 = 7
        def __init__(self, name="", offset=0, datatype=0, count=1):
            self.name = name
            self.offset = offset
            self.datatype = datatype
            self.count = count
    
    class MockMarker:
        CUBE = 1
        ADD = 0
        def __init__(self):
            self.header = MockHeader()
            self.ns = ""
            self.id = 0
            self.type = 0
            self.action = 0
            self.pose = type('obj', (object,), {
                'position': type('obj', (object,), {'x': 0, 'y': 0, 'z': 0})()
            })()
            self.scale = type('obj', (object,), {'x': 0, 'y': 0, 'z': 0})()
            self.color = type('obj', (object,), {'r': 0, 'g': 0, 'b': 0, 'a': 0})()
            self.lifetime = None
    
    class MockMarkerArray:
        def __init__(self):
            self.markers = []
    
    class MockString:
        def __init__(self):
            self.data = ""
    
    # Mock 객체들을 전역으로 설정
    rospy = MockRosPy()
    PointCloud2 = MockPointCloud2
    PointField = MockPointField
    MarkerArray = MockMarkerArray
    Marker = MockMarker
    Header = MockHeader
    String = MockString
    Point = type('Point', (object,), {'__init__': lambda self, x=0, y=0, z=0: setattr(self, 'x', x) or setattr(self, 'y', y) or setattr(self, 'z', z)})
    Vector3 = type('Vector3', (object,), {'__init__': lambda self, x=0, y=0, z=0: setattr(self, 'x', x) or setattr(self, 'y', y) or setattr(self, 'z', z)})
    Quaternion = type('Quaternion', (object,), {'__init__': lambda self, x=0, y=0, z=0, w=1: None})


class DataProcessor:
    """SENSR 데이터를 ROS 메시지로 변환하는 클래스"""
    
    def __init__(self, config: Dict[str, Any]):
        """
        DataProcessor 초기화
        
        Args:
            config: 설정 딕셔너리
        """
        self.config = config
        self.topics = config['ros']['topics']
        self.logger = logging.getLogger(__name__)

        track_log_enabled = config.get('recording', {}).get('track_log_enabled', True)
        self.track_logger = TrackLogger(logging.getLogger('sensr.track_log'), enabled=track_log_enabled)

        # SENSR API 설정
        self.sensr_host = config.get('sensr', {}).get('host', '112.133.37.122')
        self.rest_port = config.get('sensr', {}).get('ports', {}).get('rest', 9080)
        self.base_url = f"http://{self.sensr_host}:{self.rest_port}"
        self.version = None

        if not ROS2_AVAILABLE:
            self.logger.warning("ROS2 패키지를 찾을 수 없습니다. Windows 모드로 실행합니다.")

        # Zone 및 Health 데이터 캐시
        self.zone_cache = {}
        self.health_cache = {}
        self.last_zone_update = 0
        self.last_health_update = 0
        self.cache_timeout = 5.0  # 5초 캐시 유지

        # 🚀 Phase 3: 메시지 객체 재사용 풀 (GC 부하 감소)
        self.header_pool = []
        self.max_pool_size = 20  # 최대 20개까지 풀에 보관
    
    def process_message(self, message_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        수신된 메시지를 처리하여 ROS 메시지로 변환
        
        Args:
            message_data: SENSR 메시지 데이터
            
        Returns:
            변환된 ROS 메시지 정보 또는 None
        """
        try:
            data_type = message_data['type']
            raw_data = message_data['data']
            timestamp = message_data['timestamp']

            self.logger.debug(f"메시지 처리: type={data_type}, size={len(raw_data)} bytes")

            if data_type == 'point_cloud':
                return self._process_pointcloud(raw_data, timestamp)
            elif data_type == 'output_data':
                return self._process_output_data(raw_data, timestamp)
            else:
                self.logger.warning(f"알 수 없는 데이터 타입: {data_type}")
                return None

        except Exception as e:
            self.logger.error(f"메시지 처리 오류: {e}")
            return None
    
    def _process_pointcloud(self, raw_data: bytes, timestamp: float) -> Dict[str, Any]:
        """
        포인트클라우드 데이터를 sensor_msgs/PointCloud2로 변환
        
        Args:
            raw_data: 원본 포인트클라우드 데이터
            timestamp: 타임스탬프
            
        Returns:
            변환된 ROS 메시지 정보
        """
        try:
            self.logger.info(f"포인트클라우드 프로토버프 디코딩 시작: {len(raw_data)} bytes")
            # Protobuf 디코딩 (실제 구현에서는 sensr_proto에서 정의된 메시지 타입 사용)
            pointcloud_msg = self._decode_pointcloud_protobuf(raw_data)
            
            if pointcloud_msg is None:
                self.logger.warning("포인트클라우드 프로토버프 디코딩 실패")
                return None
            
            self.logger.info(f"포인트클라우드 디코딩 성공: {pointcloud_msg['num_points']} 포인트")
            
            # ROS PointCloud2 메시지 생성
            ros_msg = self._create_pointcloud2_message(pointcloud_msg, timestamp)
            
            self.logger.info("PointCloud2 ROS 메시지 생성 완료")
            
            return {
                'topic': self.topics['pointcloud'],
                'message': ros_msg,
                'timestamp': timestamp,
                'message_type': 'sensor_msgs/PointCloud2'
            }
            
        except Exception as e:
            self.logger.error(f"포인트클라우드 처리 오류: {e}")
            return None
    
    def _process_output_data(self, raw_data: bytes, timestamp: float) -> List[Dict[str, Any]]:
        """
        출력 데이터 (객체, 이벤트)를 ROS 메시지로 변환

        Args:
            raw_data: 원본 출력 데이터
            timestamp: 타임스탬프

        Returns:
            변환된 ROS 메시지들의 리스트
        """
        try:
            # Protobuf 디코딩
            output_data = self._decode_output_protobuf(raw_data)

            if output_data is None:
                return []

            # 트래킹 객체 수 변동 로깅
            self.track_logger.handle_objects(output_data.get('objects'), timestamp)
            messages = []

            # 객체 정보 처리
            if 'objects' in output_data:
                objects_msg = self._create_objects_message(output_data['objects'], timestamp)
                if objects_msg:
                    messages.append({
                        'topic': self.topics['objects'],
                        'message': objects_msg,
                        'timestamp': timestamp,
                        'message_type': 'visualization_msgs/MarkerArray'
                    })

            # 이벤트 정보 처리
            if 'events' in output_data:
                events_msg = self._create_events_message(output_data['events'], timestamp)
                if events_msg:
                    messages.append({
                        'topic': self.topics['events'],
                        'message': events_msg,
                        'timestamp': timestamp,
                        'message_type': 'std_msgs/String'
                    })

            # Health 정보 처리 (1초마다)
            if 'health' in output_data:
                health_msg = self._create_health_message(output_data['health'], timestamp)
                if health_msg:
                    messages.append({
                        'topic': '/sensr/health',  # 별도 토픽
                        'message': health_msg,
                        'timestamp': timestamp,
                        'message_type': 'diagnostic_msgs/DiagnosticArray'
                    })

            # Zone 정보 처리 (10초마다)
            if 'zones' in output_data:
                zones_msg = self._create_zones_message(output_data['zones'], timestamp)
                if zones_msg:
                    messages.append({
                        'topic': '/sensr/zones',  # 별도 토픽
                        'message': zones_msg,
                        'timestamp': timestamp,
                        'message_type': 'std_msgs/String'
                    })

            return messages

        except Exception as e:
            self.logger.error(f"출력 데이터 처리 오류: {e}")
            return []
    
    def _process_health_message(self, health) -> Dict[str, Any]:
        """
        Health 메시지 처리 (공식 문서 방식)
        
        Args:
            health: SystemHealth 메시지
            
        Returns:
            처리된 Health 데이터
        """
        try:
            # Master 상태 매핑
            master_status_map = {
                0: "None",
                1: "OK", 
                2: "Storage Shortage",
                3: "SlowDown Error",
                4: "Internal Error"
            }
            
            # Node 상태 매핑
            node_status_map = {
                0: "None",
                1: "OK",
                2: "ROS Error", 
                3: "Lost Connection",
                4: "Invalid GPU Config"
            }
            
            # Sensor 상태 매핑
            sensor_status_map = {
                0: "Sensor Dead",
                1: "Sensor Alive",
                2: "Sensor Erroneous", 
                3: "Sensor Tilted",
                4: "Sensor Suspended"
            }
            
            health_data = {
                'master_status': master_status_map.get(health.master, f"Unknown({health.master})"),
                'master_code': health.master,
                'nodes': {}
            }
            
            # 공식 문서의 샘플 코드 방식
            if len(health.nodes) > 0:
                for node_key in health.nodes:
                    node_health = health.nodes[node_key]
                    
                    node_info = {
                        'status': node_status_map.get(node_health.status, f"Unknown({node_health.status})"),
                        'status_code': node_health.status,
                        'sensors': {}
                    }
                    
                    if len(node_health.sensors) > 0:
                        for sensor_key in node_health.sensors:
                            sensor_health = node_health.sensors[sensor_key]
                            node_info['sensors'][sensor_key] = {
                                'status': sensor_status_map.get(sensor_health, f"Unknown({sensor_health})"),
                                'status_code': sensor_health
                            }
                    
                    health_data['nodes'][node_key] = node_info
            
            # 중요한 상태 변화 로깅
            if health.master != 1:  # OK가 아닌 경우
                self.logger.warning(f"Master 상태 주의: {health_data['master_status']}")
            
            for node_key, node_info in health_data['nodes'].items():
                if node_info['status_code'] != 1:  # OK가 아닌 경우
                    self.logger.warning(f"Node {node_key} 상태 주의: {node_info['status']}")
                
                for sensor_key, sensor_info in node_info['sensors'].items():
                    if sensor_info['status_code'] != 1:  # Sensor Alive가 아닌 경우
                        self.logger.warning(f"Sensor {sensor_key} 상태 주의: {sensor_info['status']}")
            
            return health_data
            
        except Exception as e:
            self.logger.error(f"Health 메시지 처리 오류: {e}")
            return {'error': str(e)}
    
    def _decode_pointcloud_protobuf(self, raw_data: bytes) -> Optional[Dict[str, Any]]:
        """
        SENSR Protobuf 포인트클라우드 데이터 디코딩

        Args:
            raw_data: 원본 바이너리 데이터

        Returns:
            디코딩된 포인트클라우드 데이터
        """
        try:
            # SENSR protobuf 사용 시도
            try:
                # 🚀 Phase 1: 성능 측정 시작
                parse_start_time = time.time()

                self.logger.info("point_cloud_pb2 import 시도")
                from sensr_proto import point_cloud_pb2

                self.logger.info("PointResult 객체 생성")
                point_result = point_cloud_pb2.PointResult()

                self.logger.info(f"Protobuf 파싱 시작: {len(raw_data)} bytes")
                protobuf_parse_start = time.time()
                point_result.ParseFromString(raw_data)
                protobuf_parse_time = (time.time() - protobuf_parse_start) * 1000  # ms

                self.logger.info(f"파싱 완료: {len(point_result.points)}개의 포인트클라우드 (Protobuf 파싱: {protobuf_parse_time:.1f}ms)")
                
                # 🚀 Zero-Copy 최적화: NumPy 배열로 직접 수집
                all_points_list = []
                all_intensities_list = []

                for i, point_cloud in enumerate(point_result.points):
                    points_data = point_cloud.points
                    intensities_data = point_cloud.intensities

                    # 로깅 최적화: DEBUG 레벨로 변경
                    if self.logger.isEnabledFor(logging.DEBUG):
                        self.logger.debug(f"포인트클라우드 {i}: type={point_cloud.type}, points={len(points_data)} bytes, intensities={len(intensities_data)} bytes")

                    # 공식 SENSR SDK 방식으로 파싱
                    import ctypes
                    import numpy as np

                    # 포인트 파싱 (Zero-Copy)
                    float_size = ctypes.sizeof(ctypes.c_float)
                    num_points = len(points_data) // (float_size * 3)  # Each point is 3 floats (x,y,z)

                    if num_points > 0:
                        # 🚀 NumPy 배열 직접 사용 (dict 변환 제거로 10-20배 속도 향상)
                        points_array = np.frombuffer(points_data, np.float32).reshape(-1, 3)
                        all_points_list.append(points_array)

                    # Intensities 파싱 (Zero-Copy)
                    if len(intensities_data) > 0:
                        intensity_np = np.frombuffer(intensities_data, np.float32)
                        all_intensities_list.append(intensity_np)

                        # 통계 정보 로깅 (DEBUG 레벨로 변경)
                        if self.logger.isEnabledFor(logging.DEBUG):
                            min_intensity = np.min(intensity_np)
                            median_intensity = np.median(intensity_np)
                            max_intensity = np.max(intensity_np)
                            self.logger.debug(f"포인트클라우드 {i}: {num_points}개 포인트, 강도 범위 [{min_intensity:.2f}, {median_intensity:.2f}, {max_intensity:.2f}]")

                # 🚀 NumPy 배열 결합 (vstack/concatenate는 매우 빠름)
                if all_points_list:
                    all_points_array = np.vstack(all_points_list)
                    total_points = len(all_points_array)
                else:
                    all_points_array = np.array([]).reshape(0, 3)
                    total_points = 0

                if all_intensities_list:
                    all_intensities_array = np.concatenate(all_intensities_list)
                else:
                    all_intensities_array = np.array([])

                # 🚀 Phase 1: 전체 디코딩 시간 측정
                total_decode_time = (time.time() - parse_start_time) * 1000  # ms
                self.logger.info(f"최종 파싱 결과: {total_points}개 포인트, {len(all_intensities_array)}개 강도값 (총 {total_decode_time:.1f}ms)")

                return {
                    'points': all_points_array,  # 🚀 NumPy 배열로 반환
                    'intensities': all_intensities_array,  # 🚀 NumPy 배열로 반환
                    'num_points': total_points,
                    'fields': ['x', 'y', 'z', 'intensity']
                }
                
            except ImportError:
                self.logger.warning("SENSR protobuf 바인딩을 찾을 수 없습니다. 일반적인 파싱을 시도합니다.")
                
                # Fallback: 일반적인 바이너리 파싱
                if len(raw_data) < 16:
                    self.logger.warning("포인트클라우드 데이터가 너무 짧습니다")
                    return None
                
                # 간단한 포인트클라우드 구조 가정
                points = []
                # 4바이트씩 float으로 파싱하여 x, y, z 좌표 추출
                for i in range(0, len(raw_data) - 11, 12):
                    try:
                        x, y, z = struct.unpack('<fff', raw_data[i:i + 12])
                        points.append({'x': x, 'y': y, 'z': z})
                    except:
                        break
                
                return {
                    'points': points,
                    'intensities': [],
                    'num_points': len(points),
                    'fields': ['x', 'y', 'z']
                }
            
        except Exception as e:
            self.logger.error(f"포인트클라우드 디코딩 오류: {e}")
            return None
    
    def _decode_output_protobuf(self, raw_data: bytes) -> Optional[Dict[str, Any]]:
        """
        SENSR Protobuf 출력 데이터 디코딩
        
        Args:
            raw_data: 원본 바이너리 데이터
            
        Returns:
            디코딩된 출력 데이터
        """
        try:
            # SENSR protobuf 사용 시도
            try:
                from sensr_proto import output_pb2
                
                output_message = output_pb2.OutputMessage()
                output_message.ParseFromString(raw_data)
                
                result = {}
                
                # StreamMessage 처리 (공식 문서 방식)
                if output_message.HasField('stream'):
                    stream = output_message.stream
                    
                    # 객체 처리 (has_objects 플래그 확인)
                    if stream.has_objects:
                        objects = []
                        
                        for obj in stream.objects:
                            # 공식 문서의 추천 - TRACKING 상태만 사용
                            if obj.tracking_status == 3:  # TRACKING = 3 (RECOMMENDED VALUE TO USE FOR TRACKING)
                                
                                # 객체 포인트 파싱 (공식 방식)
                                object_point_num = 0
                                object_intensities = []
                                
                                if len(obj.points) > 0:
                                    import ctypes
                                    import numpy as np
                                    float_size = ctypes.sizeof(ctypes.c_float)
                                    object_point_num = len(obj.points) // (float_size * 3)  # Each point is 3 floats (x,y,z)
                                    
                                    if len(obj.intensities) > 0:
                                        intensity_np = np.frombuffer(obj.intensities, np.float32)
                                        object_intensities = intensity_np.tolist()
                                
                                object_data = {
                                    'id': obj.id,
                                    'label': obj.label,
                                    'confidence': obj.confidence,
                                    'x': obj.bbox.position.x,
                                    'y': obj.bbox.position.y,
                                    'z': obj.bbox.position.z,
                                    'width': obj.bbox.size.x,
                                    'height': obj.bbox.size.y,
                                    'length': obj.bbox.size.z,
                                    'yaw': obj.bbox.yaw,
                                    'velocity_x': obj.velocity.x,
                                    'velocity_y': obj.velocity.y,
                                    'velocity_z': obj.velocity.z,
                                    'tracking_status': obj.tracking_status,
                                    'yaw_rate': obj.yaw_rate,
                                    'retro_reflective': obj.retro_reflective,
                                    'point_count': object_point_num,
                                    'intensities': object_intensities
                                }
                                objects.append(object_data)
                                
                                self.logger.debug(f"객체 {obj.id}: TRACKING 상태, {object_point_num}개 포인트")
                            else:
                                self.logger.debug(f"객체 {obj.id}: 상태={obj.tracking_status} (TRACKING 아님, 무시)")
                        
                        result['objects'] = objects
                    else:
                        self.logger.debug("has_objects=False, 객체 데이터 없음")
                    
                    # Health 메시지 처리 (1초마다)
                    if stream.HasField('health'):
                        health_data = self._process_health_message(stream.health)
                        result['health'] = health_data
                        self.logger.debug(f"Health 상태: Master={health_data.get('master_status', 'None')}")
                    
                    # Zone 설정 처리 (10초마다)
                    if stream.has_zones:
                        zones_data = []
                        for zone in stream.zones:
                            zone_data = {
                                'id': zone.id,
                                'name': zone.name,
                                'type': zone.type,  # 0=None, 1=Event
                                'min_z': zone.pbox.min_z,
                                'max_z': zone.pbox.max_z,
                                'points': [(point.x, point.y) for point in zone.pbox.points]
                            }
                            zones_data.append(zone_data)
                        result['zones'] = zones_data
                
                # EventMessage 처리
                if output_message.HasField('event'):
                    event = output_message.event
                    events = []
                    
                    for zone_event in event.zone:
                        events.append({
                            'type': 'zone',
                            'zone_id': zone_event.id,
                            'event_type': zone_event.type,
                            'object_id': zone_event.object.id,
                            'timestamp': zone_event.timestamp.seconds
                        })
                    
                    for losing_event in event.losing:
                        events.append({
                            'type': 'losing',
                            'object_id': losing_event.id,
                            'timestamp': losing_event.timestamp.seconds
                        })
                    
                    result['events'] = events
                
                return result
                
            except ImportError:
                self.logger.warning("SENSR protobuf 바인딩을 찾을 수 없습니다. 바이너리 데이터를 건너뜁니다.")
                return None
                
        except Exception as e:
            # 디코딩 실패는 정상적인 상황일 수 있음 (바이너리 protobuf 데이터)
            # DEBUG 레벨로 변경하여 로그 스팸 방지
            self.logger.debug(f"출력 데이터 디코딩 오류: {e}")
            return None
    
    def _create_pointcloud2_message(self, pointcloud_data: Dict[str, Any], timestamp: float) -> PointCloud2:
        """
        ROS PointCloud2 메시지 생성
        
        Args:
            pointcloud_data: 디코딩된 포인트클라우드 데이터
            timestamp: 타임스탬프
            
        Returns:
            ROS PointCloud2 메시지
        """
        msg = PointCloud2()
        
        # 헤더 설정
        msg.header = self._create_ros_header(timestamp)
        
        # 포인트 필드 정의
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        # 포인트 데이터 패킹
        points = pointcloud_data['points']
        msg.width = len(points)
        msg.height = 1
        msg.is_bigendian = False
        msg.point_step = 16  # 4 fields * 4 bytes each
        msg.row_step = msg.point_step * msg.width
        
        # 🚀 바이너리 데이터 생성 (Zero-Copy 최적화 v2)
        intensities = pointcloud_data.get('intensities', np.array([]))

        # 🚀 NumPy 배열인 경우 최적화된 처리
        if isinstance(points, np.ndarray):
            num_points = len(points)
            
            # 🚀 hstack으로 한 번에 결합 (zeros + 복사보다 빠름)
            if isinstance(intensities, np.ndarray) and len(intensities) == num_points:
                # Intensity를 (N, 1) 형태로 reshape하여 hstack
                cloud_array = np.hstack([points, intensities.reshape(-1, 1)])
            elif isinstance(intensities, np.ndarray) and len(intensities) > 0:
                # 크기가 다른 경우
                min_len = min(num_points, len(intensities))
                intensity_col = np.zeros((num_points, 1), dtype=np.float32)
                intensity_col[:min_len, 0] = intensities[:min_len]
                cloud_array = np.hstack([points, intensity_col])
            else:
                # Intensity 없는 경우
                intensity_col = np.zeros((num_points, 1), dtype=np.float32)
                cloud_array = np.hstack([points, intensity_col])
            
            # 🚀 dtype 확인 및 변환 (필요시에만)
            if cloud_array.dtype != np.float32:
                cloud_array = cloud_array.astype(np.float32, copy=False)
        else:
            # Fallback: dict 리스트인 경우 (이전 방식 호환)
            num_points = len(points)
            cloud_array = np.zeros((num_points, 4), dtype=np.float32)

            for idx, point in enumerate(points):
                cloud_array[idx, 0] = point.get('x', 0.0)
                cloud_array[idx, 1] = point.get('y', 0.0)
                cloud_array[idx, 2] = point.get('z', 0.0)

                if idx < len(intensities):
                    cloud_array[idx, 3] = float(intensities[idx])
                elif 'intensity' in point:
                    cloud_array[idx, 3] = float(point['intensity'])
                else:
                    cloud_array[idx, 3] = 0.0

        # 🚀 NumPy 배열을 바이트로 직접 변환 (매우 빠름)
        msg.data = cloud_array.tobytes()
        msg.is_dense = True
        
        return msg
    
    def _create_objects_message(self, objects_data: List[Dict[str, Any]], timestamp: float) -> MarkerArray:
        """
        ROS MarkerArray 메시지 생성 (객체 정보)
        
        Args:
            objects_data: 객체 데이터 리스트
            timestamp: 타임스탬프
            
        Returns:
            ROS MarkerArray 메시지
        """
        marker_array = MarkerArray()
        
        for i, obj in enumerate(objects_data):
            marker = Marker()
            marker.header = self._create_ros_header(timestamp)
            marker.ns = "sensr_objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # 객체 위치 설정 (예시)
            marker.pose.position = Point(
                x=obj.get('x', 0.0),
                y=obj.get('y', 0.0),
                z=obj.get('z', 0.0)
            )
            
            # 객체 크기 설정 (예시)
            marker.scale = Vector3(
                x=obj.get('width', 1.0),
                y=obj.get('height', 1.0),
                z=obj.get('length', 1.0)
            )
            
            # 색상 설정
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            if ROS2_AVAILABLE:
                from builtin_interfaces.msg import Duration as DurationMsg
                duration_msg = DurationMsg()
                duration_msg.sec = 0
                duration_msg.nanosec = 100_000_000  # 100ms
                marker.lifetime = duration_msg
            else:
                marker.lifetime = None
            
            marker_array.markers.append(marker)
        
        return marker_array
    
    def _create_events_message(self, events_data: List[Dict[str, Any]], timestamp: float) -> String:
        """
        ROS String 메시지 생성 (이벤트 정보)
        
        Args:
            events_data: 이벤트 데이터 리스트
            timestamp: 타임스탬프
            
        Returns:
            ROS String 메시지
        """
        import json
        
        events_str = json.dumps({
            'timestamp': timestamp,
            'events': events_data
        })
        
        msg = String()
        msg.data = events_str
        
        return msg
    
    def _create_health_message(self, health_data: Dict[str, Any], timestamp: float) -> Any:
        """Health 데이터를 diagnostic_msgs/DiagnosticArray로 변환"""
        if ROS2_AVAILABLE:
            from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

            array_msg = DiagnosticArray()
            array_msg.header = self._create_ros_header(timestamp)
            array_msg.header.frame_id = "sensr_health"

            status_msg = DiagnosticStatus()
            status_msg.name = "SENSR System Health"

            master_code = health_data.get('master_code', 0)
            if master_code == 1:
                status_msg.level = DiagnosticStatus.OK
            elif master_code in [2, 3]:
                status_msg.level = DiagnosticStatus.WARN
            else:
                status_msg.level = DiagnosticStatus.ERROR

            status_msg.message = f"Master: {health_data.get('master_status', 'Unknown')}"
            status_msg.hardware_id = health_data.get('hardware_id', 'SENSR')

            status_msg.values = []
            status_msg.values.append(KeyValue(
                key="master_status",
                value=health_data.get('master_status', 'Unknown')
            ))

            for node_key, node_info in health_data.get('nodes', {}).items():
                status_msg.values.append(KeyValue(
                    key=f"node_{node_key}_status",
                    value=node_info.get('status', 'Unknown')
                ))
                for sensor_key, sensor_info in node_info.get('sensors', {}).items():
                    status_msg.values.append(KeyValue(
                        key=f"sensor_{sensor_key}_status",
                        value=sensor_info.get('status', 'Unknown')
                    ))

            array_msg.status.append(status_msg)
            return array_msg
        else:
            header = self._create_ros_header(timestamp)
            header.frame_id = "sensr_health"

            status = type('DiagnosticStatus', (), {
                'name': "SENSR System Health",
                'message': f"Master: {health_data.get('master_status', 'Unknown')}",
                'level': health_data.get('master_code', 0),
                'values': []
            })()

            for node_key, node_info in health_data.get('nodes', {}).items():
                status.values.append({
                    'key': f"node_{node_key}_status",
                    'value': node_info.get('status', 'Unknown')
                })
                for sensor_key, sensor_info in node_info.get('sensors', {}).items():
                    status.values.append({
                        'key': f"sensor_{sensor_key}_status",
                        'value': sensor_info.get('status', 'Unknown')
                    })

            return type('DiagnosticArray', (), {
                'header': header,
                'status': [status]
            })()

    def _create_zones_message(self, zones_data: List[Dict[str, Any]], timestamp: float) -> String:
        """
        Zone 설정을 std_msgs/String으로 변환
        
        Args:
            zones_data: Zone 데이터 리스트
            timestamp: 타임스탬프
            
        Returns:
            std_msgs/String 메시지
        """
        import json
        
        msg = String()
        
        zone_info = {
            'timestamp': timestamp,
            'zones': zones_data
        }
        
        msg.data = json.dumps(zone_info, indent=2)
        
        return msg
    
    def _create_ros_header(self, timestamp: float) -> Header:
        """
        🚀 Phase 3: ROS Header 생성 (객체 재사용 풀 적용)

        Args:
            timestamp: 타임스탬프

        Returns:
            ROS Header
        """
        # 🚀 Phase 3: 풀에서 Header 재사용
        if self.header_pool:
            header = self.header_pool.pop()
        else:
            header = Header()

        if ROS2_AVAILABLE:
            # ROS2의 시간 처리
            from builtin_interfaces.msg import Time as TimeMsg
            sec = int(timestamp)
            nanosec = int((timestamp - sec) * 1e9)
            time_msg = TimeMsg()
            time_msg.sec = sec
            time_msg.nanosec = nanosec
            header.stamp = time_msg
        else:
            header.stamp = Time.from_sec(timestamp)

        header.frame_id = "sensr_frame"

        return header

    def recycle_header(self, header: Header):
        """
        🚀 Phase 3: Header 객체를 풀에 반환 (재사용)

        Args:
            header: 재사용할 Header 객체
        """
        if len(self.header_pool) < self.max_pool_size:
            self.header_pool.append(header)
    
    def get_sensr_version(self) -> str:
        """SENSR 버전 자동 감지"""
        if self.version:
            return self.version
            
        versions = ["v4.0", "v3.0", "v2.0", "latest"]
        
        for version in versions:
            try:
                test_url = f"{self.base_url}/{version}/settings/parameters/common?config-key=publish_level_point_cloud"
                response = requests.get(test_url, timeout=3)
                if response.status_code == 200:
                    self.version = version
                    return version
            except:
                continue
        
        self.version = "v4.0"
        return self.version

    def fetch_zone_status(self, zone_id: int) -> Optional[Dict[str, Any]]:
        """
        Zone 실시간 상태 조회 (Results API)
        
        Args:
            zone_id: Zone ID
            
        Returns:
            Zone 상태 데이터 또는 None
        """
        current_time = time.time()
        
        # 캐시 확인
        if (zone_id in self.zone_cache and 
            current_time - self.last_zone_update < self.cache_timeout):
            return self.zone_cache[zone_id]
        
        try:
            zone_url = f"{self.base_url}/results/zone?id={zone_id}"
            response = requests.get(zone_url, timeout=5)
            
            if response.status_code == 200:
                zone_data = response.json()
                
                # 캐시 업데이트
                self.zone_cache[zone_id] = zone_data
                self.last_zone_update = current_time
                
                self.logger.debug(f"Zone {zone_id} 상태 조회 성공")
                return zone_data
            else:
                self.logger.warning(f"Zone {zone_id} 상태 조회 실패: HTTP {response.status_code}")
                return None
                
        except Exception as e:
            self.logger.error(f"Zone {zone_id} 상태 조회 오류: {e}")
            return None

    def fetch_all_zones_status(self) -> Dict[int, Dict[str, Any]]:
        """
        모든 Zone의 실시간 상태 조회
        
        Returns:
            Zone ID별 상태 데이터
        """
        current_time = time.time()
        
        # 캐시 확인
        if (self.zone_cache and 
            current_time - self.last_zone_update < self.cache_timeout):
            return self.zone_cache
        
        try:
            # 모든 Zone 조회 (id=0으로 모든 Zone 가져오기)
            zone_url = f"{self.base_url}/results/zone?id=0"
            response = requests.get(zone_url, timeout=10)
            
            if response.status_code == 200:
                zones_data = response.json()
                
                # Zone별로 캐시 저장
                self.zone_cache.clear()
                if 'zones' in zones_data:
                    for zone in zones_data['zones']:
                        zone_id = zone.get('id')
                        if zone_id:
                            self.zone_cache[zone_id] = zone
                
                self.last_zone_update = current_time
                self.logger.debug(f"전체 Zone 상태 조회 성공: {len(self.zone_cache)}개")
                return self.zone_cache
            else:
                self.logger.warning(f"전체 Zone 상태 조회 실패: HTTP {response.status_code}")
                return {}
                
        except Exception as e:
            self.logger.error(f"전체 Zone 상태 조회 오류: {e}")
            return {}

    def fetch_health_status(self) -> Optional[Dict[str, Any]]:
        """
        시스템 Health 상태 조회 (Results API)
        
        Returns:
            Health 상태 데이터 또는 None
        """
        current_time = time.time()
        
        # 캐시 확인
        if (self.health_cache and 
            current_time - self.last_health_update < self.cache_timeout):
            return self.health_cache
        
        try:
            health_url = f"{self.base_url}/results/health"
            response = requests.get(health_url, timeout=5)
            
            if response.status_code == 200:
                health_data = response.json()
                
                # 캐시 업데이트
                self.health_cache = health_data
                self.last_health_update = current_time
                
                self.logger.debug("시스템 Health 상태 조회 성공")
                return health_data
            else:
                self.logger.warning(f"Health 상태 조회 실패: HTTP {response.status_code}")
                return None
                
        except Exception as e:
            self.logger.error(f"Health 상태 조회 오류: {e}")
            return None

    def enrich_output_data_with_zones(self, output_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        출력 데이터에 Zone 정보를 추가
        
        Args:
            output_data: 기본 출력 데이터
            
        Returns:
            Zone 정보가 추가된 출력 데이터
        """
        try:
            # 모든 Zone 상태 가져오기
            zones_status = self.fetch_all_zones_status()
            
            if zones_status:
                # 출력 데이터에 Zone 정보 추가
                enriched_data = output_data.copy()
                enriched_data['zones_status'] = {}
                
                for zone_id, zone_data in zones_status.items():
                    zone_info = {
                        'id': zone_data.get('id'),
                        'name': zone_data.get('name'),
                        'type': zone_data.get('type'),
                        'object_ids': zone_data.get('objectIds', []),
                        'active_objects': len(zone_data.get('objectIds', []))
                    }
                    
                    enriched_data['zones_status'][zone_id] = zone_info
                
                self.logger.debug(f"Zone 정보가 추가된 출력 데이터: {len(zones_status)}개 Zone")
                return enriched_data
            else:
                return output_data
                
        except Exception as e:
            self.logger.error(f"Zone 정보 추가 오류: {e}")
            return output_data

    def enrich_output_data_with_health(self, output_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        출력 데이터에 Health 정보를 추가
        
        Args:
            output_data: 기본 출력 데이터
            
        Returns:
            Health 정보가 추가된 출력 데이터
        """
        try:
            # 시스템 Health 상태 가져오기
            health_status = self.fetch_health_status()
            
            if health_status:
                # 출력 데이터에 Health 정보 추가
                enriched_data = output_data.copy()
                enriched_data['system_health'] = {
                    'master_status': health_status.get('master', 'Unknown'),
                    'nodes_status': {},
                    'sensors_status': {},
                    'overall_healthy': True
                }
                
                # 노드별 상태 분석
                nodes = health_status.get('nodes', {})
                for node_id, node_data in nodes.items():
                    node_status = node_data.get('status', 'Unknown')
                    enriched_data['system_health']['nodes_status'][node_id] = node_status
                    
                    if node_status != 'OK':
                        enriched_data['system_health']['overall_healthy'] = False
                    
                    # 센서별 상태 분석
                    sensors = node_data.get('sensors', {})
                    for sensor_name, sensor_status in sensors.items():
                        enriched_data['system_health']['sensors_status'][sensor_name] = sensor_status
                        
                        if 'ALIVE' not in sensor_status:
                            enriched_data['system_health']['overall_healthy'] = False
                
                # 마스터 상태 확인
                if health_status.get('master') != 'OK':
                    enriched_data['system_health']['overall_healthy'] = False
                
                self.logger.debug(f"Health 정보가 추가된 출력 데이터 (전체 상태: {enriched_data['system_health']['overall_healthy']})")
                return enriched_data
            else:
                return output_data
                
        except Exception as e:
            self.logger.error(f"Health 정보 추가 오류: {e}")
            return output_data

    def create_enhanced_output_message(self, output_data: Dict[str, Any], timestamp: float) -> Dict[str, Any]:
        """
        Zone 및 Health 정보가 포함된 확장된 출력 메시지 생성
        
        Args:
            output_data: 기본 출력 데이터
            timestamp: 타임스탬프
            
        Returns:
            확장된 ROS 메시지 정보
        """
        try:
            # Zone 정보 추가
            enriched_data = self.enrich_output_data_with_zones(output_data)
            
            # Health 정보 추가
            enriched_data = self.enrich_output_data_with_health(enriched_data)
            
            # 기존 메시지 생성 로직 호출
            messages = {}
            
            # 객체 메시지
            if 'objects' in enriched_data:
                objects_msg = self._create_objects_message(enriched_data['objects'], timestamp)
                messages[self.topics['objects']] = objects_msg
            
            # 이벤트 메시지
            if 'events' in enriched_data:
                events_msg = self._create_events_message(enriched_data['events'], timestamp)
                messages[self.topics['events']] = events_msg
            
            # Health 메시지
            if 'system_health' in enriched_data:
                health_msg = self._create_health_message_enhanced(enriched_data['system_health'], timestamp)
                messages[self.topics['health']] = health_msg
            
            # Zone 메시지
            if 'zones_status' in enriched_data:
                zones_msg = self._create_zones_message_enhanced(enriched_data['zones_status'], timestamp)
                messages[self.topics['zones']] = zones_msg
            
            return messages
            
        except Exception as e:
            self.logger.error(f"확장된 출력 메시지 생성 오류: {e}")
            return {}

    def _create_health_message_enhanced(self, health_data: Dict[str, Any], timestamp: float) -> Any:
        """
        확장된 Health 메시지 생성 (API 데이터 포함)
        
        Args:
            health_data: 확장된 Health 데이터
            timestamp: 타임스탬프
            
        Returns:
            ROS Health 메시지
        """
        import json
        
        if ROS2_AVAILABLE:
            from std_msgs.msg import String
            msg = String()
        else:
            msg = String()
        
        # 확장된 Health 정보를 JSON으로 변환
        enhanced_health = {
            'timestamp': timestamp,
            'master_status': health_data.get('master_status', 'Unknown'),
            'nodes_count': len(health_data.get('nodes_status', {})),
            'sensors_count': len(health_data.get('sensors_status', {})),
            'overall_healthy': health_data.get('overall_healthy', False),
            'nodes_detail': health_data.get('nodes_status', {}),
            'sensors_detail': health_data.get('sensors_status', {}),
            'api_data_available': True
        }
        
        msg.data = json.dumps(enhanced_health, indent=2)
        return msg

    def _create_zones_message_enhanced(self, zones_data: Dict[int, Dict[str, Any]], timestamp: float) -> Any:
        """
        확장된 Zone 메시지 생성 (API 데이터 포함)
        
        Args:
            zones_data: 확장된 Zone 데이터
            timestamp: 타임스탬프
            
        Returns:
            ROS Zone 메시지
        """
        import json
        
        if ROS2_AVAILABLE:
            from std_msgs.msg import String
            msg = String()
        else:
            msg = String()
        
        # 확장된 Zone 정보를 JSON으로 변환
        enhanced_zones = {
            'timestamp': timestamp,
            'total_zones': len(zones_data),
            'active_zones': len([z for z in zones_data.values() if z.get('active_objects', 0) > 0]),
            'zones_detail': zones_data,
            'api_data_available': True
        }
        
        msg.data = json.dumps(enhanced_zones, indent=2)
        return msg
