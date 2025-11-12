#!/usr/bin/env python3
"""
ROS2 Bag Recorder Module
10초 간격으로 ROS2 bag 파일을 생성하고 관리하는 모듈
"""

import os
import time
import threading
import logging
from datetime import datetime
from typing import Dict, Any, Optional, List
import queue

try:
    import rclpy
    from rclpy.serialization import serialize_message
    from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
    from std_msgs.msg import Header
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    logging.warning("ROS2 패키지를 찾을 수 없습니다. ROS2 환경에서 실행해주세요.")


class BagRecorder:
    """ROS2 bag 파일 레코딩을 관리하는 클래스"""
    
    def __init__(self, config: Dict[str, Any]):
        """
        BagRecorder 초기화
        
        Args:
            config: 설정 딕셔너리
        """
        self.config = config
        self.recording_config = config['recording']
        self.duration = self.recording_config['duration']
        self.output_directory = self.recording_config['output_directory']
        self.filename_format = self.recording_config['filename_format']
        
        # 출력 디렉토리 생성
        if not os.path.exists(self.output_directory):
            os.makedirs(self.output_directory)
        
        # 상태 관리
        self.is_recording = False
        self.current_writer = None
        self.current_bag_path = None
        self.bag_start_time = None
        self.message_queue = queue.Queue()
        self.created_topics = set()  # 생성된 토픽 추적
        
        # 스레드 관리
        self.recording_thread = None
        self.timer_thread = None
        self.lock = threading.RLock()
        
        # 로깅
        self.logger = logging.getLogger(__name__)
        
        if not ROS2_AVAILABLE:
            raise ImportError("ROS2 패키지가 필요합니다. ROS2 환경에서 실행해주세요.")
        
        # ROS2 초기화
        if not rclpy.ok():
            rclpy.init()
    
    def start_recording(self) -> bool:
        """
        레코딩 시작
        
        Returns:
            bool: 시작 성공 여부
        """
        if self.is_recording:
            self.logger.warning("이미 레코딩이 진행 중입니다.")
            return False
        
        try:
            self.is_recording = True
            
            # 첫 번째 bag 파일 생성
            self._create_new_bag()
            
            # 메시지 처리 스레드 시작
            self.recording_thread = threading.Thread(
                target=self._recording_worker,
                daemon=True
            )
            self.recording_thread.start()
            
            # 타이머 스레드 시작
            self.timer_thread = threading.Thread(
                target=self._timer_worker,
                daemon=True
            )
            self.timer_thread.start()
            
            self.logger.info("ROS2 bag 레코딩이 시작되었습니다.")
            return True
            
        except Exception as e:
            self.logger.error(f"레코딩 시작 실패: {e}")
            self.is_recording = False
            return False
    
    def stop_recording(self):
        """레코딩 중지"""
        if not self.is_recording:
            self.logger.warning("레코딩이 진행되지 않고 있습니다.")
            return
        
        self.is_recording = False
        
        # 현재 bag 파일 닫기
        self._close_current_bag()
        
        # 스레드 종료 대기
        if self.recording_thread and self.recording_thread.is_alive():
            self.recording_thread.join(timeout=5)
        
        if self.timer_thread and self.timer_thread.is_alive():
            self.timer_thread.join(timeout=5)
        
        self.logger.info("ROS2 bag 레코딩이 중지되었습니다.")
    
    def write_message(self, topic: str, message: Any, timestamp: Optional[float] = None):
        """
        메시지를 bag 파일에 쓰기
        
        Args:
            topic: ROS2 토픽 이름
            message: ROS2 메시지 객체
            timestamp: 타임스탬프 (None이면 현재 시간 사용)
        """
        if not self.is_recording:
            self.logger.warning(f"레코딩 중이 아님. 메시지 건너뜀: {topic}")
            return
        
        if timestamp is None:
            timestamp = time.time()
        
        self.logger.info(f"메시지 수신: topic={topic}, type={type(message).__name__}, timestamp={timestamp}")
        
        message_data = {
            'topic': topic,
            'message': message,
            'timestamp': timestamp
        }
        
        try:
            self.message_queue.put_nowait(message_data)
            self.logger.debug(f"메시지 큐에 추가됨: {topic}")
        except queue.Full:
            self.logger.warning("메시지 큐가 가득함. 메시지를 건너뜁니다.")
    
    def _create_new_bag(self):
        """새로운 bag 파일 생성"""
        with self.lock:
            # 기존 bag 파일 닫기
            if self.current_writer is not None:
                self._close_current_bag()
            
            # 새 파일명 생성
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = self.filename_format.format(timestamp=timestamp_str)
            # ROS2 bag은 폴더로 생성되므로 .bag 확장자 제거
            if filename.endswith('.bag'):
                filename = filename[:-4]
            
            self.current_bag_path = os.path.join(self.output_directory, filename)
            
            try:
                # 새 bag 파일 생성
                storage_options = StorageOptions(
                    uri=self.current_bag_path,
                    storage_id='sqlite3'
                )
                converter_options = ConverterOptions(
                    input_serialization_format='cdr',
                    output_serialization_format='cdr'
                )
                
                self.current_writer = SequentialWriter()
                self.current_writer.open(storage_options, converter_options)
                
                self.bag_start_time = time.time()
                self.created_topics.clear()  # 새 bag 파일이므로 토픽 목록 리셋
                
                self.logger.info(f"새 bag 파일 생성: {self.current_bag_path}")
                
            except Exception as e:
                self.logger.error(f"Bag 파일 생성 실패: {e}")
                self.current_writer = None
                self.current_bag_path = None
    
    def _close_current_bag(self):
        """현재 bag 파일 닫기"""
        with self.lock:
            if self.current_writer is not None:
                try:
                    self.current_writer.close()
                    self.logger.info(f"Bag 파일 저장 완료: {self.current_bag_path}")
                except Exception as e:
                    self.logger.error(f"Bag 파일 닫기 오류: {e}")
                finally:
                    self.current_writer = None
                    self.current_bag_path = None
                    self.bag_start_time = None
    
    def _recording_worker(self):
        """메시지 처리 작업 스레드"""
        self.logger.info("메시지 처리 스레드가 시작되었습니다.")
        
        while self.is_recording:
            try:
                # 메시지 큐에서 가져오기
                message_data = self.message_queue.get(timeout=1.0)
                
                with self.lock:
                    if self.current_writer is not None:
                        try:
                            # ROS2 시간으로 변환 (nanoseconds)
                            ros_time_ns = int(message_data['timestamp'] * 1e9)

                            # 토픽 정보 생성
                            message_type = message_data['message'].__class__
                            module_parts = message_type.__module__.split('.')
                            if len(module_parts) >= 2 and module_parts[1] == 'msg':
                                type_name = f"{module_parts[0]}/msg/{message_type.__name__}"
                            else:
                                type_name = f"{message_type.__module__}/{message_type.__name__}"

                            topic_name = message_data['topic']

                            # 토픽 생성 (아직 생성되지 않은 경우만)
                            if topic_name not in self.created_topics:
                                try:
                                    topic_metadata = TopicMetadata(
                                        name=topic_name,
                                        type=type_name,
                                        serialization_format='cdr'
                                    )

                                    self.current_writer.create_topic(topic_metadata)
                                    self.created_topics.add(topic_name)
                                    self.logger.info(f"새 토픽 생성됨: {topic_name} ({type_name})")
                                except Exception as e:
                                    self.logger.error(f"토픽 생성 실패 {topic_name}: {e}")
                                    continue  # 토픽 생성 실패시 메시지 건너뛰기

                            # 메시지 직렬화 및 쓰기
                            try:
                                serialized_msg = serialize_message(message_data['message'])
                                self.current_writer.write(
                                    topic_name,
                                    serialized_msg,
                                    ros_time_ns
                                )
                                self.logger.debug(f"메시지 저장됨: {topic_name}")
                            except Exception as write_error:
                                self.logger.error(f"메시지 쓰기 오류: {write_error}")
                                # 토픽이 제대로 생성되지 않았을 수 있으므로 다시 시도하도록 제거
                                self.created_topics.discard(topic_name)
                            
                        except Exception as e:
                            self.logger.error(f"메시지 쓰기 오류: {e}")
                
                # 큐에서 작업 완료 표시
                self.message_queue.task_done()
                
            except queue.Empty:
                # 타임아웃은 정상적인 상황
                continue
            except Exception as e:
                self.logger.error(f"메시지 처리 오류: {e}")
        
        self.logger.info("메시지 처리 스레드가 종료되었습니다.")
    
    def _timer_worker(self):
        """타이머 작업 스레드 (10초마다 새 파일 생성)"""
        self.logger.info("타이머 스레드가 시작되었습니다.")
        
        while self.is_recording:
            try:
                # duration 초 대기
                time.sleep(self.duration)
                
                if self.is_recording:
                    # 새 bag 파일 생성
                    self._create_new_bag()
                    
            except Exception as e:
                self.logger.error(f"타이머 스레드 오류: {e}")
        
        self.logger.info("타이머 스레드가 종료되었습니다.")
    
    def get_status(self) -> Dict[str, Any]:
        """
        레코더 상태 정보 반환
        
        Returns:
            상태 딕셔너리
        """
        with self.lock:
            current_duration = 0
            if self.bag_start_time is not None:
                current_duration = time.time() - self.bag_start_time
            
            return {
                'is_recording': self.is_recording,
                'current_bag_path': self.current_bag_path,
                'bag_start_time': self.bag_start_time,
                'current_duration': current_duration,
                'queue_size': self.message_queue.qsize(),
                'output_directory': self.output_directory
            }
    
    def get_recorded_files(self) -> List[str]:
        """
        레코딩된 파일 목록 반환
        
        Returns:
            bag 파일 경로 리스트
        """
        try:
            files = []
            for item in os.listdir(self.output_directory):
                item_path = os.path.join(self.output_directory, item)
                # ROS2 bag은 폴더 형태
                if os.path.isdir(item_path) and not item.startswith('.'):
                    # metadata.yaml 파일이 있는지 확인하여 bag 폴더인지 검증
                    metadata_path = os.path.join(item_path, 'metadata.yaml')
                    if os.path.exists(metadata_path):
                        files.append(item_path)
            
            # 생성 시간순으로 정렬
            files.sort(key=os.path.getctime, reverse=True)
            return files
            
        except Exception as e:
            self.logger.error(f"파일 목록 조회 오류: {e}")
            return []
    
    def cleanup_old_files(self, max_files: int = 100):
        """
        오래된 bag 파일 정리
        
        Args:
            max_files: 보관할 최대 파일 수
        """
        try:
            files = self.get_recorded_files()
            
            if len(files) > max_files:
                files_to_delete = files[max_files:]
                
                for file_path in files_to_delete:
                    try:
                        # ROS2 bag은 폴더이므로 shutil.rmtree 사용
                        import shutil
                        shutil.rmtree(file_path)
                        self.logger.info(f"오래된 파일 삭제: {file_path}")
                    except Exception as e:
                        self.logger.error(f"파일 삭제 오류 {file_path}: {e}")
                        
        except Exception as e:
            self.logger.error(f"파일 정리 오류: {e}")