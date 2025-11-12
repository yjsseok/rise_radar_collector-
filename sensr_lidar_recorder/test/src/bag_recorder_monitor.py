#!/usr/bin/env python3
"""
ROS2 Bag Recorder Module with Monitoring
모니터링 기능이 추가된 bag recorder
"""

import os
import sys
import time
import threading
import logging
from datetime import datetime
from typing import Dict, Any, Optional, List
import queue

# 원본 소스 경로 추가
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

try:
    import rclpy
    from rclpy.serialization import serialize_message
    from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
    from std_msgs.msg import Header
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    logging.warning("ROS2 패키지를 찾을 수 없습니다. ROS2 환경에서 실행해주세요.")


class BagRecorderMonitor:
    """모니터링 기능이 추가된 ROS2 bag 파일 레코더"""

    def __init__(self, config: Dict[str, Any]):
        """
        BagRecorderMonitor 초기화

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
        self.created_topics = set()

        # 🔍 모니터링 통계
        self.stats = {
            'total_received': 0,        # 총 수신한 메시지
            'total_written': 0,         # 디스크에 쓴 메시지
            'total_dropped': 0,         # 드롭된 메시지
            'queue_sizes': [],          # 큐 크기 기록
            'write_times': [],          # 디스크 쓰기 시간
            'start_time': None,
            'last_stats_time': 0,
        }
        self.stats_lock = threading.Lock()

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
        """레코딩 시작"""
        if self.is_recording:
            self.logger.warning("이미 레코딩이 진행 중입니다.")
            return False

        try:
            self.is_recording = True
            self.stats['start_time'] = time.time()

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

            self.logger.info("🎬 ROS2 bag 레코딩 시작 (모니터링 모드)")
            return True

        except Exception as e:
            self.logger.error(f"레코딩 시작 실패: {e}")
            self.is_recording = False
            return False

    def stop_recording(self):
        """레코딩 중지"""
        if not self.is_recording:
            return

        self.is_recording = False

        # 현재 bag 파일 닫기
        self._close_current_bag()

        # 스레드 종료 대기
        if self.recording_thread and self.recording_thread.is_alive():
            self.recording_thread.join(timeout=5)

        if self.timer_thread and self.timer_thread.is_alive():
            self.timer_thread.join(timeout=5)

        self.logger.info("🛑 ROS2 bag 레코딩 중지")

    def write_message(self, topic: str, message: Any, timestamp: Optional[float] = None):
        """
        메시지를 bag 파일에 쓰기 (모니터링 포함)

        Args:
            topic: ROS2 토픽 이름
            message: ROS2 메시지 객체
            timestamp: 타임스탬프
        """
        if not self.is_recording:
            return

        if timestamp is None:
            timestamp = time.time()

        # 🔍 수신 카운터 증가
        with self.stats_lock:
            self.stats['total_received'] += 1

        message_data = {
            'topic': topic,
            'message': message,
            'timestamp': timestamp
        }

        try:
            self.message_queue.put_nowait(message_data)
        except queue.Full:
            # 🔍 드롭 카운터 증가
            with self.stats_lock:
                self.stats['total_dropped'] += 1
            self.logger.warning("⚠️ 메시지 큐 가득참! 메시지 드롭")

    def _create_new_bag(self):
        """새로운 bag 파일 생성"""
        with self.lock:
            if self.current_writer is not None:
                self._close_current_bag()

            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = self.filename_format.format(timestamp=timestamp_str)
            if filename.endswith('.bag'):
                filename = filename[:-4]

            self.current_bag_path = os.path.join(self.output_directory, filename)

            try:
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
                self.created_topics.clear()

                self.logger.info(f"📁 새 bag 파일: {self.current_bag_path}")

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
                    self.logger.info(f"💾 Bag 파일 저장: {self.current_bag_path}")
                except Exception as e:
                    self.logger.error(f"Bag 파일 닫기 오류: {e}")
                finally:
                    self.current_writer = None
                    self.current_bag_path = None
                    self.bag_start_time = None

    def _recording_worker(self):
        """메시지 처리 작업 스레드 (모니터링 포함)"""
        self.logger.info("🔧 메시지 처리 스레드 시작")

        while self.is_recording:
            try:
                # 🔍 큐 크기 기록
                current_queue_size = self.message_queue.qsize()
                with self.stats_lock:
                    self.stats['queue_sizes'].append(current_queue_size)

                # 메시지 가져오기
                message_data = self.message_queue.get(timeout=1.0)

                with self.lock:
                    if self.current_writer is not None:
                        try:
                            # 🔍 쓰기 시작 시간
                            write_start = time.time()

                            ros_time_ns = int(message_data['timestamp'] * 1e9)
                            message_type = message_data['message'].__class__
                            module_parts = message_type.__module__.split('.')
                            if len(module_parts) >= 2 and module_parts[1] == 'msg':
                                type_name = f"{module_parts[0]}/msg/{message_type.__name__}"
                            else:
                                type_name = f"{message_type.__module__}/{message_type.__name__}"

                            topic_name = message_data['topic']

                            # 토픽 생성
                            if topic_name not in self.created_topics:
                                try:
                                    topic_metadata = TopicMetadata(
                                        name=topic_name,
                                        type=type_name,
                                        serialization_format='cdr'
                                    )
                                    self.current_writer.create_topic(topic_metadata)
                                    self.created_topics.add(topic_name)
                                except Exception as e:
                                    self.logger.error(f"토픽 생성 실패 {topic_name}: {e}")
                                    continue

                            # 메시지 쓰기
                            try:
                                serialized_msg = serialize_message(message_data['message'])
                                self.current_writer.write(
                                    topic_name,
                                    serialized_msg,
                                    ros_time_ns
                                )

                                # 🔍 쓰기 완료 시간 기록
                                write_time = time.time() - write_start
                                with self.stats_lock:
                                    self.stats['total_written'] += 1
                                    self.stats['write_times'].append(write_time)

                            except Exception as write_error:
                                self.logger.error(f"메시지 쓰기 오류: {write_error}")
                                self.created_topics.discard(topic_name)

                        except Exception as e:
                            self.logger.error(f"메시지 처리 오류: {e}")

                self.message_queue.task_done()

            except queue.Empty:
                continue
            except Exception as e:
                self.logger.error(f"워커 오류: {e}")

        self.logger.info("🔧 메시지 처리 스레드 종료")

    def _timer_worker(self):
        """타이머 작업 스레드"""
        while self.is_recording:
            try:
                time.sleep(self.duration)
                if self.is_recording:
                    self._create_new_bag()
            except Exception as e:
                self.logger.error(f"타이머 스레드 오류: {e}")

    def get_status(self) -> Dict[str, Any]:
        """레코더 상태 정보 반환"""
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

    def get_stats(self) -> Dict[str, Any]:
        """🔍 모니터링 통계 반환"""
        with self.stats_lock:
            runtime = time.time() - self.stats['start_time'] if self.stats['start_time'] else 0

            avg_queue_size = sum(self.stats['queue_sizes']) / len(self.stats['queue_sizes']) if self.stats['queue_sizes'] else 0
            max_queue_size = max(self.stats['queue_sizes']) if self.stats['queue_sizes'] else 0

            avg_write_time = sum(self.stats['write_times']) / len(self.stats['write_times']) if self.stats['write_times'] else 0
            max_write_time = max(self.stats['write_times']) if self.stats['write_times'] else 0

            receive_rate = self.stats['total_received'] / runtime if runtime > 0 else 0
            write_rate = self.stats['total_written'] / runtime if runtime > 0 else 0

            return {
                'runtime': runtime,
                'total_received': self.stats['total_received'],
                'total_written': self.stats['total_written'],
                'total_dropped': self.stats['total_dropped'],
                'current_queue_size': self.message_queue.qsize(),
                'avg_queue_size': avg_queue_size,
                'max_queue_size': max_queue_size,
                'avg_write_time_ms': avg_write_time * 1000,
                'max_write_time_ms': max_write_time * 1000,
                'receive_rate': receive_rate,
                'write_rate': write_rate,
            }
