#!/usr/bin/env python3
"""
SENSR Monitor with Optimized Bag Recorder
메모리 최적화가 적용된 버전
"""

import os
import sys
import time
import gc
import argparse
import logging
import copy
import psutil
from typing import Optional, Dict, Any

parent_dir = os.path.join(os.path.dirname(__file__), '..')
sys.path.insert(0, parent_dir)

test_src_dir = os.path.join(os.path.dirname(__file__), 'src')
sys.path.insert(0, test_src_dir)

from src.sensr_client import SensrClient
from src.data_processor import DataProcessor
from src.utils import (
    load_config, setup_logging, validate_config,
    setup_signal_handlers, find_host_entry, set_active_host
)

# 🔧 최적화된 bag recorder
from bag_recorder_optimized import BagRecorderOptimized


class SensrOptimizedApp:
    """메모리 최적화가 적용된 앱"""

    def __init__(self, config_path: str, runtime_config: Optional[Dict[str, Any]] = None, test_duration: int = 300):
        self.config_path = config_path
        self._runtime_config = runtime_config
        self.config = None
        self.logger = None
        self.test_duration = test_duration

        self.sensr_client = None
        self.data_processor = None
        self.bag_recorder = None

        self.is_running = False
        self.start_time = None

        self.monitor_stats = {
            'messages_received': 0,
            'pointcloud_count': 0,
            'output_data_count': 0,
            'memory_samples': [],
        }

        self.process = psutil.Process()

        self.last_pointcloud_time = 0.0
        self.last_output_data_time = 0.0
        self.pointcloud_interval = 1.0
        self.output_data_interval = 1.0
        self.pointcloud_only_mode = False
        self.skip_empty_data = True

    def initialize_components(self) -> bool:
        try:
            if self._runtime_config is not None:
                self.config = self._runtime_config
            else:
                self.config = load_config(self.config_path)

            if not validate_config(self.config):
                return False

            self.logger = setup_logging(self.config)
            self.logger.info("🚀 SENSR Optimized App 시작")

            recording_config = self.config.get('recording', {})
            self.pointcloud_interval = recording_config.get('pointcloud_interval', 0.5)
            self.output_data_interval = recording_config.get('output_data_interval', 0.5)
            self.pointcloud_only_mode = recording_config.get('pointcloud_only', False)
            self.skip_empty_data = recording_config.get('skip_empty_data', True)

            self.data_processor = DataProcessor(self.config)

            # 🔧 최적화된 Bag 레코더
            self.logger.info("🔧 최적화된 Bag 레코더 초기화")
            self.bag_recorder = BagRecorderOptimized(self.config)

            self.sensr_client = SensrClient(
                self.config,
                message_callback=self._on_message_received
            )

            self.logger.info("✅ 초기화 완료")
            return True

        except Exception as e:
            if self.logger:
                self.logger.error(f"❌ 초기화 실패: {e}")
            return False

    def start(self) -> bool:
        if self.is_running:
            return False

        try:
            self.is_running = True
            self.start_time = time.time()

            if not self.sensr_client.connect():
                return False

            self.sensr_client.start_listening()
            time.sleep(2)

            return True

        except Exception as e:
            self.logger.error(f"❌ 시작 실패: {e}")
            self.is_running = False
            return False

    def stop(self):
        if not self.is_running:
            return

        self.is_running = False

        try:
            if self.bag_recorder:
                self.bag_recorder.stop_recording()
            if self.sensr_client:
                self.sensr_client.stop_listening()
        except Exception as e:
            self.logger.error(f"종료 중 오류: {e}")

    def run(self):
        if not self.initialize_components():
            return False

        setup_signal_handlers(self.stop)

        if not self.start():
            return False

        try:
            self.logger.info(f"⏱️  {self.test_duration}초 동안 최적화 테스트")
            self.logger.info("=" * 70)

            status_interval = 30
            last_status_time = time.time()

            while self.is_running:
                time.sleep(0.5)

                current_time = time.time()
                elapsed = current_time - self.start_time

                # 메모리 샘플링
                mem_info = self.process.memory_info()
                mem_mb = mem_info.rss / 1024 / 1024
                self.monitor_stats['memory_samples'].append(mem_mb)

                # 주기적 상태 출력
                if current_time - last_status_time >= status_interval:
                    self._print_status()
                    last_status_time = current_time

                if elapsed >= self.test_duration:
                    self.logger.info(f"⏰ {self.test_duration}초 완료")
                    break

        except KeyboardInterrupt:
            self.logger.info("⌨️  사용자 중단")
        except Exception as e:
            self.logger.error(f"❌ 실행 오류: {e}")
        finally:
            self.stop()
            self._print_final_report()

        return True

    def _on_message_received(self, message_data: Dict[str, Any]):
        try:
            current_time = time.time()
            message_type = message_data.get('type')

            self.monitor_stats['messages_received'] += 1

            should_process = False

            if message_type == 'point_cloud':
                if current_time - self.last_pointcloud_time >= self.pointcloud_interval:
                    should_process = True
                    self.last_pointcloud_time = current_time
                    self.monitor_stats['pointcloud_count'] += 1

            elif message_type == 'output_data':
                if self.pointcloud_only_mode:
                    return

                if current_time - self.last_output_data_time >= self.output_data_interval:
                    should_process = True
                    self.last_output_data_time = current_time
                    self.monitor_stats['output_data_count'] += 1
            else:
                should_process = True

            if not should_process:
                return

            if self.skip_empty_data:
                data_size = len(message_data.get('data', b''))
                if data_size < 100:
                    return

            if not self.bag_recorder.is_recording:
                if not self.bag_recorder.start_recording():
                    return

            processed_messages = self.data_processor.process_message(message_data)

            if processed_messages:
                if not isinstance(processed_messages, list):
                    processed_messages = [processed_messages]

                for msg_info in processed_messages:
                    if msg_info:
                        self.bag_recorder.write_message(
                            msg_info['topic'],
                            msg_info['message'],
                            msg_info['timestamp']
                        )

        except Exception as e:
            self.logger.error(f"❌ 메시지 처리 오류: {e}")

    def _print_status(self):
        try:
            elapsed = time.time() - self.start_time
            bag_stats = self.bag_recorder.get_stats()
            mem_info = self.process.memory_info()
            mem_mb = mem_info.rss / 1024 / 1024

            self.logger.info("=" * 70)
            self.logger.info(f"⏱️  {elapsed:.1f}초 | 💻 메모리: {mem_mb:.1f} MB")
            self.logger.info(f"📨 수신: {self.monitor_stats['messages_received']} | "
                           f"💾 쓰기: {bag_stats['total_written']} | "
                           f"🗑️ GC: {bag_stats['gc_count']}회")
            self.logger.info(f"📊 큐: {bag_stats['current_queue_size']} | "
                           f"드롭: {bag_stats['total_dropped']}")
            self.logger.info("=" * 70)

        except Exception as e:
            self.logger.error(f"상태 출력 오류: {e}")

    def _print_final_report(self):
        try:
            self.logger.info("\n")
            self.logger.info("=" * 70)
            self.logger.info("📊 최종 테스트 리포트 (메모리 최적화)")
            self.logger.info("=" * 70)

            elapsed = time.time() - self.start_time
            bag_stats = self.bag_recorder.get_stats()

            avg_mem = sum(self.monitor_stats['memory_samples']) / len(self.monitor_stats['memory_samples'])
            max_mem = max(self.monitor_stats['memory_samples'])
            min_mem = min(self.monitor_stats['memory_samples'])

            self.logger.info(f"\n⏱️  총 실행 시간: {elapsed:.2f}초")

            self.logger.info(f"\n📨 메시지 수신:")
            self.logger.info(f"  - 총 수신: {self.monitor_stats['messages_received']}")
            self.logger.info(f"  - 포인트클라우드: {self.monitor_stats['pointcloud_count']}")
            self.logger.info(f"  - Output Data: {self.monitor_stats['output_data_count']}")

            self.logger.info(f"\n💾 디스크 쓰기:")
            self.logger.info(f"  - 총 쓰기: {bag_stats['total_written']}")
            self.logger.info(f"  - 드롭된 메시지: {bag_stats['total_dropped']}")

            self.logger.info(f"\n💻 시스템 리소스:")
            self.logger.info(f"  - 메모리 (평균): {avg_mem:.1f} MB")
            self.logger.info(f"  - 메모리 (최소): {min_mem:.1f} MB")
            self.logger.info(f"  - 메모리 (최대): {max_mem:.1f} MB")
            self.logger.info(f"  - 메모리 증가: {max_mem - min_mem:.1f} MB")

            self.logger.info(f"\n🔧 최적화 통계:")
            self.logger.info(f"  - GC 실행: {bag_stats['gc_count']}회")
            self.logger.info(f"  - 최대 큐 크기: {bag_stats['max_queue_size']}")

            self.logger.info("\n" + "=" * 70)

        except Exception as e:
            self.logger.error(f"❌ 리포트 출력 오류: {e}")


def main():
    parser = argparse.ArgumentParser(description='SENSR Optimized Test')
    parser.add_argument('--config', '-c', default='../config/config.yaml')
    parser.add_argument('--host', '-H', help='SENSR 호스트')
    parser.add_argument('--duration', '-d', type=int, default=300, help='테스트 시간 (초)')

    args = parser.parse_args()

    config_path = args.config
    if not os.path.isabs(config_path):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, config_path)

    if not os.path.exists(config_path):
        print(f"❌ 설정 파일 없음: {config_path}")
        sys.exit(1)

    base_config = load_config(config_path)
    runtime_config = copy.deepcopy(base_config)

    if args.host:
        host_entry = find_host_entry(runtime_config, args.host)
        if not host_entry:
            print(f"❌ 호스트 '{args.host}' 없음")
            sys.exit(1)
        set_active_host(runtime_config, host_entry)
        print(f"🎯 호스트: {host_entry['id']} ({host_entry['address']})")

    runtime_config['recording']['output_directory'] = './test/output'
    os.makedirs('./test/output', exist_ok=True)

    print("=" * 70)
    print("🔧 SENSR 메모리 최적화 테스트")
    print("=" * 70)
    print(f"⏱️  테스트 시간: {args.duration}초")
    print("🔧 적용된 최적화:")
    print("  - 큐 크기 제한 (50개)")
    print("  - 주기적 가비지 컬렉션 (30초)")
    print("  - 메시지 객체 즉시 삭제")
    print("=" * 70)

    app = SensrOptimizedApp(config_path, runtime_config=runtime_config, test_duration=args.duration)
    success = app.run()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
