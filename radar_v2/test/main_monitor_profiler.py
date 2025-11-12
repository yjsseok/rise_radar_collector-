#!/usr/bin/env python3
"""
SENSR Monitor with Detailed Memory Profiling
메모리 누수 지점을 찾기 위한 상세 프로파일링 버전
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

# 원본 소스 경로 추가
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

from bag_recorder_monitor import BagRecorderMonitor


class MemoryProfiler:
    """메모리 사용량 상세 추적"""

    def __init__(self):
        self.process = psutil.Process()
        self.snapshots = []

    def snapshot(self, label: str):
        """메모리 스냅샷 저장"""
        mem_info = self.process.memory_info()
        mem_mb = mem_info.rss / 1024 / 1024

        # 가비지 컬렉션 통계
        gc_stats = gc.get_stats()
        gc_count = gc.get_count()

        snapshot = {
            'time': time.time(),
            'label': label,
            'memory_mb': mem_mb,
            'gc_count': gc_count,
            'gc_stats': gc_stats,
        }

        self.snapshots.append(snapshot)
        return snapshot

    def get_growth(self, from_label: str, to_label: str):
        """두 지점 간 메모리 증가량 계산"""
        from_snap = None
        to_snap = None

        for snap in self.snapshots:
            if snap['label'] == from_label and from_snap is None:
                from_snap = snap
            if snap['label'] == to_label:
                to_snap = snap

        if from_snap and to_snap:
            growth = to_snap['memory_mb'] - from_snap['memory_mb']
            time_diff = to_snap['time'] - from_snap['time']
            return {
                'growth_mb': growth,
                'time_sec': time_diff,
                'rate_mb_per_sec': growth / time_diff if time_diff > 0 else 0
            }
        return None


class SensrMemoryProfileApp:
    """메모리 프로파일링 앱"""

    def __init__(self, config_path: str, runtime_config: Optional[Dict[str, Any]] = None, test_duration: int = 300):
        self.config_path = config_path
        self._runtime_config = runtime_config
        self.config = None
        self.logger = None
        self.test_duration = test_duration

        # 컴포넌트
        self.sensr_client = None
        self.data_processor = None
        self.bag_recorder = None

        self.is_running = False
        self.start_time = None

        # 🔍 메모리 프로파일러
        self.profiler = MemoryProfiler()

        # 통계
        self.monitor_stats = {
            'messages_received': 0,
            'pointcloud_count': 0,
            'output_data_count': 0,
            'message_processing_times': [],
        }

        self.process = psutil.Process()

        self.last_pointcloud_time = 0.0
        self.last_output_data_time = 0.0
        self.pointcloud_interval = 1.0
        self.output_data_interval = 1.0
        self.pointcloud_only_mode = False
        self.skip_empty_data = True

    def initialize_components(self) -> bool:
        """컴포넌트 초기화"""
        try:
            self.profiler.snapshot('init_start')

            if self._runtime_config is not None:
                self.config = self._runtime_config
            else:
                self.config = load_config(self.config_path)

            if not validate_config(self.config):
                return False

            self.profiler.snapshot('config_loaded')

            self.logger = setup_logging(self.config)
            self.logger.info("🔬 Memory Profiler App 시작")

            recording_config = self.config.get('recording', {})
            self.pointcloud_interval = recording_config.get('pointcloud_interval', 0.5)
            self.output_data_interval = recording_config.get('output_data_interval', 0.5)
            self.pointcloud_only_mode = recording_config.get('pointcloud_only', False)
            self.skip_empty_data = recording_config.get('skip_empty_data', True)

            self.profiler.snapshot('before_data_processor')
            self.data_processor = DataProcessor(self.config)
            self.profiler.snapshot('after_data_processor')

            self.profiler.snapshot('before_bag_recorder')
            self.bag_recorder = BagRecorderMonitor(self.config)
            self.profiler.snapshot('after_bag_recorder')

            self.profiler.snapshot('before_sensr_client')
            self.sensr_client = SensrClient(
                self.config,
                message_callback=self._on_message_received
            )
            self.profiler.snapshot('after_sensr_client')

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

            self.profiler.snapshot('before_connect')
            if not self.sensr_client.connect():
                return False
            self.profiler.snapshot('after_connect')

            self.sensr_client.start_listening()
            time.sleep(2)

            self.profiler.snapshot('ready')
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
            self.logger.info(f"⏱️  {self.test_duration}초 동안 프로파일링 테스트")
            self.logger.info("=" * 70)

            status_interval = 30  # 30초마다 상태 출력
            last_status_time = time.time()
            snapshot_counter = 0

            while self.is_running:
                time.sleep(1.0)

                current_time = time.time()
                elapsed = current_time - self.start_time

                # 주기적 메모리 스냅샷
                if int(elapsed) % 30 == 0 and elapsed > 0:
                    self.profiler.snapshot(f'runtime_{snapshot_counter}')
                    snapshot_counter += 1

                    # 명시적 가비지 컬렉션
                    gc.collect()

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
            import traceback
            self.logger.error(traceback.format_exc())
        finally:
            self.profiler.snapshot('before_stop')
            self.stop()
            self.profiler.snapshot('after_stop')
            self._print_memory_analysis()

        return True

    def _on_message_received(self, message_data: Dict[str, Any]):
        """메시지 수신 콜백 (메모리 추적 포함)"""
        try:
            msg_start_time = time.time()

            current_time = time.time()
            message_type = message_data.get('type')

            self.monitor_stats['messages_received'] += 1

            # 간격 제어
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

            # 빈 데이터 필터링
            if self.skip_empty_data:
                data_size = len(message_data.get('data', b''))
                if data_size < 100:
                    return

            # 레코딩 시작
            if not self.bag_recorder.is_recording:
                self.profiler.snapshot('first_message')
                if not self.bag_recorder.start_recording():
                    return
                self.profiler.snapshot('recording_started')

            # 데이터 처리
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

            # 처리 시간 기록
            msg_processing_time = time.time() - msg_start_time
            self.monitor_stats['message_processing_times'].append(msg_processing_time)

        except Exception as e:
            self.logger.error(f"❌ 메시지 처리 오류: {e}")

    def _print_status(self):
        """주기적 상태 출력"""
        try:
            elapsed = time.time() - self.start_time
            bag_stats = self.bag_recorder.get_stats()
            mem_info = self.process.memory_info()
            mem_mb = mem_info.rss / 1024 / 1024

            self.logger.info("=" * 70)
            self.logger.info(f"⏱️  {elapsed:.1f}초 | 💻 메모리: {mem_mb:.1f} MB")
            self.logger.info(f"📨 수신: {self.monitor_stats['messages_received']} | "
                           f"💾 쓰기: {bag_stats['total_written']} | "
                           f"📊 큐: {bag_stats['current_queue_size']}")
            self.logger.info("=" * 70)

        except Exception as e:
            self.logger.error(f"상태 출력 오류: {e}")

    def _print_memory_analysis(self):
        """메모리 증가 분석 리포트"""
        try:
            self.logger.info("\n")
            self.logger.info("=" * 70)
            self.logger.info("🔬 메모리 프로파일링 리포트")
            self.logger.info("=" * 70)

            # 초기화 단계별 메모리 증가
            self.logger.info("\n📊 초기화 단계별 메모리:")

            phases = [
                ('init_start', 'config_loaded', '설정 로드'),
                ('config_loaded', 'after_data_processor', 'DataProcessor 생성'),
                ('after_data_processor', 'after_bag_recorder', 'BagRecorder 생성'),
                ('after_bag_recorder', 'after_sensr_client', 'SensrClient 생성'),
                ('ready', 'runtime_0', '첫 30초 실행'),
            ]

            for from_label, to_label, desc in phases:
                growth = self.profiler.get_growth(from_label, to_label)
                if growth:
                    self.logger.info(f"  {desc}: {growth['growth_mb']:+.1f} MB "
                                   f"({growth['time_sec']:.1f}초)")

            # 실행 시간 동안 메모리 증가
            self.logger.info("\n📈 실행 시간별 메모리 증가:")

            runtime_snapshots = [s for s in self.profiler.snapshots if s['label'].startswith('runtime_')]
            if len(runtime_snapshots) >= 2:
                for i in range(len(runtime_snapshots) - 1):
                    from_snap = runtime_snapshots[i]
                    to_snap = runtime_snapshots[i + 1]
                    growth = to_snap['memory_mb'] - from_snap['memory_mb']
                    time_diff = to_snap['time'] - from_snap['time']

                    self.logger.info(f"  {from_snap['label']} → {to_snap['label']}: "
                                   f"{growth:+.1f} MB ({time_diff:.1f}초)")

            # 전체 증가량
            if len(self.profiler.snapshots) >= 2:
                first = self.profiler.snapshots[0]
                last = self.profiler.snapshots[-1]
                total_growth = last['memory_mb'] - first['memory_mb']
                total_time = last['time'] - first['time']
                growth_rate = total_growth / (total_time / 60) if total_time > 0 else 0

                self.logger.info(f"\n💥 총 메모리 증가: {total_growth:.1f} MB")
                self.logger.info(f"⏱️  총 실행 시간: {total_time:.1f}초 ({total_time/60:.1f}분)")
                self.logger.info(f"📊 증가율: {growth_rate:.1f} MB/분")

                # 예측
                if growth_rate > 0:
                    minutes_to_10gb = (10240 - last['memory_mb']) / growth_rate
                    self.logger.info(f"⚠️  현재 증가율로 10GB 도달까지: {minutes_to_10gb:.1f}분")

            # 가비지 컬렉션 통계
            self.logger.info("\n🗑️  가비지 컬렉션:")
            gc_count = gc.get_count()
            self.logger.info(f"  세대별 객체 수: Gen0={gc_count[0]}, Gen1={gc_count[1]}, Gen2={gc_count[2]}")

            self.logger.info("\n" + "=" * 70)

        except Exception as e:
            self.logger.error(f"❌ 메모리 분석 오류: {e}")
            import traceback
            self.logger.error(traceback.format_exc())


def main():
    parser = argparse.ArgumentParser(description='SENSR Memory Profiler')
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
    print("🔬 SENSR 메모리 프로파일링 테스트")
    print("=" * 70)
    print(f"⏱️  테스트 시간: {args.duration}초")
    print("=" * 70)

    app = SensrMemoryProfileApp(config_path, runtime_config=runtime_config, test_duration=args.duration)
    success = app.run()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
