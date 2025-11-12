#!/usr/bin/env python3
"""
SENSR Monitor with Multiprocessing
멀티프로세싱을 사용한 고속 처리 버전
"""

import os
import sys
import time
import gc
import argparse
import logging
import copy
import psutil
import multiprocessing as mp
from typing import Optional, Dict, Any

# radar_v3 구조: main_multiprocessing.py가 루트, src/가 바로 옆
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

from src.sensr_client import SensrClient
from src.data_processor_multiprocessing import DataProcessorMultiprocessing
from src.utils import (
    load_config, setup_logging, validate_config,
    setup_signal_handlers, find_host_entry, set_active_host
)
from src.bag_recorder_optimized import BagRecorderOptimized


class SensrMultiprocessingApp:
    """🚀 v2.1.0: 멀티프로세싱을 사용한 고속 처리 앱 (Graceful Shutdown)"""

    def __init__(self, config_path: str, runtime_config: Optional[Dict[str, Any]] = None,
                 test_duration: int = 300, num_workers: int = 4):
        self.config_path = config_path
        self._runtime_config = runtime_config
        self.config = None
        self.logger = None
        self.test_duration = test_duration
        self.num_workers = num_workers

        self.sensr_client = None
        self.data_processor = None
        self.bag_recorder = None

        self.is_running = False
        self.start_time = None

        # 🚀 v2.1.0: Graceful shutdown을 위한 Event
        self.shutdown_event = mp.Event()

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
            self.logger.info("🚀 SENSR Multiprocessing App 시작")

            recording_config = self.config.get('recording', {})
            self.pointcloud_interval = recording_config.get('pointcloud_interval', 0.5)
            self.output_data_interval = recording_config.get('output_data_interval', 0.5)
            self.pointcloud_only_mode = recording_config.get('pointcloud_only', False)
            self.skip_empty_data = recording_config.get('skip_empty_data', True)

            # 🚀 멀티프로세싱 데이터 프로세서
            self.logger.info(f"🚀 멀티프로세싱 초기화: {self.num_workers}개 워커")
            self.data_processor = DataProcessorMultiprocessing(self.config, num_workers=self.num_workers)
            self.data_processor.start()

            # 최적화된 Bag 레코더
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
            if self.data_processor:
                self.data_processor.stop()
            if self.sensr_client:
                self.sensr_client.stop_listening()
        except Exception as e:
            self.logger.error(f"종료 중 오류: {e}")

    def run(self):
        """🚀 v2.1.0: 메인 실행 루프 (Graceful Shutdown 및 적응형 워커 풀)"""
        if not self.initialize_components():
            return False

        # 🚀 v2.1.0: shutdown_event를 signal handler에 전달
        setup_signal_handlers(self.stop, self.shutdown_event)

        if not self.start():
            return False

        try:
            self.logger.info(f"⏱️  {self.test_duration}초 동안 멀티프로세싱 테스트")
            self.logger.info("=" * 70)

            status_interval = 30
            last_status_time = time.time()

            # 🚀 Phase 2: 배치 결과 수집 (2-5배 성능 향상)
            batch_size = 50  # 한 번에 최대 50개 결과 수집
            batch_timeout = 0.01  # 10ms 타임아웃

            # 🚀 v2.1.0: 적응형 워커 풀 체크 간격
            last_scale_check = time.time()
            scale_check_interval = 5.0

            while self.is_running and not self.shutdown_event.is_set():
                # 🚀 v2.1.0: 적응형 워커 풀 체크
                if time.time() - last_scale_check >= scale_check_interval:
                    self.data_processor._scale_workers()
                    last_scale_check = time.time()

                # 🚀 멀티프로세싱 배치 결과 수집 (비블로킹)
                results = self.data_processor.get_results_batch(
                    max_count=batch_size,
                    timeout=batch_timeout
                )

                if results:
                    if not self.bag_recorder.is_recording:
                        if not self.bag_recorder.start_recording():
                            continue

                    # 배치로 받은 결과들을 처리
                    for result in results:
                        if not result:
                            continue

                        if not isinstance(result, list):
                            result = [result]

                        for msg_info in result:
                            if msg_info:
                                self.bag_recorder.write_message(
                                    msg_info['topic'],
                                    msg_info['message'],
                                    msg_info['timestamp']
                                )

                current_time = time.time()
                elapsed = current_time - self.start_time

                # 메모리 샘플링 (1초마다)
                if int(elapsed) != int(elapsed - batch_timeout):
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
            try:
                self.stop()
            except Exception as e:
                print(f"종료 중 오류 (무시): {e}")

            try:
                self._print_final_report()
            except Exception as e:
                print(f"리포트 출력 실패 (무시): {e}")

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

            # 🚀 멀티프로세싱으로 메시지 전달 (비블로킹)
            self.data_processor.process_message(message_data)

        except Exception as e:
            self.logger.error(f"❌ 메시지 처리 오류: {e}")

    def _print_status(self):
        try:
            elapsed = time.time() - self.start_time
            bag_stats = self.bag_recorder.get_stats()
            proc_stats = self.data_processor.get_stats()
            mem_info = self.process.memory_info()
            mem_mb = mem_info.rss / 1024 / 1024

            self.logger.info("=" * 70)
            self.logger.info(f"⏱️  {elapsed:.1f}초 | 💻 메모리: {mem_mb:.1f} MB")
            self.logger.info(f"📨 수신: {self.monitor_stats['messages_received']} | "
                           f"⚙️ 처리: {proc_stats['total_processed']} | "
                           f"💾 쓰기: {bag_stats['total_written']}")
            self.logger.info(f"🚀 워커: {self.num_workers}개 | "
                           f"입력큐: {proc_stats['input_queue_size']} | "
                           f"출력큐: {proc_stats['output_queue_size']}")
            self.logger.info(f"📊 드롭 (프로세서): {proc_stats['total_dropped']} | "
                           f"드롭 (레코더): {bag_stats['total_dropped']}")
            self.logger.info("=" * 70)

        except Exception as e:
            self.logger.error(f"상태 출력 오류: {e}")

    def _print_final_report(self):
        """🚀 v2.1.1: 강화된 예외 처리로 리포트 출력"""
        try:
            print("\n")
            print("=" * 70)
            print("📊 최종 테스트 리포트 (멀티프로세싱)")
            print("=" * 70)

            elapsed = time.time() - self.start_time if self.start_time else 1.0

            # 안전하게 통계 가져오기
            try:
                bag_stats = self.bag_recorder.get_stats() if self.bag_recorder else {}
            except:
                bag_stats = {'total_written': 0, 'total_dropped': 0}

            try:
                proc_stats = self.data_processor.get_stats() if self.data_processor else {}
            except:
                proc_stats = {'total_processed': 0, 'total_dropped': 0, 'avg_process_time': 0}

            avg_mem = sum(self.monitor_stats['memory_samples']) / len(self.monitor_stats['memory_samples']) if self.monitor_stats['memory_samples'] else 0
            max_mem = max(self.monitor_stats['memory_samples']) if self.monitor_stats['memory_samples'] else 0
            min_mem = min(self.monitor_stats['memory_samples']) if self.monitor_stats['memory_samples'] else 0

            print(f"\n⏱️  총 실행 시간: {elapsed:.2f}초")

            print(f"\n📨 메시지 수신:")
            print(f"  - 총 수신: {self.monitor_stats.get('messages_received', 0)}")
            print(f"  - 포인트클라우드: {self.monitor_stats.get('pointcloud_count', 0)}")
            print(f"  - Output Data: {self.monitor_stats.get('output_data_count', 0)}")

            print(f"\n⚙️ 멀티프로세싱 통계:")
            print(f"  - 워커 수: {self.num_workers}개")
            print(f"  - 총 처리: {proc_stats.get('total_processed', 0)}")
            print(f"  - 드롭: {proc_stats.get('total_dropped', 0)}")
            print(f"  - 평균 처리 시간: {proc_stats.get('avg_process_time', 0)*1000:.2f}ms")

            print(f"\n💾 디스크 쓰기:")
            print(f"  - 총 쓰기: {bag_stats.get('total_written', 0)}")
            print(f"  - 드롭된 메시지: {bag_stats.get('total_dropped', 0)}")

            # 처리량 계산
            throughput_received = self.monitor_stats.get('messages_received', 0) / elapsed
            throughput_processed = proc_stats.get('total_processed', 0) / elapsed
            throughput_written = bag_stats.get('total_written', 0) / elapsed

            print(f"\n🚀 처리량:")
            print(f"  - 수신: {throughput_received:.1f} msg/s")
            print(f"  - 처리: {throughput_processed:.1f} msg/s")
            print(f"  - 쓰기: {throughput_written:.1f} msg/s")

            print(f"\n💻 시스템 리소스:")
            print(f"  - 메모리 (평균): {avg_mem:.1f} MB")
            print(f"  - 메모리 (최소): {min_mem:.1f} MB")
            print(f"  - 메모리 (최대): {max_mem:.1f} MB")
            print(f"  - 메모리 증가: {max_mem - min_mem:.1f} MB")

            print("\n" + "=" * 70)

        except Exception as e:
            print(f"❌ 리포트 출력 오류: {e}")
            import traceback
            traceback.print_exc()


def main():
    parser = argparse.ArgumentParser(description='SENSR Multiprocessing Test')
    parser.add_argument('--config', '-c', default='./config/config.yaml')
    parser.add_argument('--host', '-H', help='SENSR 호스트')
    parser.add_argument('--duration', '-d', type=int, default=300, help='테스트 시간 (초)')
    parser.add_argument('--workers', '-w', type=int, default=4, help='워커 프로세스 수 (기본: 4)')

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

    runtime_config['recording']['output_directory'] = './output'
    os.makedirs('./output', exist_ok=True)

    print("=" * 70)
    print("🚀 SENSR 멀티프로세싱 고속 처리 테스트")
    print("=" * 70)
    print(f"⏱️  테스트 시간: {args.duration}초")
    print(f"🚀 워커 프로세스: {args.workers}개")
    print("🔧 적용된 최적화:")
    print("  - 멀티프로세싱 병렬 처리")
    print("  - 큐 크기 제한 (100개)")
    print("  - 불필요한 로깅 제거")
    print("  - 비블로킹 결과 수집")
    print("=" * 70)

    app = SensrMultiprocessingApp(
        config_path,
        runtime_config=runtime_config,
        test_duration=args.duration,
        num_workers=args.workers
    )
    success = app.run()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
