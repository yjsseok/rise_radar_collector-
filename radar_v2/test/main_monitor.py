#!/usr/bin/env python3
"""
SENSR LiDAR Data Recorder - Monitoring Test Version
10초 테스트로 메모리/큐/처리 속도를 모니터링하는 버전
"""

import os
import sys
import time
import argparse
import logging
import copy
import psutil
from typing import Optional, Dict, Any

# 원본 소스 경로 추가
parent_dir = os.path.join(os.path.dirname(__file__), '..')
sys.path.insert(0, parent_dir)

# test/src 경로도 추가
test_src_dir = os.path.join(os.path.dirname(__file__), 'src')
sys.path.insert(0, test_src_dir)

from src.sensr_client import SensrClient
from src.data_processor import DataProcessor
from src.utils import (
    load_config, setup_logging, validate_config,
    setup_signal_handlers, find_host_entry, set_active_host
)

# 모니터링 버전 bag recorder 임포트
from bag_recorder_monitor import BagRecorderMonitor


class SensrMonitorApp:
    """모니터링 기능이 추가된 SENSR 데이터 레코더"""

    def __init__(self, config_path: str, runtime_config: Optional[Dict[str, Any]] = None, test_duration: int = 10):
        """
        초기화

        Args:
            config_path: 설정 파일 경로
            runtime_config: 실행 설정
            test_duration: 테스트 실행 시간 (초)
        """
        self.config_path = config_path
        self._runtime_config = runtime_config
        self.config = None
        self.logger = None
        self.test_duration = test_duration

        # 컴포넌트
        self.sensr_client = None
        self.data_processor = None
        self.bag_recorder = None

        # 실행 상태
        self.is_running = False
        self.start_time = None

        # 🔍 모니터링 통계
        self.monitor_stats = {
            'messages_received': 0,
            'pointcloud_count': 0,
            'output_data_count': 0,
            'memory_samples': [],
            'cpu_samples': [],
        }

        # 프로세스 정보
        self.process = psutil.Process()

        # 데이터 수집 간격
        self.last_pointcloud_time = 0.0
        self.last_output_data_time = 0.0
        self.pointcloud_interval = 0.1
        self.output_data_interval = 0.1
        self.pointcloud_only_mode = False
        self.skip_empty_data = True

    def initialize_components(self) -> bool:
        """컴포넌트 초기화"""
        try:
            print(f"📄 설정 파일 로드: {self.config_path}")

            if self._runtime_config is not None:
                self.config = self._runtime_config
            else:
                self.config = load_config(self.config_path)

            if not validate_config(self.config):
                print("❌ 설정 파일 유효성 검사 실패")
                return False

            # 로깅 설정
            self.logger = setup_logging(self.config)
            self.logger.info("🚀 SENSR Monitor App 시작")

            # 데이터 수집 간격 설정
            recording_config = self.config.get('recording', {})
            self.pointcloud_interval = recording_config.get('pointcloud_interval', 0.5)
            self.output_data_interval = recording_config.get('output_data_interval', 0.5)
            self.pointcloud_only_mode = recording_config.get('pointcloud_only', False)
            self.skip_empty_data = recording_config.get('skip_empty_data', True)

            # 데이터 프로세서 초기화
            self.logger.info("🔧 데이터 프로세서 초기화")
            self.data_processor = DataProcessor(self.config)

            # 🔍 모니터링 Bag 레코더 초기화
            self.logger.info("🔧 Bag 레코더 초기화 (모니터링 모드)")
            self.bag_recorder = BagRecorderMonitor(self.config)

            # SENSR 클라이언트 초기화
            self.logger.info("🔧 SENSR 클라이언트 초기화")
            self.sensr_client = SensrClient(
                self.config,
                message_callback=self._on_message_received
            )

            self.logger.info("✅ 모든 컴포넌트 초기화 완료")
            return True

        except Exception as e:
            if self.logger:
                self.logger.error(f"❌ 초기화 실패: {e}")
            else:
                print(f"❌ 초기화 실패: {e}")
            return False

    def start(self) -> bool:
        """애플리케이션 시작"""
        if self.is_running:
            self.logger.warning("이미 실행 중")
            return False

        try:
            self.is_running = True
            self.start_time = time.time()

            # SENSR 서버 연결
            self.logger.info("🔌 SENSR 서버 연결 중...")
            if not self.sensr_client.connect():
                self.logger.error("❌ SENSR 서버 연결 실패")
                return False

            # 데이터 수신 시작
            self.logger.info("📡 데이터 수신 시작")
            self.sensr_client.start_listening()

            time.sleep(2)

            self.logger.info("✅ 연결 완료! 데이터 수신 중...")
            return True

        except Exception as e:
            self.logger.error(f"❌ 시작 실패: {e}")
            self.is_running = False
            return False

    def stop(self):
        """애플리케이션 정지"""
        if not self.is_running:
            return

        self.logger.info("🛑 종료 중...")
        self.is_running = False

        try:
            if self.bag_recorder:
                self.bag_recorder.stop_recording()

            if self.sensr_client:
                self.sensr_client.stop_listening()

        except Exception as e:
            self.logger.error(f"종료 중 오류: {e}")

    def run(self):
        """메인 실행 루프 (테스트 모드)"""
        if not self.initialize_components():
            return False

        setup_signal_handlers(self.stop)

        if not self.start():
            return False

        try:
            self.logger.info(f"⏱️  {self.test_duration}초 동안 모니터링 테스트 실행")
            self.logger.info("=" * 70)

            status_interval = 5  # 5초마다 상태 출력
            last_status_time = time.time()

            while self.is_running:
                time.sleep(0.5)

                current_time = time.time()
                elapsed = current_time - self.start_time

                # 🔍 메모리/CPU 샘플링
                mem_info = self.process.memory_info()
                mem_mb = mem_info.rss / 1024 / 1024
                cpu_percent = self.process.cpu_percent(interval=0.1)

                self.monitor_stats['memory_samples'].append(mem_mb)
                self.monitor_stats['cpu_samples'].append(cpu_percent)

                # 주기적 상태 출력
                if current_time - last_status_time >= status_interval:
                    self._print_status()
                    last_status_time = current_time

                # 테스트 종료 조건
                if elapsed >= self.test_duration:
                    self.logger.info(f"⏰ {self.test_duration}초 완료! 테스트 종료")
                    break

        except KeyboardInterrupt:
            self.logger.info("⌨️  사용자 중단")
        except Exception as e:
            self.logger.error(f"❌ 실행 오류: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
        finally:
            self.stop()
            self._print_final_report()

        return True

    def _on_message_received(self, message_data: Dict[str, Any]):
        """메시지 수신 콜백"""
        try:
            current_time = time.time()
            message_type = message_data.get('type')

            # 🔍 수신 카운터 증가
            self.monitor_stats['messages_received'] += 1

            # 데이터 수집 간격 제어
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

            # 첫 메시지 수신 시 레코딩 시작
            if not self.bag_recorder.is_recording:
                self.logger.info("🎬 첫 메시지 수신! 레코딩 시작")
                if not self.bag_recorder.start_recording():
                    self.logger.error("❌ 레코딩 시작 실패")
                    return

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

        except Exception as e:
            self.logger.error(f"❌ 메시지 처리 오류: {e}")

    def _print_status(self):
        """주기적 상태 출력"""
        try:
            elapsed = time.time() - self.start_time

            # Bag 레코더 통계
            bag_stats = self.bag_recorder.get_stats()

            # 메모리 정보
            mem_info = self.process.memory_info()
            mem_mb = mem_info.rss / 1024 / 1024

            self.logger.info("=" * 70)
            self.logger.info(f"⏱️  실행 시간: {elapsed:.1f}초")
            self.logger.info(f"📨 수신: {self.monitor_stats['messages_received']} "
                           f"(PC: {self.monitor_stats['pointcloud_count']}, "
                           f"OD: {self.monitor_stats['output_data_count']})")
            self.logger.info(f"💾 디스크 쓰기: {bag_stats['total_written']} "
                           f"(드롭: {bag_stats['total_dropped']})")
            self.logger.info(f"📊 큐: 현재={bag_stats['current_queue_size']}, "
                           f"평균={bag_stats['avg_queue_size']:.1f}, "
                           f"최대={bag_stats['max_queue_size']}")
            self.logger.info(f"⚡ 속도: 수신={bag_stats['receive_rate']:.2f} msg/s, "
                           f"쓰기={bag_stats['write_rate']:.2f} msg/s")
            self.logger.info(f"⏲️  쓰기시간: 평균={bag_stats['avg_write_time_ms']:.2f}ms, "
                           f"최대={bag_stats['max_write_time_ms']:.2f}ms")
            self.logger.info(f"💻 메모리: {mem_mb:.1f} MB")
            self.logger.info("=" * 70)

        except Exception as e:
            self.logger.error(f"상태 출력 오류: {e}")

    def _print_final_report(self):
        """최종 리포트 출력"""
        try:
            self.logger.info("\n")
            self.logger.info("=" * 70)
            self.logger.info("📊 최종 테스트 리포트")
            self.logger.info("=" * 70)

            elapsed = time.time() - self.start_time
            bag_stats = self.bag_recorder.get_stats()

            # 메모리 통계
            avg_mem = sum(self.monitor_stats['memory_samples']) / len(self.monitor_stats['memory_samples'])
            max_mem = max(self.monitor_stats['memory_samples'])
            min_mem = min(self.monitor_stats['memory_samples'])

            # CPU 통계
            avg_cpu = sum(self.monitor_stats['cpu_samples']) / len(self.monitor_stats['cpu_samples'])
            max_cpu = max(self.monitor_stats['cpu_samples'])

            self.logger.info(f"\n⏱️  총 실행 시간: {elapsed:.2f}초")
            self.logger.info(f"\n📨 메시지 수신:")
            self.logger.info(f"  - 총 수신: {self.monitor_stats['messages_received']}")
            self.logger.info(f"  - 포인트클라우드: {self.monitor_stats['pointcloud_count']}")
            self.logger.info(f"  - Output Data: {self.monitor_stats['output_data_count']}")
            self.logger.info(f"  - 수신 속도: {bag_stats['receive_rate']:.2f} msg/s")

            self.logger.info(f"\n💾 디스크 쓰기:")
            self.logger.info(f"  - 총 쓰기: {bag_stats['total_written']}")
            self.logger.info(f"  - 드롭된 메시지: {bag_stats['total_dropped']}")
            self.logger.info(f"  - 쓰기 속도: {bag_stats['write_rate']:.2f} msg/s")
            self.logger.info(f"  - 평균 쓰기 시간: {bag_stats['avg_write_time_ms']:.2f} ms")
            self.logger.info(f"  - 최대 쓰기 시간: {bag_stats['max_write_time_ms']:.2f} ms")

            self.logger.info(f"\n📊 메시지 큐:")
            self.logger.info(f"  - 평균 크기: {bag_stats['avg_queue_size']:.1f}")
            self.logger.info(f"  - 최대 크기: {bag_stats['max_queue_size']}")
            self.logger.info(f"  - 최종 크기: {bag_stats['current_queue_size']}")

            self.logger.info(f"\n💻 시스템 리소스:")
            self.logger.info(f"  - 메모리 (평균): {avg_mem:.1f} MB")
            self.logger.info(f"  - 메모리 (최소): {min_mem:.1f} MB")
            self.logger.info(f"  - 메모리 (최대): {max_mem:.1f} MB")
            self.logger.info(f"  - CPU (평균): {avg_cpu:.1f}%")
            self.logger.info(f"  - CPU (최대): {max_cpu:.1f}%")

            # 🔍 분석 및 권장사항
            self.logger.info(f"\n🔍 분석:")

            # 큐 상태 분석
            if bag_stats['max_queue_size'] > 50:
                self.logger.warning(f"  ⚠️  큐 크기가 큰 편입니다 (최대 {bag_stats['max_queue_size']})")
                self.logger.warning(f"     → 디스크 쓰기 속도가 수신 속도를 따라가지 못합니다")
            else:
                self.logger.info(f"  ✅ 큐 크기 정상 (최대 {bag_stats['max_queue_size']})")

            # 드롭 메시지 분석
            if bag_stats['total_dropped'] > 0:
                self.logger.warning(f"  ⚠️  {bag_stats['total_dropped']}개 메시지 드롭!")
            else:
                self.logger.info(f"  ✅ 메시지 드롭 없음")

            # 처리 효율성
            if bag_stats['total_written'] > 0:
                efficiency = (bag_stats['total_written'] / self.monitor_stats['messages_received']) * 100
                self.logger.info(f"  📈 처리 효율: {efficiency:.1f}% "
                               f"({bag_stats['total_written']}/{self.monitor_stats['messages_received']})")

            self.logger.info("\n" + "=" * 70)

        except Exception as e:
            self.logger.error(f"❌ 리포트 출력 오류: {e}")


def main():
    """메인 함수"""
    parser = argparse.ArgumentParser(description='SENSR Monitor Test')
    parser.add_argument('--config', '-c', default='../config/config.yaml', help='설정 파일 경로')
    parser.add_argument('--host', '-H', help='SENSR 호스트 ID')
    parser.add_argument('--duration', '-d', type=int, default=10, help='테스트 시간 (초, 기본 10초)')
    parser.add_argument('--verbose', '-v', action='store_true', help='상세 로그')

    args = parser.parse_args()

    # 설정 파일 경로
    config_path = args.config
    if not os.path.isabs(config_path):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, config_path)

    if not os.path.exists(config_path):
        print(f"❌ 설정 파일 없음: {config_path}")
        sys.exit(1)

    # 설정 로드
    base_config = load_config(config_path)
    runtime_config = copy.deepcopy(base_config)

    if args.verbose:
        runtime_config.setdefault('logging', {})['level'] = 'DEBUG'

    # 호스트 설정
    if args.host:
        host_entry = find_host_entry(runtime_config, args.host)
        if not host_entry:
            print(f"❌ 호스트 '{args.host}' 없음")
            sys.exit(1)
        set_active_host(runtime_config, host_entry)
        print(f"🎯 호스트: {host_entry['id']} ({host_entry['address']})")

    # 테스트용 출력 디렉토리
    runtime_config['recording']['output_directory'] = './test/output'
    os.makedirs('./test/output', exist_ok=True)

    print("=" * 70)
    print("🧪 SENSR 모니터링 테스트")
    print("=" * 70)
    print(f"⏱️  테스트 시간: {args.duration}초")
    print(f"📁 출력: {runtime_config['recording']['output_directory']}")
    print("=" * 70)

    # 앱 실행
    app = SensrMonitorApp(config_path, runtime_config=runtime_config, test_duration=args.duration)
    success = app.run()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
