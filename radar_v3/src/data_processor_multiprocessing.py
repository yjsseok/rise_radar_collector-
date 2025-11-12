#!/usr/bin/env python3
"""
Data Processor with Multiprocessing
멀티프로세싱을 사용한 고속 데이터 처리 모듈
"""

import struct
import time
import logging
import requests
import multiprocessing as mp
from multiprocessing import Process, Queue, Manager
from typing import Dict, Any, Optional, List
import numpy as np
import os
import sys

def _worker_process_protobuf(input_queue: mp.Queue, output_queue: mp.Queue, config: Dict[str, Any],
                             worker_id: int, timing_dict: Optional[Dict] = None):
    """
    🚀 v2.1.0: 멀티프로세싱 워커 프로세스 (타이밍 측정 추가)

    Args:
        input_queue: 입력 메시지 큐
        output_queue: 출력 결과 큐
        config: 설정 딕셔너리
        worker_id: 워커 ID
        timing_dict: 타이밍 정보를 공유할 Manager.dict()
    """
    # 각 워커 프로세스에서 독립적으로 DataProcessor를 import
    # (멀티프로세싱 시 각 프로세스는 독립된 Python 인터프리터)
    current_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(current_dir)

    # sys.path 설정
    if current_dir not in sys.path:
        sys.path.insert(0, current_dir)
    if parent_dir not in sys.path:
        sys.path.insert(0, parent_dir)

    # DataProcessor import
    try:
        from src.data_processor import DataProcessor
    except ImportError:
        # fallback: 직접 파일에서 import
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "data_processor",
            os.path.join(current_dir, "data_processor.py")
        )
        data_processor_module = importlib.util.module_from_spec(spec)
        sys.modules['data_processor'] = data_processor_module
        spec.loader.exec_module(data_processor_module)
        DataProcessor = data_processor_module.DataProcessor

    # 각 워커 프로세스에서 독립적인 DataProcessor 인스턴스 생성
    processor = DataProcessor(config)

    # 로깅 설정 (워커별로 구분)
    logger = logging.getLogger(f"worker_{worker_id}")

    logger.info(f"🔧 워커 {worker_id} 시작")

    # 로컬 타이밍 통계
    local_timing = {
        'protobuf_parse_ms': [],
        'ros_msg_build_ms': [],
        'total_process_ms': []
    }

    while True:
        try:
            # 입력 큐에서 메시지 가져오기 (타임아웃 1초)
            message_data = input_queue.get(timeout=1.0)

            # 종료 신호 확인
            if message_data is None:
                logger.info(f"🛑 워커 {worker_id} 종료 신호 수신")
                break

            # 🚀 v2.1.0: 단계별 타이밍 측정
            timing_start = time.time()

            # Protobuf 파싱 시간 측정 (DataProcessor 내부에서 측정됨)
            result = processor.process_message(message_data)

            total_time_ms = (time.time() - timing_start) * 1000

            # 로컬 타이밍 저장
            local_timing['total_process_ms'].append(total_time_ms)

            # 주기적으로 통계 업데이트 (100회마다)
            if len(local_timing['total_process_ms']) >= 100 and timing_dict is not None:
                avg_total = sum(local_timing['total_process_ms']) / len(local_timing['total_process_ms'])
                timing_dict[f'worker_{worker_id}_avg_ms'] = avg_total
                local_timing['total_process_ms'] = []  # 리셋

            # 결과를 출력 큐에 넣기
            if result:
                output_queue.put({
                    'worker_id': worker_id,
                    'result': result,
                    'process_time': total_time_ms / 1000  # 초 단위
                })

        except mp.queues.Empty:
            # 타임아웃 발생 - 정상적인 상황, 계속 대기
            continue
        except KeyboardInterrupt:
            # 🚀 Graceful shutdown: Ctrl+C 시 깨끗하게 종료
            logger.info(f"⌨️  워커 {worker_id}: 사용자 중단 신호 수신, 정리 중...")
            break
        except Exception as e:
            # 실제 에러 발생 - 상세 정보 출력
            import traceback
            logger.error(f"워커 {worker_id} 오류: {type(e).__name__}: {str(e)}")
            logger.error(f"상세:\n{traceback.format_exc()}")
            continue

    logger.info(f"✅ 워커 {worker_id} 종료")


class DataProcessorMultiprocessing:
    """🚀 v2.1.0: 멀티프로세싱을 사용하는 고속 데이터 프로세서 (적응형 워커 풀)"""

    def __init__(self, config: Dict[str, Any], num_workers: int = 4):
        """
        DataProcessorMultiprocessing 초기화

        Args:
            config: 설정 딕셔너리
            num_workers: 워커 프로세스 수 (기본값: 4)
        """
        self.config = config
        self.logger = logging.getLogger(__name__)

        # 🚀 v2.1.0: 설정에서 멀티프로세싱 파라미터 읽기
        mp_config = config.get('multiprocessing', {})
        self.num_workers = mp_config.get('num_workers', num_workers)
        self.max_workers = mp_config.get('max_workers', num_workers * 2)
        self.scale_up_threshold = mp_config.get('scale_up_threshold', 50)
        self.scale_down_seconds = mp_config.get('scale_down_seconds', 30)

        # 큐 크기
        input_queue_size = mp_config.get('input_queue_size', 100)
        output_queue_size = mp_config.get('output_queue_size', 1000)

        self.input_queue = mp.Queue(maxsize=input_queue_size)
        self.output_queue = mp.Queue(maxsize=output_queue_size)

        # 🚀 v2.1.0: 타이밍 정보 공유를 위한 Manager
        self.manager = Manager()
        self.timing_dict = self.manager.dict()

        # 워커 프로세스 리스트
        self.workers = []

        # 적응형 워커 풀 상태
        self.last_scale_check_time = time.time()
        self.last_busy_time = time.time()

        # 통계
        self.stats = {
            'total_processed': 0,
            'total_dropped': 0,
            'process_times': [],
        }

        self.is_running = False

    def start(self):
        """🚀 v2.1.0: 워커 프로세스 시작 (타이밍 측정 포함)"""
        if self.is_running:
            self.logger.warning("이미 실행 중입니다.")
            return

        self.is_running = True

        # 워커 프로세스 시작
        for i in range(self.num_workers):
            worker = Process(
                target=_worker_process_protobuf,
                args=(self.input_queue, self.output_queue, self.config, i, self.timing_dict),
                daemon=True
            )
            worker.start()
            self.workers.append(worker)

        self.logger.info(f"🚀 멀티프로세싱 시작: {self.num_workers}개 워커 (최대: {self.max_workers}개)")

    def stop(self):
        """🚀 v2.1.0: 워커 프로세스 정상 종료"""
        if not self.is_running:
            return

        self.is_running = False

        # 모든 워커에 종료 신호 전송
        for _ in range(len(self.workers)):
            try:
                self.input_queue.put(None, timeout=1)
            except:
                pass

        # 모든 워커 프로세스 종료 대기
        shutdown_timeout = self.config.get('shutdown', {}).get('timeout_s', 10)
        for worker in self.workers:
            worker.join(timeout=shutdown_timeout)
            if worker.is_alive():
                self.logger.warning(f"워커가 정상 종료되지 않아 강제 종료합니다: PID {worker.pid}")
                worker.terminate()

        self.workers.clear()
        self.logger.info("🛑 멀티프로세싱 중지")

        # Manager 정리
        try:
            self.manager.shutdown()
        except:
            pass

    def process_message(self, message_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        메시지를 워커 프로세스에 전달 (비블로킹)

        Args:
            message_data: SENSR 메시지 데이터

        Returns:
            None (결과는 get_result()로 가져옴)
        """
        if not self.is_running:
            self.logger.warning("멀티프로세싱이 시작되지 않았습니다.")
            return None

        try:
            # 입력 큐에 메시지 추가 (비블로킹)
            self.input_queue.put_nowait(message_data)
            return None  # 비동기 처리

        except Exception as e:
            self.stats['total_dropped'] += 1
            self.logger.warning(f"입력 큐 가득참! 메시지 드롭 (총 {self.stats['total_dropped']}개)")
            return None

    def get_result(self, timeout: float = 0.1) -> Optional[Dict[str, Any]]:
        """
        처리 결과 가져오기

        Args:
            timeout: 타임아웃 (초)

        Returns:
            처리된 결과 또는 None
        """
        try:
            result_data = self.output_queue.get(timeout=timeout)

            # 통계 업데이트
            self.stats['total_processed'] += 1
            self.stats['process_times'].append(result_data['process_time'])

            return result_data['result']

        except:
            return None

    def get_results_batch(self, max_count: int = 50, timeout: float = 0.01) -> List[Dict[str, Any]]:
        """
        🚀 Phase 2: 여러 결과를 배치로 가져오기 (2-5배 성능 향상)

        Args:
            max_count: 최대 가져올 결과 개수
            timeout: 타임아웃 (초)

        Returns:
            처리된 결과 리스트
        """
        results = []
        deadline = time.time() + timeout

        while len(results) < max_count and time.time() < deadline:
            try:
                result_data = self.output_queue.get_nowait()

                # 통계 업데이트
                self.stats['total_processed'] += 1
                self.stats['process_times'].append(result_data['process_time'])

                results.append(result_data['result'])

            except:
                # 큐가 비었거나 타임아웃
                if len(results) > 0:
                    # 이미 일부 결과를 얻었으면 바로 반환
                    break
                # 결과가 없으면 짧게 대기
                time.sleep(0.001)  # 1ms

        return results

    def _scale_workers(self):
        """
        🚀 v2.1.0: 적응형 워커 풀 - 부하에 따라 워커 수 조절
        """
        if not self.is_running:
            return

        current_time = time.time()

        # 주기적 체크 (5초마다)
        if current_time - self.last_scale_check_time < 5.0:
            return

        self.last_scale_check_time = current_time

        try:
            input_qsize = self.input_queue.qsize()
            current_worker_count = len(self.workers)

            # Scale Up: 입력 큐가 임계값 이상이고 워커 수가 최대보다 적으면
            if input_qsize >= self.scale_up_threshold and current_worker_count < self.max_workers:
                new_worker_id = current_worker_count
                worker = Process(
                    target=_worker_process_protobuf,
                    args=(self.input_queue, self.output_queue, self.config, new_worker_id, self.timing_dict),
                    daemon=True
                )
                worker.start()
                self.workers.append(worker)
                self.logger.info(f"🚀 워커 추가: {current_worker_count} → {len(self.workers)}개 (큐 크기: {input_qsize})")
                self.last_busy_time = current_time

            # Scale Down: 일정 시간 idle이고 워커 수가 초기값보다 많으면
            elif input_qsize < self.scale_up_threshold // 2 and current_worker_count > self.num_workers:
                idle_time = current_time - self.last_busy_time
                if idle_time >= self.scale_down_seconds:
                    # 마지막 워커 제거
                    worker = self.workers.pop()
                    try:
                        self.input_queue.put(None, timeout=0.5)
                        worker.join(timeout=2)
                        if worker.is_alive():
                            worker.terminate()
                        self.logger.info(f"🔽 워커 제거: {current_worker_count} → {len(self.workers)}개 (idle: {idle_time:.1f}초)")
                    except:
                        self.workers.append(worker)  # 제거 실패 시 복원

            # 큐에 작업이 있으면 busy 타임 업데이트
            if input_qsize > 0:
                self.last_busy_time = current_time

        except Exception as e:
            self.logger.error(f"워커 스케일링 오류: {e}")

    def get_stats(self) -> Dict[str, Any]:
        """🚀 v2.1.0: 통계 정보 반환 (타이밍 정보 포함)"""
        avg_process_time = 0
        if self.stats['process_times']:
            avg_process_time = sum(self.stats['process_times']) / len(self.stats['process_times'])

        # 워커별 평균 처리 시간 수집
        worker_timings = {}
        for key, value in self.timing_dict.items():
            worker_timings[key] = value

        return {
            'num_workers': len(self.workers),
            'max_workers': self.max_workers,
            'total_processed': self.stats['total_processed'],
            'total_dropped': self.stats['total_dropped'],
            'avg_process_time': avg_process_time,
            'input_queue_size': self.input_queue.qsize(),
            'output_queue_size': self.output_queue.qsize(),
            'worker_timings': worker_timings,
        }
