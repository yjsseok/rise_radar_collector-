#!/usr/bin/env python3
"""
Simple Point Cloud Recorder
SENSR 서버 부하를 최소화하면서 포인트클라우드 데이터만 수집하는 경량 스크립트
- API 호출 최소화
- 포인트클라우드 데이터만 수집
- 설정 가능한 수집 간격
"""

import os
import sys
import time
import argparse
import logging
from datetime import datetime

# 프로젝트 모듈 임포트
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

# 환경변수에서 기본 출력 경로 가져오기
DEFAULT_OUTPUT_PATH = os.environ.get('RADAR_OUTPUT_PATH', './simple_output')

from src.sensr_client import SensrClient
from src.data_processor import DataProcessor
from src.bag_recorder import BagRecorder
from src.utils import load_config, setup_logging


class SimplePointCloudRecorder:
    """경량 포인트클라우드 레코더"""
    
    def __init__(self, interval=0.5):
        """
        초기화
        
        Args:
            interval: 데이터 수집 간격 (초)
        """
        self.interval = interval
        self.last_process_time = 0.0
        self.message_count = 0
        self.start_time = time.time()
        
        # 기본 설정 생성
        self.config = {
            'sensr': {
                'host': '112.133.37.122',
                'ports': {
                    'point_cloud': 5051,
                    'rest': 9080
                },
                'reconnect_interval': 5
            },
            'recording': {
                'duration': 30,  # 30초마다 새 파일
                'output_directory': DEFAULT_OUTPUT_PATH,
                'filename_format': 'pointcloud_{timestamp}.bag',
                'pointcloud_interval': interval,
                'pointcloud_only': True,
                'skip_empty_data': True
            },
            'ros': {
                'topics': {
                    'pointcloud': '/sensr/pointcloud'
                }
            },
            'logging': {
                'level': 'INFO',
                'file': './logs/simple_recorder.log'
            }
        }
        
        self.logger = None
        self.bag_recorder = None
        self.data_processor = None
        self.sensr_client = None
        
    def setup(self):
        """컴포넌트 설정"""
        # 출력 디렉토리 생성
        os.makedirs(self.config['recording']['output_directory'], exist_ok=True)
        os.makedirs('./logs', exist_ok=True)
        
        # 로깅 설정
        self.logger = setup_logging(self.config)
        self.logger.info(f"Simple Point Cloud Recorder 시작 (간격: {self.interval}초)")
        
        # 데이터 프로세서 (API 호출 없는 버전)
        self.data_processor = DataProcessor(self.config)
        
        # Bag 레코더
        self.bag_recorder = BagRecorder(self.config)
        
        # SENSR 클라이언트 (포인트클라우드만)
        self.sensr_client = SensrClient(
            self.config,
            message_callback=self._on_message_received
        )
        
        self.logger.info("모든 컴포넌트 초기화 완료")
        
    def _on_message_received(self, message_data):
        """메시지 수신 콜백 (간격 제어 포함)"""
        try:
            current_time = time.time()
            
            # 포인트클라우드 데이터만 처리
            if message_data.get('type') != 'point_cloud':
                return
            
            # 간격 제어
            if current_time - self.last_process_time < self.interval:
                return
            
            self.last_process_time = current_time
            self.message_count += 1
            
            # 빈 데이터 체크
            data_size = len(message_data.get('data', b''))
            if data_size < 100:
                self.logger.debug(f"빈 포인트클라우드 데이터 스킵 (크기: {data_size} bytes)")
                return
            
            self.logger.info(f"포인트클라우드 처리 #{self.message_count} (크기: {data_size} bytes)")
            
            # 레코딩 시작 (첫 메시지 시)
            if not self.bag_recorder.is_recording:
                self.logger.info("레코딩 시작...")
                if not self.bag_recorder.start_recording():
                    self.logger.error("레코딩 시작 실패")
                    return
            
            # 데이터 처리
            processed_messages = self.data_processor.process_message(message_data)
            
            if processed_messages:
                # 포인트클라우드 메시지만 저장
                if isinstance(processed_messages, dict):
                    topic = processed_messages.get('topic')
                    message = processed_messages.get('message')
                    
                    if topic and message:
                        self.bag_recorder.record_message(topic, message)
                        
        except Exception as e:
            self.logger.error(f"메시지 처리 오류: {e}")
    
    def run(self, duration=None):
        """실행"""
        try:
            self.setup()
            
            self.logger.info("SENSR 서버 연결 중...")
            if not self.sensr_client.connect():
                self.logger.error("서버 연결 실패")
                return False
                
            self.logger.info("포인트클라우드 데이터 수집 시작")
            
            # 실행 시간 제한이 있는 경우
            if duration:
                end_time = time.time() + duration
                self.logger.info(f"{duration}초 동안 실행됩니다")
            else:
                end_time = None
                self.logger.info("Ctrl+C로 중단할 때까지 실행됩니다")
            
            last_status_time = time.time()
            status_interval = 30.0  # 30초마다 상태 출력
            
            while True:
                # 종료 조건 체크
                if end_time and time.time() > end_time:
                    self.logger.info("설정된 시간이 완료되어 종료합니다")
                    break
                
                # 상태 출력
                current_time = time.time()
                if current_time - last_status_time >= status_interval:
                    self._print_status()
                    last_status_time = current_time
                
                time.sleep(1.0)
                
        except KeyboardInterrupt:
            self.logger.info("사용자에 의해 중단됨")
        except Exception as e:
            self.logger.error(f"실행 오류: {e}")
        finally:
            self.stop()
            
        return True
    
    def _print_status(self):
        """상태 출력"""
        runtime = time.time() - self.start_time
        rate = self.message_count / runtime if runtime > 0 else 0
        
        self.logger.info("=" * 50)
        self.logger.info(f"실행 시간: {runtime:.1f}초")
        self.logger.info(f"수집된 메시지: {self.message_count}개")
        self.logger.info(f"평균 수집 률: {rate:.2f} msg/sec")
        self.logger.info(f"설정된 간격: {self.interval}초")
        if self.bag_recorder and self.bag_recorder.is_recording:
            self.logger.info(f"현재 bag 파일: {self.bag_recorder.current_bag_path}")
        self.logger.info("=" * 50)
    
    def stop(self):
        """종료"""
        if self.bag_recorder:
            self.bag_recorder.stop_recording()
        if self.sensr_client:
            self.sensr_client.disconnect()
        
        # 최종 상태 출력
        if self.logger:
            self._print_status()
            self.logger.info("Simple Point Cloud Recorder 종료")


def main():
    """메인 함수"""
    parser = argparse.ArgumentParser(description='Simple Point Cloud Recorder')
    
    parser.add_argument(
        '--interval', '-i',
        type=float,
        default=0.5,
        help='포인트클라우드 수집 간격 (초) - 기본값: 0.5초'
    )
    
    parser.add_argument(
        '--duration', '-d',
        type=int,
        help='실행 시간 (초) - 설정하지 않으면 무한 실행'
    )
    
    parser.add_argument(
        '--output-dir', '-o',
        default=DEFAULT_OUTPUT_PATH,
        help=f'출력 디렉토리 - 기본값: {DEFAULT_OUTPUT_PATH} (환경변수 RADAR_OUTPUT_PATH로 설정 가능)'
    )
    
    parser.add_argument(
        '--host',
        default='112.133.37.122',
        help='SENSR 서버 IP - 기본값: 112.133.37.122'
    )
    
    args = parser.parse_args()
    
    # 유효성 검사
    if args.interval < 0.1:
        print("경고: 0.1초 미만의 간격은 서버 부하를 증가시킬 수 있습니다")
        if input("계속하시겠습니까? (y/N): ").lower() != 'y':
            return
    
    print("=" * 60)
    print("🚀 Simple Point Cloud Recorder")
    print("=" * 60)
    print(f"📡 SENSR 서버: {args.host}")
    print(f"⏱️  수집 간격: {args.interval}초")
    print(f"📁 출력 디렉토리: {args.output_dir}")
    if args.duration:
        print(f"⏰ 실행 시간: {args.duration}초")
    else:
        print("⏰ 실행 시간: 무제한 (Ctrl+C로 중단)")
    print("=" * 60)
    
    # 레코더 생성 및 실행
    recorder = SimplePointCloudRecorder(interval=args.interval)
    
    # 설정 오버라이드
    recorder.config['sensr']['host'] = args.host
    recorder.config['recording']['output_directory'] = args.output_dir
    
    success = recorder.run(duration=args.duration)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()