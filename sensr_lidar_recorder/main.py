#!/usr/bin/env python3
"""
SENSR LiDAR Data Recorder - Main Application
Seoul Robotics SENSR 시스템에서 라이다 데이터를 수집하여 ROS bag으로 저장하는 메인 애플리케이션
"""

import os
import sys
import time
import argparse
import logging
import copy
from typing import Optional, Dict, Any

# 프로젝트 모듈 임포트
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from src.sensr_client import SensrClient
from src.data_processor import DataProcessor
from src.bag_recorder import BagRecorder
from src.utils import (
    load_config, setup_logging, validate_config, 
    setup_signal_handlers, StatusMonitor, check_disk_space,
    list_available_hosts, find_host_entry, set_active_host
)


class SensrRecorderApp:
    """SENSR 데이터 레코더 메인 애플리케이션"""
    
    def __init__(self, config_path: str, runtime_config: Optional[Dict[str, Any]] = None):
        """
        애플리케이션 초기화
        
        Args:
            config_path: 설정 파일 경로
            runtime_config: 실행 시점에 오버라이드된 설정 딕셔너리
        """
        self.config_path = config_path
        self._runtime_config = runtime_config
        self.config = None
        self.logger = None
        
        self.active_host_label: Optional[str] = None
        self.active_host_address: Optional[str] = None

        # 컴포넌트들
        self.sensr_client = None
        self.data_processor = None
        self.bag_recorder = None
        self.status_monitor = None
        
        # 실행 상태
        self.is_running = False
        self.quick_start = False  # 빠른 시작 모드
        
        # 데이터 수집 간격 제어
        self.last_pointcloud_time = 0.0
        self.last_output_data_time = 0.0
        self.pointcloud_interval = 0.1  # 기본값
        self.output_data_interval = 0.1  # 기본값
        self.pointcloud_only_mode = False
        self.skip_empty_data = True
        
    def initialize_components(self) -> bool:
        """
        애플리케이션 컴포넌트들 초기화
        
        Returns:
            초기화 성공 여부
        """
        try:
            # 설정 파일 로드
            print(f"설정 파일 로드 중: {self.config_path}")
            if self._runtime_config is not None:
                self.config = self._runtime_config
            else:
                self.config = load_config(self.config_path)
            
            # 설정 유효성 검사
            if not validate_config(self.config):
                print("설정 파일 유효성 검사 실패")
                return False
            
            # 로깅 설정
            self.logger = setup_logging(self.config)
            self.logger.info("SENSR LiDAR Data Recorder 시작")
            self.logger.info(f"설정 파일: {self.config_path}")

            sensr_cfg = self.config.get('sensr', {})
            self.active_host_address = sensr_cfg.get('host')
            self.active_host_label = sensr_cfg.get('active_host_label', self.active_host_address)
            if self.active_host_address:
                self.logger.info(f"활성 호스트: {self.active_host_label} ({self.active_host_address})")
            
            # 디스크 공간 확인
            output_dir = self.config['recording']['output_directory']
            if not check_disk_space(output_dir, 2.0):  # 최소 2GB 필요
                self.logger.warning("디스크 공간이 부족할 수 있습니다")
            
            # 데이터 수집 간격 설정 로드
            recording_config = self.config.get('recording', {})
            self.pointcloud_interval = recording_config.get('pointcloud_interval', 0.5)
            self.output_data_interval = recording_config.get('output_data_interval', 0.5)
            self.pointcloud_only_mode = recording_config.get('pointcloud_only', False)
            self.skip_empty_data = recording_config.get('skip_empty_data', True)
            
            self.logger.info(f"데이터 수집 설정:")
            self.logger.info(f"  포인트클라우드 간격: {self.pointcloud_interval}초")
            self.logger.info(f"  출력 데이터 간격: {self.output_data_interval}초")
            self.logger.info(f"  포인트클라우드 전용 모드: {self.pointcloud_only_mode}")
            
            # 상태 모니터 초기화
            self.status_monitor = StatusMonitor()
            
            # 데이터 프로세서 초기화
            self.logger.info("데이터 프로세서 초기화 중...")
            self.data_processor = DataProcessor(self.config)
            
            # Bag 레코더 초기화
            self.logger.info("Bag 레코더 초기화 중...")
            self.bag_recorder = BagRecorder(self.config)
            
            # SENSR 클라이언트 초기화
            self.logger.info("SENSR 클라이언트 초기화 중...")
            self.sensr_client = SensrClient(
                self.config,
                message_callback=self._on_message_received
            )
            
            self.logger.info("모든 컴포넌트 초기화 완료")
            return True
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"컴포넌트 초기화 실패: {e}")
            else:
                print(f"컴포넌트 초기화 실패: {e}")
            return False
    
    def start(self) -> bool:
        """
        애플리케이션 시작
        
        Returns:
            시작 성공 여부
        """
        if self.is_running:
            self.logger.warning("애플리케이션이 이미 실행 중입니다")
            return False
        
        try:
            self.is_running = True
            
            # SENSR 서버 연결
            self.logger.info("SENSR 서버 연결 중...")
            if not self.sensr_client.connect():
                self.logger.error("SENSR 서버 연결 실패")
                return False
            
            # 데이터 수신 시작
            self.logger.info("데이터 수신 시작...")
            self.sensr_client.start_listening()
            
            # 빠른 시작 모드가 아닌 경우에만 연결 대기
            if not self.quick_start:
                # 연결이 안정화될 때까지 잠시 대기
                time.sleep(2)
            else:
                self.logger.info("빠른 시작 모드: 연결 대기 생략")
            
            self.logger.info("SENSR 연결 및 데이터 수신이 시작되었습니다")
            self.logger.info("첫 번째 메시지 수신 시 자동으로 레코딩을 시작합니다")
            if self.quick_start:
                self.logger.info("빠른 시작 완료! 백그라운드에서 연결을 계속 시도합니다.")
            return True
            
        except Exception as e:
            self.logger.error(f"애플리케이션 시작 실패: {e}")
            self.is_running = False
            return False
    
    def stop(self):
        """애플리케이션 정지"""
        if not self.is_running:
            return
        
        self.logger.info("애플리케이션 종료 중...")
        self.is_running = False
        
        try:
            # 레코딩 중지
            if self.bag_recorder:
                self.logger.info("레코딩 중지 중...")
                self.bag_recorder.stop_recording()
            
            # SENSR 클라이언트 종료
            if self.sensr_client:
                self.logger.info("SENSR 연결 종료 중...")
                self.sensr_client.stop_listening()
            
            self.logger.info("애플리케이션이 성공적으로 종료되었습니다")
            
        except Exception as e:
            self.logger.error(f"애플리케이션 종료 중 오류: {e}")
    
    def run(self):
        """메인 실행 루프"""
        if not self.initialize_components():
            return False
        
        # 시그널 핸들러 설정
        setup_signal_handlers(self.stop)
        
        if not self.start():
            return False
        
        try:
            # 상태 출력 간격 (초)
            status_interval = 30
            last_status_time = time.time()
            
            # 메인 루프
            while self.is_running:
                time.sleep(1)
                
                # 주기적으로 상태 출력
                current_time = time.time()
                if current_time - last_status_time >= status_interval:
                    self._print_status()
                    last_status_time = current_time
                
                # 디스크 공간 체크
                if not check_disk_space(self.config['recording']['output_directory'], 1.0):
                    self.logger.warning("디스크 공간이 부족합니다. 오래된 파일을 정리하세요.")
                    # 자동으로 오래된 파일 정리
                    self.bag_recorder.cleanup_old_files(max_files=50)
        
        except KeyboardInterrupt:
            self.logger.info("사용자에 의해 중단됨")
        except Exception as e:
            self.logger.error(f"메인 루프 오류: {e}")
        finally:
            self.stop()
        
        return True
    
    def _on_message_received(self, message_data: Dict[str, Any]):
        """
        SENSR 메시지 수신 시 호출되는 콜백 함수
        
        Args:
            message_data: 수신된 메시지 데이터
        """
        try:
            current_time = time.time()
            message_type = message_data.get('type')
            
            # 데이터 수집 간격 제어
            should_process = False
            
            if message_type == 'point_cloud':
                if current_time - self.last_pointcloud_time >= self.pointcloud_interval:
                    should_process = True
                    self.last_pointcloud_time = current_time
                else:
                    self.logger.debug(f"포인트클라우드 데이터 스킵 (간격: {current_time - self.last_pointcloud_time:.3f}초)")
                    
            elif message_type == 'output_data':
                if self.pointcloud_only_mode:
                    self.logger.debug("포인트클라우드 전용 모드: output_data 스킵")
                    return
                    
                if current_time - self.last_output_data_time >= self.output_data_interval:
                    should_process = True
                    self.last_output_data_time = current_time
                else:
                    self.logger.debug(f"출력 데이터 스킵 (간격: {current_time - self.last_output_data_time:.3f}초)")
            else:
                # 기타 메시지는 그대로 처리
                should_process = True
            
            if not should_process:
                return
            
            # 빈 데이터 필터링
            if self.skip_empty_data:
                data_size = len(message_data.get('data', b''))
                if data_size < 100:  # 100바이트 이하면 빈 데이터로 간주
                    self.logger.debug(f"빈 데이터 스킵 (크기: {data_size} bytes)")
                    return
            
            self.logger.info(f"메시지 처리! type={message_type}, size={len(message_data.get('data', b''))} bytes")
            
            # 첫 번째 메시지 수신 시 레코딩 시작
            if not self.bag_recorder.is_recording:
                self.logger.info("첫 번째 메시지 수신됨! ROS bag 레코딩을 시작합니다...")
                if not self.bag_recorder.start_recording():
                    self.logger.error("레코딩 시작 실패")
                    return
                else:
                    self.logger.info("ROS bag 레코딩이 시작되었습니다!")
            
            # 데이터 처리 및 ROS 메시지 변환
            processed_messages = self.data_processor.process_message(message_data)
            
            self.logger.debug(f"처리된 메시지 결과: {type(processed_messages)}")
            
            if processed_messages:
                # 단일 메시지인 경우 리스트로 변환
                if not isinstance(processed_messages, list):
                    processed_messages = [processed_messages]
                
                self.logger.info(f"Bag에 기록할 메시지: {len(processed_messages)}개")
                
                # 각 메시지를 bag에 기록
                for i, msg_info in enumerate(processed_messages):
                    if msg_info:
                        self.logger.debug(f"메시지 {i}: topic={msg_info.get('topic')}")
                        self.bag_recorder.write_message(
                            msg_info['topic'],
                            msg_info['message'],
                            msg_info['timestamp']
                        )
                    else:
                        self.logger.warning(f"메시지 {i}가 None입니다")
            else:
                self.logger.warning("처리된 메시지가 없습니다")
        
        except Exception as e:
            self.logger.error(f"메시지 처리 콜백 오류: {e}")
            import traceback
            self.logger.error(f"스택 트레이스: {traceback.format_exc()}")
    
    def _print_status(self):
        """상태 정보 출력"""
        try:
            if self.status_monitor:
                status = self.status_monitor.get_status_summary(
                    self.sensr_client,
                    self.bag_recorder,
                    self.data_processor
                )
                
                self.logger.info("=== 상태 정보 ===")
                self.logger.info(f"실행 시간: {status['uptime']:.1f}초")
                
                if status['sensr_connection']:
                    conn = status['sensr_connection']
                    self.logger.info(f"SENSR 연결: {'연결됨' if conn['is_connected'] else '연결 안됨'}")
                    self.logger.info(f"메시지 큐 크기: {conn['queue_size']}")
                
                if status['recording']:
                    rec = status['recording']
                    self.logger.info(f"레코딩: {'진행 중' if rec['is_recording'] else '정지'}")
                    if rec['current_bag_path']:
                        self.logger.info(f"현재 파일: {os.path.basename(rec['current_bag_path'])}")
                        self.logger.info(f"현재 파일 시간: {rec['current_duration']:.1f}초")
                
                self.logger.info("================")
        
        except Exception as e:
            self.logger.error(f"상태 출력 오류: {e}")



def main():
    """메인 함수"""
    parser = argparse.ArgumentParser(description='SENSR LiDAR Data Recorder')
    parser.add_argument(
        '--config', '-c',
        default='config/config.yaml',
        help='설정 파일 경로 (기본값: config/config.yaml)'
    )
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='상세 로그 출력'
    )
    parser.add_argument(
        '--output-dir', '-o',
        help='출력 디렉토리 경로 (설정 파일 오버라이드)'
    )
    parser.add_argument(
        '--quick-start', '-q',
        action='store_true',
        help='빠른 시작 (연결 대기 시간 단축)'
    )
    parser.add_argument(
        '--pointcloud-interval', '-pi',
        type=float,
        help='포인트클라우드 데이터 수집 간격 (초) - 예: 0.1, 0.5, 1.0'
    )
    parser.add_argument(
        '--output-data-interval', '-oi',
        type=float,
        help='출력 데이터 수집 간격 (초) - 예: 0.1, 0.5, 1.0'
    )
    parser.add_argument(
        '--pointcloud-only',
        action='store_true',
        help='포인트클라우드 데이터만 수집 (출력 데이터 무시)'
    )
    parser.add_argument(
        '--host', '-H',
        help='수집할 SENSR 호스트 ID 또는 IP 주소'
    )
    parser.add_argument(
        '--list-hosts',
        action='store_true',
        help='구성된 호스트 목록을 출력하고 종료'
    )

    args = parser.parse_args()

    config_path = args.config
    if not os.path.isabs(config_path):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, config_path)

    if not os.path.exists(config_path):
        print(f"오류: 설정 파일을 찾을 수 없습니다 - {config_path}")
        sys.exit(1)

    base_config = load_config(config_path)

    if args.list_hosts:
        hosts = list_available_hosts(base_config)
        if not hosts:
            print('구성된 호스트가 없습니다.')
        else:
            print('사용 가능한 호스트:')
            for host in hosts:
                description = f" - {host['description']}" if host.get('description') else ''
                print(f"  {host['id']} ({host['address']}){description}")
        sys.exit(0)

    runtime_config = copy.deepcopy(base_config)

    if args.verbose:
        runtime_config.setdefault('logging', {})['level'] = 'DEBUG'
        print('로그 레벨을 DEBUG로 설정합니다.')

    if args.output_dir:
        runtime_config.setdefault('recording', {})['output_directory'] = args.output_dir
        print(f"출력 디렉토리를 {args.output_dir}로 설정합니다")

    if args.pointcloud_interval is not None:
        runtime_config.setdefault('recording', {})['pointcloud_interval'] = args.pointcloud_interval
        print(f"포인트클라우드 수집 간격을 {args.pointcloud_interval}초로 설정합니다")

    if args.output_data_interval is not None:
        runtime_config.setdefault('recording', {})['output_data_interval'] = args.output_data_interval
        print(f"출력 데이터 수집 간격을 {args.output_data_interval}초로 설정합니다")

    if args.pointcloud_only:
        runtime_config.setdefault('recording', {})['pointcloud_only'] = True
        print('포인트클라우드 전용 모드로 실행합니다')

    host_entry = None
    if args.host:
        if args.host.strip().lower() == 'all':
            hosts = list_available_hosts(runtime_config)
            if not hosts:
                print('구성된 호스트가 없습니다.')
            else:
                print('현재 단일 프로세스에서는 한 번에 하나의 호스트만 수집할 수 있습니다.')
                print('다중 호스트 수집은 각 호스트별로 별도 프로세스를 실행해 주세요:')
                for host in hosts:
                    description = f" - {host['description']}" if host.get('description') else ''
                    print(f"  python main.py --host {host['id']}  # {host['address']}{description}")
            sys.exit(1)

        host_entry = find_host_entry(runtime_config, args.host)
        if not host_entry:
            hosts = list_available_hosts(runtime_config)
            print(f"호스트 '{args.host}'를 찾을 수 없습니다.")
            if hosts:
                print('사용 가능한 호스트:')
                for host in hosts:
                    description = f" - {host['description']}" if host.get('description') else ''
                    print(f"  {host['id']} ({host['address']}){description}")
            sys.exit(1)
        set_active_host(runtime_config, host_entry)
    else:
        host_entry = find_host_entry(runtime_config, None)
        if host_entry:
            set_active_host(runtime_config, host_entry)

    if host_entry:
        description = f" - {host_entry['description']}" if host_entry.get('description') else ''
        print(f"선택된 호스트: {host_entry['id']} ({host_entry['address']}){description}")

    app = SensrRecorderApp(config_path, runtime_config=runtime_config)

    if args.quick_start:
        print('빠른 시작 모드로 실행합니다...')
        app.quick_start = True

    success = app.run()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
