#!/usr/bin/env python3
"""
Utility Functions Module
프로젝트에서 사용되는 유틸리티 함수들
"""

import os
import yaml
import logging
import signal
import sys
from typing import Dict, Any, Optional, List
from datetime import datetime


def load_config(config_path: str) -> Dict[str, Any]:
    """
    YAML 설정 파일 로드
    
    Args:
        config_path: 설정 파일 경로
        
    Returns:
        설정 딕셔너리
        
    Raises:
        FileNotFoundError: 설정 파일이 없을 때
        yaml.YAMLError: YAML 파싱 오류
    """
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"설정 파일을 찾을 수 없습니다: {config_path}")
    
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        return config
    except yaml.YAMLError as e:
        raise yaml.YAMLError(f"설정 파일 파싱 오류: {e}")


def list_available_hosts(config: Dict[str, Any]) -> List[Dict[str, str]]:
    """Return a normalized list of configured SENSR host endpoints."""
    sensr_cfg = config.get('sensr') or {}
    hosts: List[Dict[str, str]] = []

    raw_hosts = sensr_cfg.get('hosts') or []
    for entry in raw_hosts:
        if not isinstance(entry, dict):
            continue

        address = entry.get('address')
        if not address:
            continue

        host_id = str(entry.get('id') or address)
        hosts.append({
            'id': host_id,
            'address': str(address),
            'description': str(entry.get('description', '') or '').strip()
        })

    default_address = sensr_cfg.get('host')
    default_label = str(sensr_cfg.get('active_host_label') or 'default')
    if default_address and not any(h['address'] == default_address for h in hosts):
        hosts.insert(0, {
            'id': default_label,
            'address': str(default_address),
            'description': str(sensr_cfg.get('description', '') or '').strip()
        })

    return hosts


def find_host_entry(config: Dict[str, Any], selector: Optional[str]) -> Optional[Dict[str, str]]:
    """Resolve a host entry by id, description, or direct IP/hostname."""
    hosts = list_available_hosts(config)
    if not hosts:
        return None

    if not selector:
        sensr_cfg = config.get('sensr') or {}
        current_label = sensr_cfg.get('active_host_label')
        if current_label:
            for host in hosts:
                if host['id'].lower() == str(current_label).lower():
                    return host

        current_address = sensr_cfg.get('host')
        if current_address:
            for host in hosts:
                if host['address'] == str(current_address):
                    return host

        return hosts[0]

    normalized = selector.strip().lower()
    for host in hosts:
        if host['id'].lower() == normalized or host['address'] == selector:
            return host

        description = host.get('description')
        if description and description.lower() == normalized:
            return host

    return None


def set_active_host(config: Dict[str, Any], host_entry: Dict[str, str]) -> Dict[str, Any]:
    """Persist the resolved host selection back into the config dict."""
    sensr_cfg = config.setdefault('sensr', {})
    sensr_cfg['host'] = host_entry['address']
    sensr_cfg['active_host_label'] = host_entry['id']
    return config


def setup_logging(config: Dict[str, Any]) -> logging.Logger:
    """
    로깅 설정
    
    Args:
        config: 설정 딕셔너리
        
    Returns:
        설정된 로거
    """
    logging_config = config.get('logging', {})
    log_level = logging_config.get('level', 'INFO')
    log_file = logging_config.get('file', './logs/sensr_recorder.log')
    
    # 로그 디렉토리 생성
    log_dir = os.path.dirname(log_file)
    if log_dir and not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    # 로그 레벨 설정
    numeric_level = getattr(logging, log_level.upper(), logging.INFO)
    
    # 로깅 포맷 설정
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # 루트 로거 설정
    logger = logging.getLogger()
    logger.setLevel(numeric_level)
    
    # 콘솔 핸들러
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(numeric_level)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    # 파일 핸들러
    file_handler = logging.FileHandler(log_file, encoding='utf-8')
    file_handler.setLevel(numeric_level)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    
    return logger


def validate_config(config: Dict[str, Any]) -> bool:
    """
    설정 파일 유효성 검사
    
    Args:
        config: 설정 딕셔너리
        
    Returns:
        유효성 검사 결과
    """
    required_keys = [
        'sensr.host',
        'sensr.ports.output_data',
        'sensr.ports.point_cloud',
        'recording.duration',
        'recording.output_directory',
        'ros.topics.pointcloud',
        'ros.topics.objects',
        'ros.topics.events'
    ]
    
    def get_nested_value(d: Dict[str, Any], key: str) -> Any:
        """중첩된 딕셔너리에서 값 가져오기"""
        keys = key.split('.')
        value = d
        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                return None
        return value
    
    for key in required_keys:
        if get_nested_value(config, key) is None:
            print(f"설정 오류: 필수 키가 없습니다 - {key}")
            return False
    
    # 포트 번호 유효성 검사
    ports = config['sensr']['ports']
    for port_name, port_value in ports.items():
        if not isinstance(port_value, int) or not (1 <= port_value <= 65535):
            print(f"설정 오류: 잘못된 포트 번호 - {port_name}: {port_value}")
            return False
    
    # 레코딩 시간 유효성 검사
    duration = config['recording']['duration']
    if not isinstance(duration, (int, float)) or duration <= 0:
        print(f"설정 오류: 잘못된 레코딩 시간 - {duration}")
        return False
    
    return True


def create_signal_handler(cleanup_func, shutdown_event=None):
    """
    🚀 v2.1.0: 시그널 핸들러 생성 (Graceful shutdown - multiprocessing 지원)

    Args:
        cleanup_func: 정리 작업을 수행할 함수
        shutdown_event: multiprocessing.Event 또는 None

    Returns:
        시그널 핸들러 함수
    """
    signal_count = [0]  # CTRL+C 카운트
    import time
    first_signal_time = [0]

    def signal_handler(sig, frame):
        if signal_received[0]:
            print("\n이미 종료 중입니다. 잠시만 기다려주세요...")
            return

        signal_received[0] = True
        print(f"\n시그널 {sig} 수신. 프로그램을 정상 종료합니다...")

        # 🚀 v2.1.2: shutdown_event만 설정, cleanup은 finally 블록에서
        if shutdown_event:
            try:
                shutdown_event.set()
            except:
                pass

        # 🚀 v2.1.2: cleanup은 signal handler에서 호출하지 않음
        # finally 블록에서 안전하게 처리되도록 변경

    return signal_handler


def setup_signal_handlers(cleanup_func, shutdown_event=None):
    """
    🚀 v2.1.0: 시그널 핸들러 설정 (multiprocessing 지원)

    Args:
        cleanup_func: 정리 작업을 수행할 함수
        shutdown_event: multiprocessing.Event 또는 None
    """
    handler = create_signal_handler(cleanup_func, shutdown_event)

    # SIGINT (Ctrl+C) 처리
    signal.signal(signal.SIGINT, handler)

    # SIGTERM 처리 (Linux/Unix, Windows에서는 무시)
    try:
        if hasattr(signal, 'SIGTERM'):
            signal.signal(signal.SIGTERM, handler)
    except (OSError, ValueError):
        # Windows에서는 SIGTERM이 지원되지 않을 수 있음
        pass


def format_bytes(bytes_value: int) -> str:
    """
    바이트 크기를 읽기 쉬운 형태로 포맷
    
    Args:
        bytes_value: 바이트 수
        
    Returns:
        포맷된 문자열
    """
    for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
        if bytes_value < 1024.0:
            return f"{bytes_value:.1f} {unit}"
        bytes_value /= 1024.0
    return f"{bytes_value:.1f} PB"


def format_duration(seconds: float) -> str:
    """
    초 단위 시간을 읽기 쉬운 형태로 포맷
    
    Args:
        seconds: 초 단위 시간
        
    Returns:
        포맷된 시간 문자열
    """
    if seconds < 60:
        return f"{seconds:.1f}초"
    elif seconds < 3600:
        minutes = seconds / 60
        return f"{minutes:.1f}분"
    else:
        hours = seconds / 3600
        return f"{hours:.1f}시간"


def get_disk_usage(path: str) -> Dict[str, int]:
    """
    디스크 사용량 정보 가져오기
    
    Args:
        path: 경로
        
    Returns:
        디스크 사용량 정보 (total, used, free)
    """
    try:
        import shutil
        total, used, free = shutil.disk_usage(path)
        return {
            'total': total,
            'used': used,
            'free': free
        }
    except Exception:
        # 기본값 반환
        return {
            'total': 1024**4,  # 1TB
            'used': 0,
            'free': 1024**4
        }


def check_disk_space(path: str, min_free_gb: float = 1.0) -> bool:
    """
    충분한 디스크 공간이 있는지 확인
    
    Args:
        path: 확인할 경로
        min_free_gb: 최소 필요 공간 (GB)
        
    Returns:
        충분한 공간이 있으면 True
    """
    try:
        disk_info = get_disk_usage(path)
        free_gb = disk_info['free'] / (1024 ** 3)
        return free_gb >= min_free_gb
    except Exception:
        return True  # 확인할 수 없으면 True 반환


def create_timestamped_filename(prefix: str, extension: str) -> str:
    """
    타임스탬프가 포함된 파일명 생성
    
    Args:
        prefix: 파일명 접두사
        extension: 파일 확장자 (.bag 등)
        
    Returns:
        타임스탬프가 포함된 파일명
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{prefix}_{timestamp}.{extension.lstrip('.')}"


class StatusMonitor:
    """시스템 상태 모니터링 클래스"""
    
    def __init__(self):
        self.start_time = datetime.now()
    
    def get_uptime(self) -> float:
        """프로그램 실행 시간 반환 (초)"""
        return (datetime.now() - self.start_time).total_seconds()
    
    def get_status_summary(self, sensr_client, bag_recorder, data_processor) -> Dict[str, Any]:
        """전체 시스템 상태 요약"""
        return {
            'uptime': self.get_uptime(),
            'start_time': self.start_time.isoformat(),
            'sensr_connection': sensr_client.get_connection_status() if sensr_client else None,
            'recording': bag_recorder.get_status() if bag_recorder else None,
            'message_queue': {
                'sensr_queue_size': sensr_client.message_queue.qsize() if sensr_client else 0,
                'recording_queue_size': bag_recorder.message_queue.qsize() if bag_recorder else 0
            }
        }