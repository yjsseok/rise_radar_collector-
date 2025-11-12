#!/usr/bin/env python3
"""
SENSR WebSocket Client Module
Seoul Robotics SENSR 시스템과의 WebSocket 연결을 관리하는 클라이언트
"""

import websocket
import threading
import time
import logging
import queue
from typing import Callable, Optional, Dict, Any
import json


class SensrClient:
    """SENSR 서버와의 WebSocket 연결을 관리하는 클라이언트 클래스"""
    
    def __init__(self, config: Dict[str, Any], message_callback: Optional[Callable] = None):
        """
        SensrClient 초기화
        
        Args:
            config: 설정 딕셔너리
            message_callback: 메시지 수신 시 호출될 콜백 함수
        """
        self.config = config
        self.host = config['sensr']['host']
        self.ports = config['sensr']['ports']
        self.reconnect_interval = config['sensr']['reconnect_interval']
        
        self.message_callback = message_callback
        self.message_queue = queue.Queue()
        
        # WebSocket 연결 객체들
        self.ws_output = None
        self.ws_pointcloud = None
        
        # 연결 상태 관리
        self.is_connected = False
        self.should_reconnect = True
        self.connection_threads = {}
        
        # 로깅 설정
        self.logger = logging.getLogger(__name__)
        
    def connect(self) -> bool:
        """
        SENSR 서버에 연결
        
        Returns:
            bool: 연결 성공 여부
        """
        try:
            self.logger.info(f"SENSR 서버에 연결 중... {self.host}")
            
            # Output data WebSocket 연결
            output_url = f"ws://{self.host}:{self.ports['output_data']}"
            self.ws_output = websocket.WebSocketApp(
                output_url,
                on_message=self._on_output_message,
                on_error=self._on_error,
                on_close=self._on_close,
                on_open=self._on_open
            )
            
            # Point cloud WebSocket 연결
            pointcloud_url = f"ws://{self.host}:{self.ports['point_cloud']}"
            self.ws_pointcloud = websocket.WebSocketApp(
                pointcloud_url,
                on_message=self._on_pointcloud_message,
                on_error=self._on_error,
                on_close=self._on_close,
                on_open=self._on_open
            )
            
            # 연결 테스트 (빠른 확인)
            self.logger.info("연결 테스트 중...")
            if self._test_connection():
                self.logger.info("연결 테스트 성공")
                return True
            else:
                self.logger.warning("연결 테스트 실패, 하지만 계속 진행합니다")
                return True
            
        except Exception as e:
            self.logger.error(f"연결 실패: {e}")
            return False
    
    def disconnect(self):
        """SENSR 서버와의 연결 종료"""
        self.should_reconnect = False
        self.is_connected = False
        
        if self.ws_output:
            self.ws_output.close()
        if self.ws_pointcloud:
            self.ws_pointcloud.close()
            
        self.logger.info("SENSR 서버와의 연결이 종료되었습니다.")
    
    def start_listening(self):
        """데이터 수신을 위한 스레드 시작"""
        self.should_reconnect = True
        
        # Output data 수신 스레드
        output_thread = threading.Thread(
            target=self._run_websocket,
            args=(self.ws_output, "output"),
            daemon=True
        )
        output_thread.start()
        self.connection_threads["output"] = output_thread
        
        # Point cloud 수신 스레드
        pointcloud_thread = threading.Thread(
            target=self._run_websocket,
            args=(self.ws_pointcloud, "pointcloud"),
            daemon=True
        )
        pointcloud_thread.start()
        self.connection_threads["pointcloud"] = pointcloud_thread
        
        # 연결이 성공할 때까지 잠시 대기
        self.logger.info("데이터 수신 스레드가 시작되었습니다.")
        self.logger.info("연결 대기 중... (최대 10초)")
        
        # 연결 상태 확인
        for i in range(10):
            if self.is_connected:
                self.logger.info(f"연결 성공! ({i+1}초 소요)")
                break
            time.sleep(1)
            if i == 4:
                self.logger.info("연결에 시간이 걸리고 있습니다. 계속 시도 중...")
        
        if not self.is_connected:
            self.logger.warning("초기 연결에 실패했지만 백그라운드에서 계속 시도합니다.")
    
    def stop_listening(self):
        """데이터 수신 스레드 정지"""
        self.should_reconnect = False
        self.disconnect()
        
        # 스레드 종료 대기
        for thread_name, thread in self.connection_threads.items():
            if thread.is_alive():
                self.logger.info(f"{thread_name} 스레드 종료 대기 중...")
                thread.join(timeout=5)
        
        self.logger.info("데이터 수신 스레드가 정지되었습니다.")
    
    def _run_websocket(self, ws: websocket.WebSocketApp, connection_type: str):
        """WebSocket 실행 및 재연결 관리"""
        while self.should_reconnect:
            try:
                self.logger.info(f"{connection_type} WebSocket 연결 시작")
                
                # 포트 확인 로그
                port = self.ports['output_data'] if connection_type == "output" else self.ports['point_cloud']
                self.logger.info(f"{connection_type} WebSocket URL: ws://{self.host}:{port}")
                
                # 연결 옵션 설정
                ws.run_forever(
                    ping_interval=30,  # 30초마다 ping
                    ping_timeout=10,   # ping 응답 타임아웃 10초
                )
                
            except Exception as e:
                self.logger.error(f"{connection_type} WebSocket 오류: {e}")
                
            if self.should_reconnect:
                self.logger.info(f"{self.reconnect_interval}초 후 {connection_type} 재연결 시도...")
                time.sleep(self.reconnect_interval)
    
    def _on_open(self, ws):
        """WebSocket 연결 성공 시 호출"""
        self.is_connected = True
        
        # 어떤 WebSocket이 연결되었는지 확인
        if ws == self.ws_output:
            self.logger.info("Output WebSocket (포트 5050) 연결 성공")
        elif ws == self.ws_pointcloud:
            self.logger.info("포인트클라우드 WebSocket (포트 5051) 연결 성공")
        else:
            self.logger.info("WebSocket 연결 성공 (알 수 없는 연결)")
    
    def _on_output_message(self, ws, message):
        """Output data 메시지 수신 시 호출"""
        try:
            self.logger.info(f"Output 데이터 수신: {len(message)} bytes")
            self._process_message(message, "output_data")
        except Exception as e:
            self.logger.error(f"Output 메시지 처리 오류: {e}")
    
    def _on_pointcloud_message(self, ws, message):
        """Point cloud 메시지 수신 시 호출"""
        try:
            self.logger.info(f"포인트클라우드 데이터 수신: {len(message)} bytes")
            self._process_message(message, "point_cloud")
        except Exception as e:
            self.logger.error(f"Point cloud 메시지 처리 오류: {e}")
    
    def _process_message(self, message, data_type: str):
        """수신된 메시지 처리"""
        # 메시지를 큐에 추가
        message_data = {
            'data': message,
            'type': data_type,
            'timestamp': time.time()
        }
        
        try:
            self.message_queue.put_nowait(message_data)
        except queue.Full:
            self.logger.warning("메시지 큐가 가득참. 오래된 메시지를 제거합니다.")
            try:
                self.message_queue.get_nowait()
                self.message_queue.put_nowait(message_data)
            except queue.Empty:
                pass
        
        # 콜백 함수 호출
        if self.message_callback:
            try:
                self.message_callback(message_data)
            except Exception as e:
                self.logger.error(f"메시지 콜백 처리 오류: {e}")
    
    def _on_error(self, ws, error):
        """WebSocket 오류 발생 시 호출"""
        self.logger.error(f"WebSocket 오류: {error}")
        self.is_connected = False
    
    def _on_close(self, ws, close_status_code=None, close_msg=None):
        """WebSocket 연결 종료 시 호출"""
        self.is_connected = False
        self.logger.info(f"WebSocket 연결이 종료됨. 상태코드: {close_status_code}, 메시지: {close_msg}")
    
    def get_message(self, timeout: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """
        메시지 큐에서 메시지 가져오기
        
        Args:
            timeout: 타임아웃 (초)
            
        Returns:
            메시지 딕셔너리 또는 None
        """
        try:
            return self.message_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def get_connection_status(self) -> Dict[str, Any]:
        """
        연결 상태 정보 반환
        
        Returns:
            연결 상태 딕셔너리
        """
        return {
            'is_connected': self.is_connected,
            'should_reconnect': self.should_reconnect,
            'queue_size': self.message_queue.qsize(),
            'active_threads': len([t for t in self.connection_threads.values() if t.is_alive()])
        }
    
    def _test_connection(self) -> bool:
        """
        빠른 연결 테스트
        
        Returns:
            bool: 연결 가능 여부
        """
        import socket
        
        try:
            # TCP 연결 테스트 (빠른 포트 확인)
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3)  # 3초 타임아웃
            
            # Output data 포트 테스트
            result1 = sock.connect_ex((self.host, self.ports['output_data']))
            sock.close()
            
            # Point cloud 포트 테스트
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3)
            result2 = sock.connect_ex((self.host, self.ports['point_cloud']))
            sock.close()
            
            return result1 == 0 or result2 == 0
            
        except Exception as e:
            self.logger.debug(f"연결 테스트 오류: {e}")
            return False