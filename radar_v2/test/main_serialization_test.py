#!/usr/bin/env python3
"""
직렬화 메모리 프로파일링 테스트
serialize_message()가 메모리 누수의 원인인지 확인
"""

import os
import sys
import time
import gc
import psutil
import argparse
import logging
from datetime import datetime

# 상위 디렉토리의 모듈 import
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import rclpy
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("❌ ROS2를 찾을 수 없습니다.")
    sys.exit(1)

from src.utils import load_config
from src.sensr_client import SensrClient
from src.data_processor import DataProcessor


class SerializationMemoryTest:
    """직렬화 메모리 테스트"""

    def __init__(self, config):
        self.config = config
        self.logger = logging.getLogger(__name__)
        self.process = psutil.Process()

        # 통계
        self.stats = {
            'total_messages': 0,
            'total_serialized': 0,
            'pointcloud_count': 0,
            'output_data_count': 0,
            'memory_before_serialize': [],
            'memory_after_serialize': [],
            'memory_after_delete': [],
        }

    def get_memory_mb(self):
        """현재 메모리 사용량 (MB)"""
        return self.process.memory_info().rss / 1024 / 1024

    def create_dummy_pointcloud(self, num_points=10000):
        """테스트용 포인트클라우드 생성"""
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.header.frame_id = "sensr"

        # 필드 정의 (x, y, z, intensity)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        msg.width = num_points
        msg.height = 1
        msg.is_bigendian = False
        msg.point_step = 16  # 4 fields * 4 bytes
        msg.row_step = msg.point_step * num_points
        msg.is_dense = True

        # 더미 데이터 생성
        msg.data = b'\x00' * (msg.point_step * num_points)

        return msg

    def test_serialize_memory_leak(self, num_iterations=100, num_points=10000):
        """
        직렬화 메모리 누수 테스트

        Args:
            num_iterations: 반복 횟수
            num_points: 포인트클라우드 점 개수
        """
        print(f"\n{'='*70}")
        print(f"🔬 직렬화 메모리 누수 테스트")
        print(f"{'='*70}")
        print(f"반복 횟수: {num_iterations}")
        print(f"포인트 개수: {num_points:,}")
        print(f"{'='*70}\n")

        # 초기 메모리
        gc.collect()
        time.sleep(1)
        initial_memory = self.get_memory_mb()
        print(f"📊 초기 메모리: {initial_memory:.1f} MB\n")

        for i in range(num_iterations):
            # 1. 메시지 생성 전 메모리
            mem_before = self.get_memory_mb()

            # 2. 포인트클라우드 메시지 생성
            msg = self.create_dummy_pointcloud(num_points)

            # 3. 직렬화 전 메모리
            mem_before_serialize = self.get_memory_mb()
            self.stats['memory_before_serialize'].append(mem_before_serialize)

            # 4. 직렬화 수행
            serialized = serialize_message(msg)

            # 5. 직렬화 후 메모리
            mem_after_serialize = self.get_memory_mb()
            self.stats['memory_after_serialize'].append(mem_after_serialize)

            # 6. 명시적 삭제
            del serialized
            del msg

            # 7. 삭제 후 메모리
            mem_after_delete = self.get_memory_mb()
            self.stats['memory_after_delete'].append(mem_after_delete)

            self.stats['total_messages'] += 1
            self.stats['total_serialized'] += 1

            # 10회마다 리포트
            if (i + 1) % 10 == 0:
                current_memory = self.get_memory_mb()
                increase = current_memory - initial_memory
                rate = increase / (i + 1)

                print(f"[{i+1:3d}] 메모리: {current_memory:6.1f} MB | "
                      f"증가: {increase:6.1f} MB | "
                      f"평균: {rate:5.2f} MB/iter | "
                      f"직렬화 차이: {mem_after_serialize - mem_before_serialize:5.2f} MB | "
                      f"삭제 차이: {mem_after_delete - mem_after_serialize:5.2f} MB")

            # 30회마다 GC
            if (i + 1) % 30 == 0:
                collected = gc.collect()
                print(f"      🗑️  GC 실행: {collected}개 객체 수집")

        # 최종 메모리
        gc.collect()
        time.sleep(1)
        final_memory = self.get_memory_mb()
        total_increase = final_memory - initial_memory

        print(f"\n{'='*70}")
        print(f"📊 최종 결과")
        print(f"{'='*70}")
        print(f"초기 메모리:   {initial_memory:.1f} MB")
        print(f"최종 메모리:   {final_memory:.1f} MB")
        print(f"총 증가량:     {total_increase:.1f} MB")
        print(f"평균 증가율:   {total_increase/num_iterations:.2f} MB/iteration")
        print(f"{'='*70}\n")

        return total_increase

    def test_with_real_data(self, duration=300, host_id="samyang"):
        """
        실제 SENSR 데이터로 테스트

        Args:
            duration: 테스트 시간 (초)
            host_id: SENSR 호스트 ID
        """
        print(f"\n{'='*70}")
        print(f"🔬 실제 데이터 직렬화 메모리 테스트")
        print(f"{'='*70}")
        print(f"호스트: {host_id}")
        print(f"시간: {duration}초")
        print(f"{'='*70}\n")

        # SENSR 클라이언트 생성
        data_processor = DataProcessor(self.config)
        client = SensrClient(self.config, data_processor, host_id=host_id)

        # 메모리 추적
        memory_samples = []
        serialize_times = []

        def on_message_callback(topic, message, timestamp):
            """메시지 수신 콜백"""
            # 직렬화 전 메모리
            mem_before = self.get_memory_mb()

            # 직렬화 수행
            start = time.time()
            serialized = serialize_message(message)
            serialize_time = time.time() - start

            # 직렬화 후 메모리
            mem_after = self.get_memory_mb()

            # 명시적 삭제
            del serialized

            # 삭제 후 메모리
            mem_after_del = self.get_memory_mb()

            # 통계 수집
            self.stats['total_messages'] += 1
            self.stats['total_serialized'] += 1

            if 'pointcloud' in topic:
                self.stats['pointcloud_count'] += 1
            else:
                self.stats['output_data_count'] += 1

            self.stats['memory_before_serialize'].append(mem_before)
            self.stats['memory_after_serialize'].append(mem_after)
            self.stats['memory_after_delete'].append(mem_after_del)
            serialize_times.append(serialize_time)

        # 메시지 핸들러 교체
        original_handler = client.message_handler
        client.message_handler = lambda msg: None  # 무시

        # 연결 및 시작
        if not client.connect():
            print("❌ SENSR 서버 연결 실패")
            return

        client.start()

        # 초기 메모리
        gc.collect()
        time.sleep(2)
        initial_memory = self.get_memory_mb()
        print(f"📊 초기 메모리: {initial_memory:.1f} MB\n")

        start_time = time.time()
        last_report_time = start_time

        try:
            while time.time() - start_time < duration:
                time.sleep(0.1)

                # 5초마다 리포트
                if time.time() - last_report_time >= 5:
                    current_memory = self.get_memory_mb()
                    elapsed = time.time() - start_time
                    increase = current_memory - initial_memory
                    rate = increase / (elapsed / 60)  # MB/분

                    print(f"⏱️  {elapsed:5.1f}초 | "
                          f"메모리: {current_memory:6.1f} MB | "
                          f"증가: {increase:6.1f} MB | "
                          f"증가율: {rate:5.1f} MB/분 | "
                          f"메시지: {self.stats['total_messages']}")

                    memory_samples.append({
                        'time': elapsed,
                        'memory': current_memory,
                        'increase': increase
                    })

                    last_report_time = time.time()

                    # GC 실행
                    if int(elapsed) % 30 == 0:
                        collected = gc.collect()
                        print(f"      🗑️  GC 실행: {collected}개 객체 수집")

        except KeyboardInterrupt:
            print("\n⚠️  사용자 중단")

        finally:
            client.stop()

        # 최종 통계
        gc.collect()
        time.sleep(1)
        final_memory = self.get_memory_mb()
        total_increase = final_memory - initial_memory
        elapsed_minutes = (time.time() - start_time) / 60

        print(f"\n{'='*70}")
        print(f"📊 최종 결과")
        print(f"{'='*70}")
        print(f"총 실행 시간:   {elapsed_minutes:.1f}분")
        print(f"초기 메모리:     {initial_memory:.1f} MB")
        print(f"최종 메모리:     {final_memory:.1f} MB")
        print(f"총 증가량:       {total_increase:.1f} MB")
        print(f"분당 증가율:     {total_increase/elapsed_minutes:.1f} MB/분")
        print(f"\n메시지 통계:")
        print(f"  - 총 메시지:   {self.stats['total_messages']}")
        print(f"  - 직렬화:      {self.stats['total_serialized']}")
        print(f"  - 포인트클라우드: {self.stats['pointcloud_count']}")
        print(f"  - Output Data: {self.stats['output_data_count']}")

        if serialize_times:
            print(f"\n직렬화 시간:")
            print(f"  - 평균: {sum(serialize_times)/len(serialize_times)*1000:.2f} ms")
            print(f"  - 최대: {max(serialize_times)*1000:.2f} ms")

        print(f"{'='*70}\n")

        return total_increase


def main():
    parser = argparse.ArgumentParser(description='직렬화 메모리 프로파일링 테스트')
    parser.add_argument('--test-type', choices=['dummy', 'real'], default='dummy',
                       help='테스트 타입: dummy(더미 데이터) 또는 real(실제 데이터)')
    parser.add_argument('--iterations', type=int, default=100,
                       help='더미 테스트 반복 횟수 (기본: 100)')
    parser.add_argument('--points', type=int, default=10000,
                       help='포인트클라우드 점 개수 (기본: 10000)')
    parser.add_argument('--duration', type=int, default=300,
                       help='실제 데이터 테스트 시간 (초, 기본: 300)')
    parser.add_argument('--host', type=str, default='samyang',
                       help='SENSR 호스트 ID (기본: samyang)')

    args = parser.parse_args()

    # 로깅 설정
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # ROS2 초기화
    if not rclpy.ok():
        rclpy.init()

    # 설정 로드
    config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'config.yaml')
    config = load_config(config_path)

    # 테스트 실행
    tester = SerializationMemoryTest(config)

    if args.test_type == 'dummy':
        print(f"\n🧪 더미 데이터 테스트 시작")
        tester.test_serialize_memory_leak(args.iterations, args.points)
    else:
        print(f"\n🧪 실제 데이터 테스트 시작")
        tester.test_with_real_data(args.duration, args.host)

    print("\n✅ 테스트 완료!")


if __name__ == '__main__':
    main()
