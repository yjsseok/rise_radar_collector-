#!/usr/bin/env python3
"""
SENSR 연결 테스트 스크립트
Seoul Robotics SENSR 시스템과의 연결을 테스트하는 스크립트
"""

import sys
import os
import time

# 프로젝트 모듈 경로 추가
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from src.sensr_client import SensrClient
from src.utils import load_config, validate_config


def test_sensr_connection(config_path='config/config.yaml'):
    """SENSR 서버 연결 테스트"""
    
    print("=== SENSR 연결 테스트 ===")
    
    try:
        # 설정 파일 로드
        print(f"1. 설정 파일 로드 중: {config_path}")
        if not os.path.exists(config_path):
            print(f"❌ 설정 파일을 찾을 수 없습니다: {config_path}")
            return False
        
        config = load_config(config_path)
        print("✅ 설정 파일 로드 성공")
        
        # 설정 유효성 검사
        print("2. 설정 유효성 검사 중...")
        if not validate_config(config):
            print("❌ 설정 파일 유효성 검사 실패")
            return False
        print("✅ 설정 유효성 검사 통과")
        
        # SENSR 클라이언트 생성
        print("3. SENSR 클라이언트 초기화 중...")
        client = SensrClient(config)
        print("✅ SENSR 클라이언트 초기화 성공")
        
        # 서버 연결 테스트
        print(f"4. SENSR 서버 연결 테스트 중...")
        print(f"   - 호스트: {config['sensr']['host']}")
        print(f"   - 출력 데이터 포트: {config['sensr']['ports']['output_data']}")
        print(f"   - 포인트클라우드 포트: {config['sensr']['ports']['point_cloud']}")
        
        if not client.connect():
            print("❌ SENSR 서버 연결 실패")
            return False
        
        print("✅ SENSR 서버 연결 성공")
        
        # 데이터 수신 테스트
        print("5. 데이터 수신 테스트 중... (10초간)")
        client.start_listening()
        
        message_count = 0
        start_time = time.time()
        test_duration = 10  # 10초간 테스트
        
        while time.time() - start_time < test_duration:
            message = client.get_message(timeout=1.0)
            if message:
                message_count += 1
                print(f"   📨 메시지 수신: {message['type']} (총 {message_count}개)")
            time.sleep(0.1)
        
        # 연결 종료
        print("6. 연결 종료 중...")
        client.stop_listening()
        print("✅ 연결 종료 완료")
        
        # 결과 출력
        print("\n=== 테스트 결과 ===")
        print(f"✅ 설정 파일: 정상")
        print(f"✅ 서버 연결: 성공")
        print(f"✅ 데이터 수신: {message_count}개 메시지 수신")
        
        if message_count > 0:
            print("🎉 SENSR 연결 테스트 성공!")
            return True
        else:
            print("⚠️  연결은 되었지만 데이터를 수신하지 못했습니다.")
            print("   - 서버에서 데이터를 전송하고 있는지 확인하세요")
            print("   - 네트워크 방화벽 설정을 확인하세요")
            return False
            
    except ImportError as e:
        print(f"❌ 모듈 import 오류: {e}")
        print("필요한 패키지가 설치되어 있는지 확인하세요:")
        print("pip3 install -r requirements.txt")
        return False
        
    except Exception as e:
        print(f"❌ 테스트 중 오류 발생: {e}")
        return False


def test_ros_environment():
    """ROS 환경 테스트"""
    
    print("\n=== ROS 환경 테스트 ===")
    
    try:
        import rospy
        import rosbag
        from sensor_msgs.msg import PointCloud2
        from visualization_msgs.msg import MarkerArray
        from std_msgs.msg import String
        
        print("✅ ROS 패키지 import 성공")
        
        # ROS 환경 변수 확인
        ros_distro = os.environ.get('ROS_DISTRO', 'None')
        print(f"✅ ROS 배포판: {ros_distro}")
        
        return True
        
    except ImportError as e:
        print(f"❌ ROS 패키지 import 실패: {e}")
        print("ROS 환경을 설정하세요:")
        print("source /opt/ros/noetic/setup.bash")
        return False


def main():
    """메인 함수"""
    
    print("Seoul Robotics SENSR 연결 테스트 도구")
    print("=" * 50)
    
    # ROS 환경 테스트
    ros_ok = test_ros_environment()
    
    # SENSR 연결 테스트
    config_path = 'config/config.yaml'
    if len(sys.argv) > 1:
        config_path = sys.argv[1]
    
    sensr_ok = test_sensr_connection(config_path)
    
    # 최종 결과
    print("\n" + "=" * 50)
    print("최종 테스트 결과:")
    print(f"ROS 환경: {'✅ 정상' if ros_ok else '❌ 오류'}")
    print(f"SENSR 연결: {'✅ 정상' if sensr_ok else '❌ 오류'}")
    
    if ros_ok and sensr_ok:
        print("\n🎉 모든 테스트 통과! 프로그램 실행 준비 완료")
        return True
    else:
        print("\n❌ 일부 테스트 실패. 문제를 해결한 후 다시 시도하세요")
        return False


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)