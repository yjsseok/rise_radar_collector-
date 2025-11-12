#!/usr/bin/env python3
"""
SENSR 포인트클라우드 스트리밍 활성화 스크립트 (공식 REST API 사용)
공식 문서: publish_level_point_cloud = 2 설정 필요
"""
import requests
import sys
import json
import argparse

def get_sensr_version(sensr_host="112.133.37.122", rest_port=9080):
    """SENSR 버전 자동 감지"""
    base_url = f"http://{sensr_host}:{rest_port}"
    
    # 일반적인 SENSR 버전들 시도
    versions = ["3.4","3.5","4.0", "3.0", "2.0", "latest"]
    
    for version in versions:
        try:
            test_url = f"{base_url}/{version}/settings/parameters/common?config-key=publish_level_point_cloud"
            response = requests.get(test_url, timeout=3)
            if response.status_code == 200:
                print(f"✅ SENSR 버전 감지: {version}")
                return version
        except:
            continue
    
    print("❌ SENSR 버전을 자동 감지할 수 없습니다. v4.0을 기본값으로 사용합니다.")
    return "v4.0"


def get_pointcloud_config(sensr_host="112.133.37.122", rest_port=9080, version=None):
    """
    현재 포인트클라우드 설정 확인 (공식 REST API)
    
    Args:
        sensr_host: SENSR 서버 IP
        rest_port: REST API 포트 (기본값: 9080)
        version: SENSR 버전
    """
    
    if not version:
        version = get_sensr_version(sensr_host, rest_port)
    
    base_url = f"http://{sensr_host}:{rest_port}"
    
    try:
        # 공식 API로 설정 조회
        config_url = f"{base_url}/{version}/settings/parameters/common"
        
        configs_to_check = [
            "publish_level_point_cloud",
            "point_cloud_update_interval", 
            "enable_point_cloud_bandwidth_reduction",
            "downsampling_resolution"
        ]
        
        print("현재 SENSR 포인트클라우드 설정:")
        print("-" * 50)
        
        current_configs = {}
        
        for config_key in configs_to_check:
            try:
                response = requests.get(
                    f"{config_url}?config-key={config_key}",
                    timeout=5
                )
                
                if response.status_code == 200:
                    value = response.text.strip().strip('"')
                    current_configs[config_key] = value
                    print(f"✅ {config_key}: {value}")
                else:
                    print(f"❌ {config_key}: 조회 실패 (HTTP {response.status_code})")
                    
            except Exception as e:
                print(f"❌ {config_key}: 오류 ({e})")
        
        # 포인트클라우드 활성화 상태 확인
        publish_level = current_configs.get("publish_level_point_cloud", "0")
        
        print("-" * 50)
        if publish_level == "2":
            print("✅ 포인트클라우드 스트리밍이 활성화되어 있습니다!")
        else:
            print("❌ 포인트클라우드 스트리밍이 비활성화되어 있습니다")
            print("   publish_level_point_cloud를 2로 설정해야 합니다")
            
        return current_configs, version
        
    except requests.exceptions.ConnectionError:
        print(f"❌ SENSR 서버 연결 실패: {sensr_host}:{rest_port}")
        return None, None
        
    except Exception as e:
        print(f"❌ 설정 조회 오류: {e}")
        return None, None


def set_pointcloud_config(sensr_host="112.133.37.122", rest_port=9080, version=None):
    """
    포인트클라우드 스트리밍 활성화 (공식 REST API)
    
    Args:
        sensr_host: SENSR 서버 IP
        rest_port: REST API 포트 (기본값: 9080)
        version: SENSR 버전
    """
    
    if not version:
        version = get_sensr_version(sensr_host, rest_port)
    
    base_url = f"http://{sensr_host}:{rest_port}"
    config_url = f"{base_url}/{version}/settings/parameters/common"
    
    try:
        print(f"포인트클라우드 스트리밍 활성화 중...")
        
        # 공식 API로 설정 변경 (publish_level_point_cloud = 2)
        response = requests.post(
            f"{config_url}?config-key=publish_level_point_cloud",
            data="2",
            headers={"Content-Type": "text/plain"},
            timeout=10
        )
        
        if response.status_code == 200:
            print("✅ publish_level_point_cloud = 2 설정 완료")
            
            # 추가 최적화 설정
            optimizations = [
                ("point_cloud_update_interval", "1"),  # 1초 간격
                ("enable_point_cloud_bandwidth_reduction", "false")  # 대역폭 압축 비활성화
            ]
            
            for config_key, value in optimizations:
                try:
                    opt_response = requests.post(
                        f"{config_url}?config-key={config_key}",
                        data=value,
                        headers={"Content-Type": "text/plain"},
                        timeout=10
                    )
                    
                    if opt_response.status_code == 200:
                        print(f"✅ {config_key} = {value} 설정 완료")
                    else:
                        print(f"⚠️ {config_key} 설정 실패 (HTTP {opt_response.status_code})")
                        
                except Exception as e:
                    print(f"⚠️ {config_key} 설정 오류: {e}")
            
            print("\n🎉 포인트클라우드 스트리밍이 활성화되었습니다!")
            print("이제 포트 5051에서 포인트클라우드 데이터를 스트리밍합니다.")
            return True
            
        else:
            print(f"❌ 설정 변경 실패: HTTP {response.status_code}")
            print(f"응답: {response.text}")
            return False
            
    except requests.exceptions.ConnectionError:
        print(f"❌ SENSR 서버 연결 실패: {sensr_host}:{rest_port}")
        return False
        
    except Exception as e:
        print(f"❌ 설정 변경 오류: {e}")
        return False


def get_sensors(sensr_host="112.133.37.122", rest_port=9080, version=None):
    """
    센서 목록 조회 (공식 REST API)
    
    Args:
        sensr_host: SENSR 서버 IP
        rest_port: REST API 포트
        version: SENSR 버전
    """
    
    if not version:
        version = get_sensr_version(sensr_host, rest_port)
    
    base_url = f"http://{sensr_host}:{rest_port}"
    sensors_url = f"{base_url}/{version}/settings/sensor-ext"
    
    try:
        print("센서 목록 조회 중...")
        
        # 센서 ID 목록 가져오기
        response = requests.get(sensors_url, timeout=10)
        
        if response.status_code == 200:
            sensor_ids = response.json()
            print(f"✅ 발견된 센서: {len(sensor_ids)}개")
            
            sensors_detail = {}
            for sensor_id in sensor_ids:
                try:
                    detail_response = requests.get(
                        f"{sensors_url}?sensor-id={sensor_id}",
                        timeout=10
                    )
                    
                    if detail_response.status_code == 200:
                        sensor_detail = detail_response.json()
                        sensors_detail[sensor_id] = sensor_detail
                        print(f"  📡 {sensor_id}: {sensor_detail.get('name', 'Unknown')} ({sensor_detail.get('sensor', 'Unknown')})")
                    else:
                        print(f"  ❌ {sensor_id}: 상세 정보 조회 실패")
                        
                except Exception as e:
                    print(f"  ❌ {sensor_id}: 오류 ({e})")
            
            return sensors_detail
            
        else:
            print(f"❌ 센서 목록 조회 실패: HTTP {response.status_code}")
            return None
            
    except requests.exceptions.ConnectionError:
        print(f"❌ SENSR 서버 연결 실패: {sensr_host}:{rest_port}")
        return None
        
    except Exception as e:
        print(f"❌ 센서 조회 오류: {e}")
        return None


def get_nodes(sensr_host="112.133.37.122", rest_port=9080, version=None):
    """
    알고리즘 노드 목록 조회 (공식 REST API)
    
    Args:
        sensr_host: SENSR 서버 IP  
        rest_port: REST API 포트
        version: SENSR 버전
    """
    
    if not version:
        version = get_sensr_version(sensr_host, rest_port)
    
    base_url = f"http://{sensr_host}:{rest_port}"
    
    try:
        print("알고리즘 노드 목록 조회 중...")
        
        # 노드 목록 조회 (API 문서에서 정확한 엔드포인트 확인 필요)
        # 일단 센서에서 노드 정보를 추출하는 방식으로 구현
        sensors = get_sensors(sensr_host, rest_port, version)
        
        if sensors:
            nodes = set()
            for sensor_id, sensor_data in sensors.items():
                # 센서가 연결된 알고리즘 노드 확인
                if sensor_data.get('connected_to_edge_node', False):
                    edge_uid = sensor_data.get('edge_uid', '')
                    if edge_uid:
                        nodes.add(edge_uid)
            
            if nodes:
                print(f"✅ 발견된 알고리즘 노드: {len(nodes)}개")
                for node in nodes:
                    print(f"  🔧 {node}")
                return list(nodes)
            else:
                print("ℹ️ 연결된 알고리즘 노드가 없습니다")
                return []
        
        return None
            
    except Exception as e:
        print(f"❌ 노드 조회 오류: {e}")
        return None


def apply_changes(sensr_host="112.133.37.122", rest_port=9080, version=None):
    """
    설정 변경사항 적용 (공식 REST API)
    
    Args:
        sensr_host: SENSR 서버 IP
        rest_port: REST API 포트
        version: SENSR 버전
    """
    
    if not version:
        version = get_sensr_version(sensr_host, rest_port)
    
    base_url = f"http://{sensr_host}:{rest_port}"
    apply_url = f"{base_url}/{version}/commands/apply-change"
    
    try:
        print("설정 변경사항 적용 중...")
        
        response = requests.post(apply_url, timeout=30)
        
        if response.status_code == 200:
            print("✅ 설정 변경사항이 성공적으로 적용되었습니다!")
            return True
        else:
            print(f"❌ 설정 적용 실패: HTTP {response.status_code}")
            print(f"응답: {response.text}")
            return False
            
    except requests.exceptions.ConnectionError:
        print(f"❌ SENSR 서버 연결 실패: {sensr_host}:{rest_port}")
        return False
        
    except Exception as e:
        print(f"❌ 설정 적용 오류: {e}")
        return False


def get_project_info(sensr_host="112.133.37.122", rest_port=9080, version=None):
    """
    현재 프로젝트 정보 조회 (공식 REST API)
    
    Args:
        sensr_host: SENSR 서버 IP
        rest_port: REST API 포트  
        version: SENSR 버전
    """
    
    if not version:
        version = get_sensr_version(sensr_host, rest_port)
    
    base_url = f"http://{sensr_host}:{rest_port}"
    project_url = f"{base_url}/{version}/commands/project"
    
    try:
        print("현재 프로젝트 정보 조회 중...")
        
        response = requests.get(project_url, timeout=10)
        
        if response.status_code == 200:
            project_info = response.json()
            project_name = project_info.get('project_name', 'Unknown')
            print(f"✅ 현재 프로젝트: {project_name}")
            return project_info
        else:
            print(f"❌ 프로젝트 정보 조회 실패: HTTP {response.status_code}")
            return None
            
    except requests.exceptions.ConnectionError:
        print(f"❌ SENSR 서버 연결 실패: {sensr_host}:{rest_port}")
        return None
        
    except Exception as e:
        print(f"❌ 프로젝트 조회 오류: {e}")
        return None




def main(host: str, rest_port: int = 9080) -> int:
    print("=" * 50)
    print("SENSR 포인트클라우드 출력 활성화 도우미")
    print("=" * 50)

    print()
    print("1. 프로젝트 정보 확인 중...")
    project_info = get_project_info(host, rest_port)

    print()
    print("2. 시스템 구성 확인 중...")
    sensors = get_sensors(host, rest_port)
    nodes = get_nodes(host, rest_port)

    if sensors is None:
        print()
        print("⚠ 센서 정보를 가져오지 못했습니다. 설정을 다시 확인해 주세요.")
        return 1

    print()
    print("3. 포인트클라우드 설정 확인 중...")
    current_configs, version = get_pointcloud_config(host, rest_port)
    if current_configs is None:
        print()
        print("⚠ 포인트클라우드 설정을 불러오지 못했습니다.")
        return 1

    publish_level = str(current_configs.get('publish_level_point_cloud', '0'))
    config_changed = False

    if publish_level != '2':
        print()
        print("4. 포인트클라우드 출력 레벨을 FULL(2)로 변경합니다...")
        if set_pointcloud_config(host, rest_port=rest_port, version=version):
            config_changed = True
            print()
            print("5. 변경된 설정을 검증합니다...")
            get_pointcloud_config(host, rest_port, version)
        else:
            print()
            print("⚠ 포인트클라우드 활성화에 실패했습니다.")
            print("   SENSR 웹 설정(8080)에서 common.output.publish_point_cloud = 2 로 수동 변경이 필요할 수 있습니다.")
            return 1
    else:
        print()
        print("✔ 포인트클라우드 출력이 이미 활성화되어 있습니다.")

    if config_changed:
        print()
        print("6. 변경 사항을 적용합니다...")
        if apply_changes(host, rest_port=rest_port, version=version):
            print("✔ 변경 사항이 성공적으로 적용되었습니다!")
        else:
            print("⚠ 변경 사항 적용 중 오류가 발생했습니다. 관리자 페이지에서 확인해 주세요.")
            return 1

    final_level = '2' if config_changed else publish_level

    print()
    print("=" * 60)
    print("✅ SENSR 포인트클라우드 설정 완료")
    print("=" * 60)
    print("📌 시스템 요약:")
    if project_info:
        print(f"   프로젝트: {project_info.get('project_name', 'Unknown')}")
    print(f"   센서 수: {len(sensors) if sensors else 0}개")
    print(f"   노드 수: {len(nodes) if nodes else 0}개")
    status_label = "활성(2)" if final_level == '2' else f"비활성({final_level})"
    print(f"   포인트클라우드 출력 레벨: {status_label}")
    print()
    print("📎 main.py를 실행하면 포인트클라우드 데이터를 수집할 수 있습니다!")
    print("=" * 60)

    return 0

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Enable SENSR point cloud output via REST API"
    )
    parser.add_argument('host', nargs='?', default='112.133.37.122', help='SENSR host IP')
    parser.add_argument('--rest', type=int, default=9080, help='REST API port')
    args = parser.parse_args()

    sys.exit(main(args.host, args.rest))
