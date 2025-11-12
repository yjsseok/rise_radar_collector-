#!/usr/bin/env python3
"""
SENSR 종합 관리 도구 (공식 REST API 기반)
- 센서 관리 (추가/삭제/수정/조회)
- 노드 관리 (알고리즘 노드 설정 및 상태)
- 존 관리 (이벤트 존 생성/수정/삭제)
- 시스템 헬스 모니터링
- 프로젝트 관리 (생성/로드/저장)
"""

import requests
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry
import sys
import json
import argparse
from typing import Dict, List, Optional, Any


class SensrManager:
    """SENSR 시스템 관리 클래스 (HTTP Connection Pooling 적용)"""

    def __init__(self, host="112.133.37.122", port=9080, version=None):
        self.host = host
        self.port = port
        self.base_url = f"http://{host}:{port}"
        self.version = version  # 수동으로 버전 지정 가능

        # HTTP Session 생성 (연결 재사용 및 dead connection 방지)
        self.session = self._create_session()

    def _create_session(self) -> requests.Session:
        """
        HTTP Session 생성 및 설정
        - Connection pooling으로 연결 재사용
        - Dead connection 방지
        - 자동 재시도 설정
        """
        session = requests.Session()

        # Connection pool 설정
        adapter = HTTPAdapter(
            pool_connections=10,    # 최대 10개의 연결 풀 유지
            pool_maxsize=20,        # 풀당 최대 20개의 연결
            max_retries=Retry(
                total=3,            # 최대 3번 재시도
                backoff_factor=0.3, # 재시도 간격: 0.3, 0.6, 1.2초
                status_forcelist=[500, 502, 503, 504],  # 재시도할 HTTP 상태 코드
            )
        )

        session.mount('http://', adapter)
        session.mount('https://', adapter)

        # Keep-alive 헤더 설정
        session.headers.update({
            'Connection': 'keep-alive',
            'Keep-Alive': 'timeout=30, max=100'
        })

        return session

    def close(self):
        """명시적 Session 종료 (권장)"""
        try:
            if hasattr(self, 'session') and self.session:
                self.session.close()
                self.session = None
                print("✅ HTTP Session 정상 종료")
        except Exception as e:
            print(f"⚠️ Session 종료 중 오류: {e}")

    def __del__(self):
        """소멸자: Session 정리"""
        try:
            if hasattr(self, 'session') and self.session:
                self.session.close()
        except Exception:
            pass  # 소멸자에서는 예외 무시

    def __enter__(self):
        """Context manager 진입"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager 종료 시 Session 정리"""
        self.close()
        return False

    def get_version(self) -> str:
        """SENSR 버전 자동 감지"""
        if self.version:
            return self.version

        versions = ["v4.0", "v3.5.0", "v3.5", "v3.0", "v2.0", "latest"]

        # 여러 엔드포인트로 시도
        test_endpoints = [
            "/settings/parameters/common?config-key=publish_level_point_cloud",
            "/health",
            "/settings/sensor-ext"
        ]

        for version in versions:
            for endpoint in test_endpoints:
                try:
                    test_url = f"{self.base_url}/{version}{endpoint}"
                    response = self.session.get(test_url, timeout=(3, 5))
                    if response.status_code in [200, 201]:
                        print(f"✅ SENSR 버전 감지: {version}")
                        self.version = version
                        return version
                except:
                    continue

        print("❌ SENSR 버전을 자동 감지할 수 없습니다. v3.5.0을 기본값으로 사용합니다.")
        self.version = "v3.5.0"
        return self.version

    # 센서 관리 기능
    def list_sensors(self) -> Dict[str, Any]:
        """센서 목록 조회"""
        version = self.get_version()
        sensors_url = f"{self.base_url}/{version}/settings/sensor-ext"
        
        try:
            print("📡 센서 목록 조회 중...")
            response = self.session.get(sensors_url, timeout=(5, 10))  # (connect timeout, read timeout)
            
            if response.status_code == 200:
                sensor_ids = response.json()
                print(f"✅ 발견된 센서: {len(sensor_ids)}개")
                
                sensors_detail = {}
                for sensor_id in sensor_ids:
                    try:
                        detail_response = self.session.get(
                            f"{sensors_url}?sensor-id={sensor_id}",
                            timeout=(5, 10)  # (connect timeout, read timeout)
                        )
                        
                        if detail_response.status_code == 200:
                            sensor_detail = detail_response.json()
                            sensors_detail[sensor_id] = sensor_detail
                            
                            status = "🟢 활성" if not sensor_detail.get('disabled', False) else "🔴 비활성"
                            sensor_type = sensor_detail.get('sensor', 'Unknown')
                            topic = sensor_detail.get('topic', 'Unknown')
                            
                            print(f"  {sensor_id}: {sensor_type} | {topic} | {status}")
                        else:
                            print(f"  ❌ {sensor_id}: 상세 정보 조회 실패")
                            
                    except Exception as e:
                        print(f"  ❌ {sensor_id}: 오류 ({e})")
                
                return sensors_detail
                
            else:
                print(f"❌ 센서 목록 조회 실패: HTTP {response.status_code}")
                return {}
                
        except Exception as e:
            print(f"❌ 센서 조회 오류: {e}")
            return {}

    def get_sensor_detail(self, sensor_id: str) -> Optional[Dict]:
        """특정 센서 상세 정보 조회"""
        version = self.get_version()
        sensor_url = f"{self.base_url}/{version}/settings/sensor-ext?sensor-id={sensor_id}"
        
        try:
            response = self.session.get(sensor_url, timeout=(5, 10))
            
            if response.status_code == 200:
                return response.json()
            else:
                print(f"❌ 센서 {sensor_id} 조회 실패: HTTP {response.status_code}")
                return None
                
        except Exception as e:
            print(f"❌ 센서 조회 오류: {e}")
            return None

    def update_sensor(self, sensor_config: Dict) -> bool:
        """센서 설정 업데이트"""
        version = self.get_version()
        sensor_url = f"{self.base_url}/{version}/settings/sensor-ext"
        
        try:
            response = self.session.post(
                sensor_url,
                json=sensor_config,
                headers={'Content-Type': 'application/json'},
                timeout=(5, 10)  # (connect timeout, read timeout)
            )
            
            if response.status_code == 200:
                print(f"✅ 센서 설정 업데이트 성공")
                return True
            else:
                print(f"❌ 센서 설정 업데이트 실패: HTTP {response.status_code}")
                print(f"응답: {response.text}")
                return False
                
        except Exception as e:
            print(f"❌ 센서 업데이트 오류: {e}")
            return False

    def delete_sensor(self, sensor_id: str) -> bool:
        """센서 삭제"""
        version = self.get_version()
        sensor_url = f"{self.base_url}/{version}/settings/sensor-ext?sensor-id={sensor_id}"
        
        try:
            response = self.session.delete(sensor_url, timeout=(5, 10))
            
            if response.status_code == 200:
                print(f"✅ 센서 {sensor_id} 삭제 성공")
                return True
            else:
                print(f"❌ 센서 삭제 실패: HTTP {response.status_code}")
                return False
                
        except Exception as e:
            print(f"❌ 센서 삭제 오류: {e}")
            return False

    # 존 관리 기능
    def list_zones(self) -> List[int]:
        """존 목록 조회"""
        version = self.get_version()
        zone_url = f"{self.base_url}/{version}/settings/zone"
        
        try:
            print("🗺️ 존 목록 조회 중...")
            response = self.session.get(zone_url, timeout=(5, 10))
            
            if response.status_code == 200:
                zones = response.json()
                print(f"✅ 발견된 존: {len(zones)}개")
                
                for zone_id in zones:
                    # 각 존의 상세 정보 조회
                    zone_detail = self.get_zone_detail(zone_id)
                    if zone_detail:
                        zone_name = zone_detail.get('name', f'Zone-{zone_id}')
                        zone_type = zone_detail.get('zone_type', 'Unknown')
                        print(f"  🏠 {zone_id}: {zone_name} ({zone_type})")
                
                return zones
            else:
                print(f"❌ 존 목록 조회 실패: HTTP {response.status_code}")
                return []
                
        except Exception as e:
            print(f"❌ 존 조회 오류: {e}")
            return []

    def get_zone_detail(self, zone_id: int) -> Optional[Dict]:
        """특정 존 상세 정보 조회"""
        version = self.get_version()
        zone_url = f"{self.base_url}/{version}/settings/zone?zone-id={zone_id}"
        
        try:
            response = self.session.get(zone_url, timeout=(5, 10))
            
            if response.status_code == 200:
                return response.json()
            else:
                return None
                
        except Exception as e:
            return None

    def get_zone_status(self, zone_id: int) -> Optional[Dict]:
        """존 실시간 상태 조회 (Results API)"""
        zone_url = f"{self.base_url}/results/zone?id={zone_id}"
        
        try:
            response = self.session.get(zone_url, timeout=(5, 10))
            
            if response.status_code == 200:
                return response.json()
            else:
                return None
                
        except Exception as e:
            return None

    def create_zone(self, zone_config: Dict) -> Optional[str]:
        """새 존 생성"""
        version = self.get_version()
        zone_url = f"{self.base_url}/{version}/settings/zone"
        
        try:
            response = self.session.put(
                zone_url,
                json=zone_config,
                headers={'Content-Type': 'application/json'},
                timeout=(5, 10)  # (connect timeout, read timeout)
            )
            
            if response.status_code == 200:
                zone_id = response.text.strip().strip('"')
                print(f"✅ 존 생성 성공: {zone_id}")
                return zone_id
            else:
                print(f"❌ 존 생성 실패: HTTP {response.status_code}")
                return None
                
        except Exception as e:
            print(f"❌ 존 생성 오류: {e}")
            return None

    def update_zone(self, zone_config: Dict) -> bool:
        """존 설정 업데이트"""
        version = self.get_version()
        zone_url = f"{self.base_url}/{version}/settings/zone"
        
        try:
            response = self.session.post(
                zone_url,
                json=zone_config,
                headers={'Content-Type': 'application/json'},
                timeout=(5, 10)  # (connect timeout, read timeout)
            )
            
            if response.status_code == 200:
                print(f"✅ 존 업데이트 성공")
                return True
            else:
                print(f"❌ 존 업데이트 실패: HTTP {response.status_code}")
                return False
                
        except Exception as e:
            print(f"❌ 존 업데이트 오류: {e}")
            return False

    def delete_zone(self, zone_id: int) -> bool:
        """존 삭제"""
        version = self.get_version()
        zone_url = f"{self.base_url}/{version}/settings/zone?zone-id={zone_id}"
        
        try:
            response = requests.delete(zone_url, timeout=10)
            
            if response.status_code == 200:
                print(f"✅ 존 {zone_id} 삭제 성공")
                return True
            else:
                print(f"❌ 존 삭제 실패: HTTP {response.status_code}")
                return False
                
        except Exception as e:
            print(f"❌ 존 삭제 오류: {e}")
            return False

    # 시스템 헬스 모니터링
    def get_health_status(self) -> Optional[Dict]:
        """시스템 헬스 상태 조회"""
        health_url = f"{self.base_url}/results/health"
        
        try:
            print("💚 시스템 헬스 상태 조회 중...")
            response = self.session.get(health_url, timeout=(5, 10))
            
            if response.status_code == 200:
                health_data = response.json()
                
                # 마스터 상태
                master_status = health_data.get('master', 'Unknown')
                status_emoji = "✅" if master_status == "OK" else "❌"
                print(f"  {status_emoji} Master: {master_status}")
                
                # 노드별 상태
                nodes = health_data.get('nodes', {})
                for node_id, node_data in nodes.items():
                    node_status = node_data.get('status', 'Unknown')
                    node_emoji = "✅" if node_status == "OK" else "❌"
                    print(f"  {node_emoji} Node {node_id}: {node_status}")
                    
                    # 센서별 상태
                    sensors = node_data.get('sensors', {})
                    for sensor_name, sensor_status in sensors.items():
                        sensor_emoji = "✅" if "ALIVE" in sensor_status else "❌"
                        print(f"    {sensor_emoji} {sensor_name}: {sensor_status}")
                
                return health_data
            else:
                print(f"❌ 헬스 상태 조회 실패: HTTP {response.status_code}")
                return None
                
        except Exception as e:
            print(f"❌ 헬스 상태 조회 오류: {e}")
            return None

    # 프로젝트 관리
    def get_current_project(self) -> Optional[Dict]:
        """현재 프로젝트 정보 조회"""
        version = self.get_version()
        project_url = f"{self.base_url}/{version}/commands/project"
        
        try:
            response = self.session.get(project_url, timeout=(5, 10))
            
            if response.status_code == 200:
                return response.json()
            else:
                return None
                
        except Exception as e:
            return None

    def list_projects(self, path: Optional[str] = None) -> List[str]:
        """프로젝트 목록 조회"""
        version = self.get_version()
        project_url = f"{self.base_url}/{version}/commands/project/list"
        
        if path:
            project_url += f"?path={path}"
        
        try:
            print("📁 프로젝트 목록 조회 중...")
            response = self.session.get(project_url, timeout=(5, 10))
            
            if response.status_code == 200:
                projects = response.json()
                print(f"✅ 발견된 프로젝트: {len(projects)}개")
                
                for project in projects:
                    print(f"  📂 {project}")
                    
                return projects
            else:
                print(f"❌ 프로젝트 목록 조회 실패: HTTP {response.status_code}")
                return []
                
        except Exception as e:
            print(f"❌ 프로젝트 조회 오류: {e}")
            return []

    def create_project(self, name: str, path: Optional[str] = None) -> bool:
        """새 프로젝트 생성"""
        version = self.get_version()
        project_url = f"{self.base_url}/{version}/commands/project?name={name}"
        
        if path:
            project_url += f"&path={path}"
        
        try:
            response = self.session.put(project_url, timeout=(5, 30))
            
            if response.status_code == 200:
                print(f"✅ 프로젝트 '{name}' 생성 성공")
                return True
            else:
                print(f"❌ 프로젝트 생성 실패: HTTP {response.status_code}")
                print(f"응답: {response.text}")
                return False
                
        except Exception as e:
            print(f"❌ 프로젝트 생성 오류: {e}")
            return False

    def load_project(self, name: str, path: Optional[str] = None) -> bool:
        """기존 프로젝트 로드"""
        version = self.get_version()
        project_url = f"{self.base_url}/{version}/commands/project?name={name}"
        
        if path:
            project_url += f"&path={path}"
        
        try:
            response = self.session.post(project_url, timeout=(5, 30))
            
            if response.status_code == 200:
                print(f"✅ 프로젝트 '{name}' 로드 성공")
                return True
            else:
                print(f"❌ 프로젝트 로드 실패: HTTP {response.status_code}")
                print(f"응답: {response.text}")
                return False
                
        except Exception as e:
            print(f"❌ 프로젝트 로드 오류: {e}")
            return False

    # 설정 적용
    def apply_changes(self) -> bool:
        """설정 변경사항 적용"""
        version = self.get_version()
        apply_url = f"{self.base_url}/{version}/commands/apply-change"
        
        try:
            print("💾 설정 변경사항 적용 중...")
            response = self.session.post(apply_url, timeout=(5, 30))
            
            if response.status_code == 200:
                print("✅ 설정 변경사항이 성공적으로 적용되었습니다!")
                return True
            else:
                print(f"❌ 설정 적용 실패: HTTP {response.status_code}")
                return False
                
        except Exception as e:
            print(f"❌ 설정 적용 오류: {e}")
            return False

    # 종합 상태 보고서
    def generate_status_report(self):
        """시스템 전체 상태 보고서 생성"""
        print("=" * 80)
        print("🔍 SENSR 시스템 종합 상태 보고서")
        print("=" * 80)
        
        # 프로젝트 정보
        project_info = self.get_current_project()
        if project_info:
            print(f"📂 현재 프로젝트: {project_info.get('project_name', 'Unknown')}")
        else:
            print("❌ 프로젝트 정보를 가져올 수 없습니다")
        
        print("\n" + "-" * 40)
        
        # 센서 상태
        sensors = self.list_sensors()
        
        print("\n" + "-" * 40)
        
        # 존 상태
        zones = self.list_zones()
        
        print("\n" + "-" * 40)
        
        # 헬스 상태
        health = self.get_health_status()
        
        print("\n" + "=" * 80)
        print("📊 요약:")
        print(f"   센서: {len(sensors)}개")
        print(f"   존: {len(zones)}개")
        if health:
            master_status = health.get('master', 'Unknown')
            nodes_count = len(health.get('nodes', {}))
            print(f"   마스터 상태: {master_status}")
            print(f"   노드: {nodes_count}개")
        print("=" * 80)


def main():
    parser = argparse.ArgumentParser(description='SENSR 종합 관리 도구')
    parser.add_argument('--host', default='112.133.37.122', help='SENSR 서버 IP')
    parser.add_argument('--port', type=int, default=9080, help='SENSR REST API 포트')
    
    subparsers = parser.add_subparsers(dest='command', help='사용 가능한 명령어')
    
    # 상태 보고서
    subparsers.add_parser('status', help='시스템 전체 상태 보고서')
    
    # 센서 관리
    sensor_parser = subparsers.add_parser('sensor', help='센서 관리')
    sensor_subparsers = sensor_parser.add_subparsers(dest='sensor_action')
    sensor_subparsers.add_parser('list', help='센서 목록 조회')
    
    sensor_detail_parser = sensor_subparsers.add_parser('detail', help='센서 상세 조회')
    sensor_detail_parser.add_argument('sensor_id', help='센서 ID')
    
    sensor_delete_parser = sensor_subparsers.add_parser('delete', help='센서 삭제')
    sensor_delete_parser.add_argument('sensor_id', help='센서 ID')
    
    # 존 관리
    zone_parser = subparsers.add_parser('zone', help='존 관리')
    zone_subparsers = zone_parser.add_subparsers(dest='zone_action')
    zone_subparsers.add_parser('list', help='존 목록 조회')
    
    zone_detail_parser = zone_subparsers.add_parser('detail', help='존 상세 조회')
    zone_detail_parser.add_argument('zone_id', type=int, help='존 ID')
    
    zone_status_parser = zone_subparsers.add_parser('status', help='존 실시간 상태')
    zone_status_parser.add_argument('zone_id', type=int, help='존 ID')
    
    zone_delete_parser = zone_subparsers.add_parser('delete', help='존 삭제')
    zone_delete_parser.add_argument('zone_id', type=int, help='존 ID')
    
    # 프로젝트 관리
    project_parser = subparsers.add_parser('project', help='프로젝트 관리')
    project_subparsers = project_parser.add_subparsers(dest='project_action')
    project_subparsers.add_parser('current', help='현재 프로젝트 조회')
    project_subparsers.add_parser('list', help='프로젝트 목록 조회')
    
    project_create_parser = project_subparsers.add_parser('create', help='프로젝트 생성')
    project_create_parser.add_argument('name', help='프로젝트 이름')
    project_create_parser.add_argument('--path', help='프로젝트 경로 (선택사항)')
    
    project_load_parser = project_subparsers.add_parser('load', help='프로젝트 로드')
    project_load_parser.add_argument('name', help='프로젝트 이름')
    project_load_parser.add_argument('--path', help='프로젝트 경로 (선택사항)')
    
    # 헬스
    subparsers.add_parser('health', help='시스템 헬스 상태')
    
    # 설정 적용
    subparsers.add_parser('apply', help='설정 변경사항 적용')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return
    
    manager = SensrManager(args.host, args.port)
    
    try:
        if args.command == 'status':
            manager.generate_status_report()
            
        elif args.command == 'sensor':
            if args.sensor_action == 'list':
                manager.list_sensors()
            elif args.sensor_action == 'detail':
                detail = manager.get_sensor_detail(args.sensor_id)
                if detail:
                    print(json.dumps(detail, indent=2, ensure_ascii=False))
            elif args.sensor_action == 'delete':
                manager.delete_sensor(args.sensor_id)
                
        elif args.command == 'zone':
            if args.zone_action == 'list':
                manager.list_zones()
            elif args.zone_action == 'detail':
                detail = manager.get_zone_detail(args.zone_id)
                if detail:
                    print(json.dumps(detail, indent=2, ensure_ascii=False))
            elif args.zone_action == 'status':
                status = manager.get_zone_status(args.zone_id)
                if status:
                    print(json.dumps(status, indent=2, ensure_ascii=False))
            elif args.zone_action == 'delete':
                manager.delete_zone(args.zone_id)
                
        elif args.command == 'project':
            if args.project_action == 'current':
                current = manager.get_current_project()
                if current:
                    print(json.dumps(current, indent=2, ensure_ascii=False))
            elif args.project_action == 'list':
                manager.list_projects()
            elif args.project_action == 'create':
                manager.create_project(args.name, args.path)
            elif args.project_action == 'load':
                manager.load_project(args.name, args.path)
                
        elif args.command == 'health':
            manager.get_health_status()
            
        elif args.command == 'apply':
            manager.apply_changes()
            
    except KeyboardInterrupt:
        print("\n\n⏹️ 사용자에 의해 중단되었습니다.")
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")


if __name__ == "__main__":
    main()