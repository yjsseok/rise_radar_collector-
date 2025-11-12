# Dead Connection 문제 수정 사항

## 개요
레이더 회사 서버에 좀비 연결(dead connection)이 누적되어 서버가 다운되는 문제를 해결하기 위한 수정 사항입니다.

## 수정 날짜
2025-11-12

## 수정된 파일

### 1. sensr_sdk/python/sensr_message_listener.py
**문제:** WebSocket ping이 완전히 비활성화되어 있어 dead connection 감지 불가

**수정 내용:**
- `ping_interval=None` → `ping_interval=20` (20초마다 ping)
- `ping_timeout=10` 추가 (10초 내 응답 없으면 연결 종료)
- Exponential backoff 재연결 로직 추가 (1초 → 2초 → 4초 → 8초 → 최대 60초)

**변경 라인:**
- 라인 67, 90: WebSocket 연결 설정
- 라인 96-101: Exponential backoff 추가

**효과:**
- 끊긴 연결을 20-30초 이내에 자동 감지 및 정리
- 서버 부하를 줄이는 점진적 재연결

---

### 2. src/sensr_client.py
**문제:** WebSocket ping 간격이 너무 김 (30초)

**수정 내용:**
- `ping_interval=30` → `ping_interval=15` (15초마다 ping)
- `ping_timeout=10` → `ping_timeout=8` (8초로 단축)
- Exponential backoff 재연결 로직 추가

**변경 라인:**
- 라인 165-166: WebSocket 옵션 설정
- 라인 153-183: 재연결 로직 전체 수정

**효과:**
- 더 빠른 dead connection 감지 (15-23초 이내)
- 재연결 시 서버 부담 감소

---

### 3. sensr_manager.py ⭐ 가장 중요
**문제:** HTTP 연결을 매번 새로 생성하고 Session을 사용하지 않아 서버에 TIME_WAIT 상태 연결 누적

**수정 내용:**
#### A. HTTP Session 연결 풀 적용
```python
# 추가된 메서드
def _create_session(self) -> requests.Session:
    """
    HTTP Session 생성 및 설정
    - Connection pooling으로 연결 재사용
    - Dead connection 방지
    - 자동 재시도 설정
    """
    session = requests.Session()

    adapter = HTTPAdapter(
        pool_connections=10,    # 최대 10개의 연결 풀 유지
        pool_maxsize=20,        # 풀당 최대 20개의 연결
        max_retries=Retry(
            total=3,            # 최대 3번 재시도
            backoff_factor=0.3, # 재시도 간격: 0.3, 0.6, 1.2초
            status_forcelist=[500, 502, 503, 504],
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

def __del__(self):
    """소멸자: Session 정리"""
    if hasattr(self, 'session'):
        self.session.close()
```

#### B. 모든 HTTP 호출을 Session으로 변경
```python
# 변경 전
response = requests.get(url, timeout=10)
response = requests.post(url, json=data, timeout=10)

# 변경 후
response = self.session.get(url, timeout=(5, 10))
response = self.session.post(url, json=data, timeout=(5, 10))
```

#### C. 타임아웃 설정 개선
```python
# 변경 전
timeout=10  # 단일 타임아웃

# 변경 후
timeout=(5, 10)  # (연결 타임아웃, 읽기 타임아웃)
```

**변경 라인:**
- 라인 11-13: 새로운 import 추가
- 라인 29-66: Session 생성 및 소멸자 추가
- 전체 파일: 모든 `requests.get/post/put/delete` → `self.session.get/post/put/delete`
- 전체 파일: 모든 `timeout=10` → `timeout=(5, 10)` 또는 `timeout=(5, 30)`

**효과:**
- HTTP 연결 재사용으로 TIME_WAIT 상태 연결 최소화
- 서버의 포트/파일 디스크립터 고갈 방지
- 연결 생성/종료 오버헤드 감소
- 자동 재시도로 안정성 향상

---

## 수정 전후 비교

### 시나리오: 1시간 운영 시

#### 수정 전
```
- WebSocket: ping 없음 → 네트워크 끊김 감지 불가
- HTTP: 매번 새 연결 생성 → 1시간에 1000번 API 호출 시
  → 1000개의 TCP 연결 생성/종료
  → TIME_WAIT 상태로 서버에 수백 개 누적
- 재연결: 1초마다 무한 재시도 → 서버에 DDoS 수준 부하
- 결과: 서버 파일 디스크립터 한계 도달 → 💀 서버 다운
```

#### 수정 후
```
- WebSocket: 15-20초마다 ping → 끊긴 연결 빠르게 감지 및 정리
- HTTP: 연결 재사용 → 1시간에 1000번 API 호출해도
  → 최대 10-20개의 연결만 유지
  → 연결 자동 정리 및 재사용
- 재연결: 지수 백오프 (1초 → 2초 → 4초 → 8초 → 60초)
  → 서버 부하 최소화
- 결과: 좀비 연결 0개 유지 → ✅ 서버 안정
```

---

## 핵심 개선 사항

### 1. WebSocket 연결 관리
- ✅ **Ping/Pong 메커니즘**: 15-20초 간격으로 연결 상태 확인
- ✅ **자동 정리**: 응답 없는 연결 자동 종료
- ✅ **재연결 백오프**: 서버 부하 최소화

### 2. HTTP 연결 풀링
- ✅ **연결 재사용**: Connection pooling으로 불필요한 연결 생성 방지
- ✅ **Keep-alive**: 연결 유지 및 효율적 관리
- ✅ **자동 재시도**: 일시적 네트워크 오류 자동 복구

### 3. 타임아웃 설정
- ✅ **세분화된 타임아웃**: 연결/읽기 타임아웃 분리
- ✅ **빠른 실패**: 문제 있는 연결 빠르게 감지

---

## 테스트 권장 사항

### 1. 정상 동작 테스트
```bash
cd radar_v2
python sensr_manager.py --list-sensors
```

### 2. 네트워크 불안정 시뮬레이션
- 네트워크 일시 차단 후 재연결 확인
- 로그에서 exponential backoff 동작 확인

### 3. 장시간 운영 테스트
- 1시간 이상 연속 운영
- 서버 연결 수 모니터링 (netstat 또는 ss 명령)

### 4. 서버 측 모니터링
```bash
# 연결 수 확인
netstat -an | grep :9080 | wc -l

# TIME_WAIT 상태 연결 수
netstat -an | grep TIME_WAIT | grep :9080 | wc -l
```

---

## 롤백 방법

문제 발생 시 원본 폴더로 복귀:
```bash
# radar_v2 대신 sensr_lidar_recorder 사용
cd ../sensr_lidar_recorder
```

---

## 레이더 회사에 전달할 내용

수정 완료 후 레이더 회사에 다음 사항을 알려주세요:

> "Dead connection 문제를 해결했습니다:
>
> 1. WebSocket에 ping/pong 메커니즘 추가 (15-20초 간격)
> 2. HTTP 연결 풀링 적용으로 연결 재사용
> 3. 재연결 시 exponential backoff 적용
>
> 이제 좀비 연결이 서버에 누적되지 않고 자동으로 정리됩니다.
> 테스트 후 안정성을 모니터링 하겠습니다."

---

## 추가 모니터링 권장

향후 문제 예방을 위해:

1. **로그 모니터링**: 재연결 빈도 확인
2. **서버 연결 수**: 정상 범위 내 유지 확인
3. **응답 시간**: API 응답 시간 추적
4. **에러율**: 연결 실패율 모니터링

---

## 문의 사항

이 수정 사항에 대한 문의는 개발팀에게 연락주세요.
