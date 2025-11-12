# 보안 가이드 (Security Guidelines)

## 민감한 정보 관리

이 프로젝트에는 다음과 같은 민감한 정보가 포함되어 있을 수 있습니다:
- SENSR 서버 IP 주소
- 네트워크 포트
- 기타 환경별 설정

### 설정 파일 관리

다음 파일들은 `.gitignore`에 포함되어 Git에 커밋되지 않습니다:
- `*/config/config.yaml`
- `ros_test/src/sensr_ros2_driver/config/sensr_params.yaml`
- `.env`, `.env.local`

### 설정 방법

1. **템플릿 파일 복사**:
   ```bash
   # sensr_lidar_recorder
   cp sensr_lidar_recorder/config/config.yaml.example sensr_lidar_recorder/config/config.yaml
   
   # radar_v2
   cp radar_v2/config/config.yaml.example radar_v2/config/config.yaml
   
   # ROS2 드라이버
   cp ros_test/src/sensr_ros2_driver/config/sensr_params.yaml.example \
      ros_test/src/sensr_ros2_driver/config/sensr_params.yaml
   ```

2. **실제 IP 주소로 변경**:
   - `YOUR_SENSR_IP` → 실제 SENSR 서버 IP
   - `YOUR_SITE1_IP` → 실제 사이트1 IP
   - `YOUR_SITE2_IP` → 실제 사이트2 IP

### 하드코딩된 IP 주소

다음 파일들에는 기본값으로 IP 주소가 하드코딩되어 있습니다:
- `simple_pointcloud_recorder.py` (기본값: `112.133.37.122`)
- `enable_pointcloud.py` (기본값: `112.133.37.122`)
- `sensr_manager.py` (기본값: `112.133.37.122`)
- ROS2 launch 파일들

**권장사항**: 
- 명령줄 인자로 IP 주소를 전달하거나
- 환경변수를 사용하여 기본값을 오버라이드하세요

예시:
```bash
# 명령줄 인자 사용
python simple_pointcloud_recorder.py --host YOUR_IP

# 환경변수 사용
export SENSR_HOST=YOUR_IP
python main.py
```

### 문서화된 IP 주소

다음 문서 파일들에는 예시 IP 주소가 포함되어 있습니다:
- `README.md`
- `USAGE.md`
- `how_to_use.md`
- `apiDOC.md`

이러한 IP 주소는 설명을 위한 예시이므로, 실제 환경에서는 해당 값을 변경하여 사용하세요.

### Git 커밋 전 확인사항

새로운 파일을 추가하기 전에 민감한 정보가 포함되어 있지 않은지 확인하세요:

```bash
# 커밋 전 변경사항 확인
git diff

# 민감한 정보 검색
grep -r "112.133.37.122" .
grep -r "122.202.187.5" .
```

### 이미 커밋된 민감한 정보 제거

실수로 민감한 정보를 커밋한 경우:

```bash
# 특정 파일을 Git 히스토리에서 제거
git filter-branch --force --index-filter \
  "git rm --cached --ignore-unmatch path/to/sensitive/file" \
  --prune-empty --tag-name-filter cat -- --all

# 또는 BFG Repo-Cleaner 사용 (권장)
# https://rtyley.github.io/bfg-repo-cleaner/
```

## 문의사항

보안 관련 문제를 발견하신 경우, 공개 이슈 트래커가 아닌 직접 연락 주시기 바랍니다.
