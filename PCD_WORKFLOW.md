# PCD 변환 및 재생 가이드

## 1. ROS 2 bag 디렉터리에서 PCD 추출하기

1. **ROS 2 환경 준비**
   - `source /opt/ros/<distro>/setup.bash` (WSL/리눅스) 또는 Windows PowerShell에서 `call C:\dev\ros2\<distro>\local_setup.bat` 실행.
   - `rosbag2_py`, `sensor_msgs_py`가 import 가능한지 확인합니다.
2. **스크립트 실행** (`rostopcd.py`는 리포지토리 루트에 위치)
   ```bash
   python3 rostopcd.py extract <bag_폴더> <pcd_출력_디렉터리> \
       --topics /sensr/pointcloud \
       --skip-nans
   ```
   - `bag_폴더`: `metadata.yaml`과 `.db3` 파일이 들어 있는 rosbag2 디렉터리 경로.
   - `pcd_출력_디렉터리`: PCD 파일을 저장할 루트 경로. 토픽별 하위 폴더가 생성됩니다.
   - 주요 옵션
     - `--topics <topic ...>`: 특정 PointCloud2 토픽만 추출.
     - `--skip-nans`: NaN 포인트를 제외하고 저장.
     - `--max-messages N`: 메시지 N개까지만 변환.
     - `--play-after`: 추출 완료 후 `ros2 bag play` 실행.
     - `--ros2-args <args>`: 재생 시 추가 인자 전달 (`--ros2-args -r 0.5 --loop`).
3. **결과 구조**
   - 파일명은 `<토픽명>_<timestamp_ns>.pcd` 형식으로 저장되며, 타임스탬프는 나노초 단위 ROS 시간입니다.

## 2. PCD 플레이어 (`pcdplayer.py`)

1. **의존성 설치**
   ```bash
   pip install open3d
   ```
2. **실행**
   ```bash
   python3 pcdplayer.py <pcd_디렉터리> --fps 8 --loop [--recursive]
   ```
   - `--fps`: 재생 프레임 속도.
   - `--loop`: 마지막 프레임 후 처음으로 반복.
   - `--recursive`: 하위 폴더까지 모든 PCD 검색 (`output_pcd_test`처럼 토픽별 폴더가 있을 때 유용).
3. **조작 방법**
   - 버튼: Play / Pause / Rewind.
   - 키보드: `Space`(재생/정지), `←` `→`(이전/다음 프레임).
   - 슬라이더: 임의 프레임으로 즉시 이동.
   - OpenGL 백엔드가 필요하므로 WSL에서는 WSLg 또는 X 서버 설정이 되어 있어야 합니다.

## 3. Bag 분할 길이 조정

- 설정 파일: `sensr_lidar_recorder/config/config.yaml`
  ```yaml
  recording:
    duration: 10  # bag 파일 분할 주기 (초)
  ```
- `duration` 값은 **초 단위**로 사용됩니다. 예) 5분 수집 → `duration: 300`.
- 실행 시 일시적으로 바꾸고 싶다면
  ```bash
  python sensr_lidar_recorder/main.py --duration 1800 ...
  ```
  처럼 `--duration <초>` 옵션을 추가합니다.

## 4. 예시 워크플로우

```bash
# WSL 예시
source /opt/ros/humble/setup.bash
python3 rostopcd.py extract sensr_lidar_recorder/output/sensr_data_20251001_175145 output_pcd_test --play-after --ros2-args -r 0.5
python3 pcdplayer.py output_pcd_test/sensr_pointcloud --fps 8 --loop
```

```powershell
# Windows PowerShell 예시
call C:\dev\ros2\humble\local_setup.bat
python rostopcd.py extract sensr_lidar_recorder/output/sensr_data_20251001_175145 output_pcd_test --skip-nans
python pcdplayer.py output_pcd_test --recursive --fps 10
```

- PCD 변환을 자주 사용할 경우 ROS 2 환경이 적용된 전용 가상환경을 마련하면 편리합니다.
