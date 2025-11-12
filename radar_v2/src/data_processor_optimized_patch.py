"""
프로토버프 파싱 최적화 패치 코드

이 코드를 data_processor.py의 _decode_pointcloud_protobuf 함수에 적용하세요.
라인 382-425를 아래 코드로 교체하면 됩니다.
"""

# ===== 최적화된 코드 시작 =====
# 🚀 Zero-Copy 최적화: NumPy 배열로 직접 수집
all_points_list = []
all_intensities_list = []

for i, point_cloud in enumerate(point_result.points):
    points_data = point_cloud.points
    intensities_data = point_cloud.intensities

    # 로깅 최적화: DEBUG 레벨로 변경
    if self.logger.isEnabledFor(logging.DEBUG):
        self.logger.debug(f"포인트클라우드 {i}: type={point_cloud.type}, points={len(points_data)} bytes, intensities={len(intensities_data)} bytes")

    # 공식 SENSR SDK 방식으로 파싱
    import ctypes
    import numpy as np

    # 포인트 파싱 (Zero-Copy)
    float_size = ctypes.sizeof(ctypes.c_float)
    num_points = len(points_data) // (float_size * 3)  # Each point is 3 floats (x,y,z)

    if num_points > 0:
        # 🚀 NumPy 배열 직접 사용 (dict 변환 제거로 10-20배 속도 향상)
        points_array = np.frombuffer(points_data, np.float32).reshape(-1, 3)
        all_points_list.append(points_array)

    # Intensities 파싱 (Zero-Copy)
    if len(intensities_data) > 0:
        intensity_np = np.frombuffer(intensities_data, np.float32)
        all_intensities_list.append(intensity_np)

        # 통계 정보 로깅 (DEBUG 레벨로 변경)
        if self.logger.isEnabledFor(logging.DEBUG):
            min_intensity = np.min(intensity_np)
            median_intensity = np.median(intensity_np)
            max_intensity = np.max(intensity_np)
            self.logger.debug(f"포인트클라우드 {i}: {num_points}개 포인트, 강도 범위 [{min_intensity:.2f}, {median_intensity:.2f}, {max_intensity:.2f}]")

# 🚀 NumPy 배열 결합 (vstack/concatenate는 매우 빠름)
if all_points_list:
    all_points_array = np.vstack(all_points_list)
    total_points = len(all_points_array)
else:
    all_points_array = np.array([]).reshape(0, 3)
    total_points = 0

if all_intensities_list:
    all_intensities_array = np.concatenate(all_intensities_list)
else:
    all_intensities_array = np.array([])

self.logger.info(f"최종 파싱 결과: {total_points}개 포인트, {len(all_intensities_array)}개 강도값")

return {
    'points': all_points_array,  # 🚀 NumPy 배열로 반환
    'intensities': all_intensities_array,  # 🚀 NumPy 배열로 반환
    'num_points': total_points,
    'fields': ['x', 'y', 'z', 'intensity']
}
# ===== 최적화된 코드 끝 =====


"""
주요 변경 사항:

1. dict 리스트 대신 NumPy 배열 사용
   - 기존: for point in points_array: all_points.append({'x': ..., 'y': ..., 'z': ...})
   - 개선: all_points_list.append(points_array)
   - 효과: 10-20배 속도 향상

2. .tolist() 제거
   - 기존: all_intensities.extend(intensity_np.tolist())
   - 개선: all_intensities_list.append(intensity_np)
   - 효과: 메모리 복사 제거

3. 로깅 레벨 최적화
   - 기존: self.logger.info() (항상 실행)
   - 개선: if self.logger.isEnabledFor(logging.DEBUG): (필요시만 실행)
   - 효과: I/O 부하 감소

4. NumPy vstack/concatenate 사용
   - 여러 배열을 빠르게 결합
   - C 레벨 최적화로 매우 빠름

예상 성능 개선:
- 파싱 시간: 797ms → 50-100ms (8-15배 향상)
- 메모리 사용: 30-40% 감소
- 큐 오버플로우 해소
"""
