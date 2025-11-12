@echo off
REM SENSR 멀티프로세싱 테스트 - Windows 환경 (ROS2 없이 실행)

echo ======================================================================
echo SENSR 멀티프로세싱 고속 처리 테스트 (Windows)
echo ======================================================================
echo.

REM Python 가상환경 활성화 시도
if exist "..\venv\Scripts\activate.bat" (
    echo Python 가상환경 활성화...
    call ..\venv\Scripts\activate.bat
)

REM 호스트와 워커 수 설정
set HOST=samyang
set DURATION=60
set WORKERS=4

echo 호스트: %HOST%
echo 테스트 시간: %DURATION%초
echo 워커 수: %WORKERS%개
echo.
echo Windows 환경에서 실행 중 (ROS2 없이 동작)
echo.

REM PYTHONPATH 설정
set PYTHONPATH=%CD%;%CD%\..\src;%PYTHONPATH%

REM Python 실행
python main_multiprocessing.py --host %HOST% --duration %DURATION% --workers %WORKERS%

echo.
echo 테스트 완료!
pause
