@echo off
REM SENSR 멀티프로세싱 테스트 실행 스크립트

echo ======================================================================
echo SENSR 멀티프로세싱 고속 처리 테스트
echo ======================================================================
echo.

REM 호스트와 워커 수 설정
set HOST=samyang
set DURATION=300
set WORKERS=4

echo 호스트: %HOST%
echo 테스트 시간: %DURATION%초
echo 워커 수: %WORKERS%개
echo.

REM Python 실행
python main_multiprocessing.py --host %HOST% --duration %DURATION% --workers %WORKERS%

echo.
echo 테스트 완료!
pause
