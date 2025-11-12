@echo off
chcp 65001 >nul

echo ================================================
echo 🥇 SENSR Simple Point Cloud Recorder (Recommended)
echo ================================================

echo [INFO] Checking Python installation...
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Python is not installed
    pause
    exit /b 1
)

echo [INFO] Creating required directories...
if not exist "logs" mkdir logs
if not exist "simple_output" mkdir simple_output

echo [INFO] Starting Simple Point Cloud Recorder...
echo [INFO] Features: Point cloud only, minimal server load
echo [INFO] Output: simple_output\ directory
echo [INFO] Press Ctrl+C to stop
echo.

REM Default arguments if none provided
if "%~1"=="" (
    echo [INFO] Using recommended settings: --interval 0.5
    python simple_pointcloud_recorder.py --interval 0.5
) else (
    python simple_pointcloud_recorder.py %*
)

echo.
echo [INFO] Simple recorder terminated
pause