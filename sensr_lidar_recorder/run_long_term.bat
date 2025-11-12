@echo off
chcp 65001 >nul

echo ================================================
echo 🔋 SENSR Long-term Monitoring (Minimal Load)
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

echo [INFO] Starting Long-term Point Cloud Monitor...
echo [INFO] Features: 1-second interval, very low server load
echo [INFO] Suitable for: 24/7 monitoring, multi-user environments
echo [INFO] Output: simple_output\ directory
echo [INFO] Press Ctrl+C to stop
echo.

python simple_pointcloud_recorder.py --interval 1.0

echo.
echo [INFO] Long-term monitor terminated
pause