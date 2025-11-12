@echo off
chcp 65001 >nul

echo ================================================
echo 🧪 SENSR Test Mode (1 Minute Collection)
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

echo [INFO] Starting Test Collection...
echo [INFO] Duration: 60 seconds (1 minute)
echo [INFO] Interval: 0.5 seconds
echo [INFO] Output: simple_output\ directory
echo.

python simple_pointcloud_recorder.py --interval 0.5 --duration 60

echo.
echo [INFO] Test completed! Check simple_output\ directory for results
pause