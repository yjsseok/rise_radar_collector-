@echo off
chcp 65001 >nul
setlocal EnableDelayedExpansion

echo ================================================
echo 🚀 SENSR LiDAR Data Recorder (Full Features)
echo ================================================

echo [INFO] Checking Python installation...
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Python is not installed
    pause
    exit /b 1
)

echo [INFO] Checking config file...
if not exist "config\config.yaml" (
    echo [ERROR] Config file not found: config\config.yaml
    echo Please run setup.bat first
    pause
    exit /b 1
)

echo [INFO] Creating required directories...
if not exist "logs" mkdir logs
if not exist "output" mkdir output

echo [INFO] Starting SENSR Full-Featured Data Recorder...
echo [INFO] Features: All data types + Zone/Health API integration
echo [INFO] Output: output\ directory
echo [INFO] Press Ctrl+C to stop
echo.

REM Default arguments if none provided
if "%~1"=="" (
    echo [INFO] Using default settings: --pointcloud-interval 0.5 --output-data-interval 0.5
    python main.py --pointcloud-interval 0.5 --output-data-interval 0.5
) else (
    python main.py %*
)

echo.
echo [INFO] Full-featured recorder terminated
pause