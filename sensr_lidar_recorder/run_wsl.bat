@echo off
chcp 65001 >nul
setlocal EnableDelayedExpansion

echo ================================================
echo 🚀 SENSR LiDAR Data Recorder (WSL Version)
echo ================================================

echo [INFO] Checking WSL installation...
wsl --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] WSL is not installed
    pause
    exit /b 1
)

echo [INFO] Checking Python installation in WSL...
wsl python3 --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Python is not installed in WSL
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

echo [INFO] Creating required directories in WSL...
wsl mkdir -p /mnt/e/work/withclaude/radar_data/sensr_lidar_recorder/logs
wsl mkdir -p /mnt/e/work/withclaude/radar_data/sensr_lidar_recorder/output

echo [INFO] Starting SENSR Full-Featured Data Recorder in WSL...
echo [INFO] Features: All data types + Zone/Health API integration
echo [INFO] Output: output/ directory
echo [INFO] Press Ctrl+C to stop
echo.

REM Default arguments if none provided
if "%~1"=="" (
    echo [INFO] Using default settings: --pointcloud-interval 0.5 --output-data-interval 0.5
    wsl python3 /mnt/e/work/withclaude/radar_data/sensr_lidar_recorder/main.py --pointcloud-interval 0.5 --output-data-interval 0.5
) else (
    wsl python3 /mnt/e/work/withclaude/radar_data/sensr_lidar_recorder/main.py %*
)

echo.
echo [INFO] Full-featured recorder terminated
pause