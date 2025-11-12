@echo off
chcp 65001 >nul
setlocal EnableDelayedExpansion

echo ================================================
echo 🚀 SENSR Multiprocessing Recorder (WSL Version)
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
    echo Please copy config.yaml.example to config.yaml and edit it
    pause
    exit /b 1
)

echo [INFO] Creating required directories in WSL...
wsl mkdir -p /mnt/e/work/withclaude/radar_data/radar_v2/logs
wsl mkdir -p /mnt/e/work/withclaude/radar_data/radar_v2/simple_output

echo [INFO] Starting SENSR Multiprocessing Recorder in WSL...
echo [INFO] Features: Multiprocessing optimization
echo [INFO] Output: simple_output/ directory
echo [INFO] Press Ctrl+C to stop
echo.

wsl cd /mnt/e/work/withclaude/radar_data/radar_v2/test ^&^& python3 main_multiprocessing.py --config ../config/config.yaml

echo.
echo [INFO] Multiprocessing recorder terminated
pause
