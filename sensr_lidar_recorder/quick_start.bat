@echo off
chcp 65001 >nul

echo ================================================
echo SENSR LiDAR Data Recorder - Quick Start
echo ================================================

echo [INFO] Starting with minimal checks for faster startup...

python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Python not found
    pause
    exit /b 1
)

if not exist "config\config.yaml" (
    echo [ERROR] Config file not found: config\config.yaml
    pause
    exit /b 1
)

if not exist "logs" mkdir logs
if not exist "output" mkdir output

echo [INFO] Quick starting SENSR recorder...
echo [INFO] Connection timeout reduced for faster startup
echo.

python main.py --quick-start

pause