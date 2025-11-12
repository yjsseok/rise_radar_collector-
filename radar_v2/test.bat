@echo off
chcp 65001 >nul

echo ================================================
echo SENSR Connection Test
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
    pause
    exit /b 1
)

echo [INFO] Starting SENSR connection test...
echo.

python test_connection.py

echo.
pause