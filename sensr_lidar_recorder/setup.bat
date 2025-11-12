@echo off
chcp 65001 >nul
setlocal EnableDelayedExpansion

echo ================================================
echo SENSR LiDAR Data Recorder Windows Setup Script
echo ================================================

echo [INFO] Checking Python installation...
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Python is not installed
    echo Please download Python 3.7+ from https://www.python.org/downloads/
    pause
    exit /b 1
)

for /f "tokens=2" %%i in ('python --version 2^>^&1') do set PYTHON_VERSION=%%i
echo [SUCCESS] Python %PYTHON_VERSION% found

echo [INFO] Upgrading pip...
python -m pip install --upgrade pip

echo [INFO] Installing Python packages...
if exist requirements.txt (
    python -m pip install -r requirements.txt
    echo [SUCCESS] Python packages installed
) else (
    echo [WARNING] requirements.txt not found
    echo [INFO] Installing essential packages...
    python -m pip install websocket-client protobuf==3.20.3 PyYAML numpy
    echo [SUCCESS] Essential Python packages installed
)

echo [INFO] Creating directories...
if not exist "logs" mkdir logs
if not exist "output" mkdir output
if not exist "simple_output" mkdir simple_output
if not exist "proto" mkdir proto
echo [SUCCESS] Directories created

echo [INFO] Checking config file...
if not exist "config\config.yaml" (
    echo [ERROR] Config file not found: config\config.yaml
    echo Please check the config.yaml file in config directory
    pause
    exit /b 1
)
echo [SUCCESS] Config file found

set /p install_sdk="Download Seoul Robotics SDK? (y/n): "
if /i "%install_sdk%"=="y" (
    echo [INFO] Downloading Seoul Robotics SDK...
    
    if not exist "sensr_sdk" (
        git clone https://github.com/seoulrobotics/sensr_sdk.git
        if %errorlevel% neq 0 (
            echo [WARNING] SENSR SDK download failed - git not installed or network issue
        ) else (
            echo [SUCCESS] SENSR SDK downloaded
        )
    ) else (
        echo [INFO] SENSR SDK already exists
    )
    
    if not exist "sensr_proto" (
        git clone https://github.com/seoulrobotics/sensr_proto.git
        if %errorlevel% neq 0 (
            echo [WARNING] SENSR Proto download failed
        ) else (
            echo [SUCCESS] SENSR Proto downloaded
        )
    ) else (
        echo [INFO] SENSR Proto already exists
    )
)

set /p setup_pointcloud="Enable SENSR pointcloud streaming? (recommended, y/n): "
if /i "%setup_pointcloud%"=="y" (
    echo [INFO] Setting up pointcloud streaming...
    python enable_pointcloud.py
    if %errorlevel% equ 0 (
        echo [SUCCESS] Pointcloud setup completed!
    ) else (
        echo [WARNING] Pointcloud setup failed. Please check server connection
    )
)

set /p run_test="Run connection test? (y/n): "
if /i "%run_test%"=="y" (
    echo [INFO] Running connection test...
    python test_connection.py
    if %errorlevel% equ 0 (
        echo [SUCCESS] Connection test passed!
    ) else (
        echo [WARNING] Connection test failed. Please check settings
    )
)

echo.
echo [SUCCESS] Setup completed!
echo.
echo ============ USAGE OPTIONS ============
echo.
echo 🥇 RECOMMENDED (Light Load):
echo   python simple_pointcloud_recorder.py --interval 0.5
echo.
echo 🔧 FULL FEATURES:
echo   python main.py --pointcloud-interval 0.5 --pointcloud-only
echo   python main.py --pointcloud-interval 0.5 --output-data-interval 1.0
echo.
echo 🛠️ MANAGEMENT TOOLS:
echo   python sensr_manager.py status         # System status
echo   python sensr_manager.py sensor list    # List sensors
echo   python sensr_manager.py zone list      # List zones
echo.
echo 📊 INTERVAL OPTIONS (server load control):
echo   --interval 0.5     # Balanced (recommended)
echo   --interval 1.0     # Long-term monitoring
echo   --interval 2.0     # Multi-user environment
echo.
echo 🧪 TESTING:
echo   python test_connection.py             # Connection test
echo   python enable_pointcloud.py           # Setup pointcloud
echo.
echo 📁 FILE LOCATIONS:
echo   - Light mode output: simple_output\
echo   - Full mode output: output\
echo   - Logs: logs\sensr_recorder.log, logs\simple_recorder.log
echo   - Config: config\config.yaml
echo.

pause