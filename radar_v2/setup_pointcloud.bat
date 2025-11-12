@echo off
chcp 65001 >nul

echo ================================================
echo 🔧 SENSR Point Cloud Setup & Management
echo ================================================

echo [INFO] Checking Python installation...
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Python is not installed
    pause
    exit /b 1
)

echo.
echo [1/3] Enabling point cloud streaming...
python enable_pointcloud.py
if %errorlevel% neq 0 (
    echo [ERROR] Point cloud setup failed
    pause
    exit /b 1
)

echo.
echo [2/3] Testing connection...
python test_connection.py
if %errorlevel% neq 0 (
    echo [WARNING] Connection test failed - please check network settings
)

echo.
echo [3/3] Checking system status...
python sensr_manager.py status

echo.
echo [SUCCESS] Point cloud setup completed!
echo.
echo Next steps:
echo   run_simple.bat          # Start simple collection
echo   run_test.bat             # Test for 1 minute
echo   run_long_term.bat        # Long-term monitoring
echo.

pause