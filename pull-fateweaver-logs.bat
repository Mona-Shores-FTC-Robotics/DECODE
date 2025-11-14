@echo off
REM Pull fateweaver logs from FTC Robot Control Hub
REM This script uses ADB to download log files from the robot

setlocal

echo ================================
echo Fateweaver Log Puller
echo ================================
echo.

REM Check if ADB is available
where adb >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: ADB not found in PATH
    echo.
    echo Please install Android SDK Platform Tools or use REV Hardware Client
    echo Download from: https://developer.android.com/tools/releases/platform-tools
    echo.
    pause
    exit /b 1
)

REM Set default output directory to current directory\fateweaver-logs
set OUTPUT_DIR=%~dp0fateweaver-logs
if not exist "%OUTPUT_DIR%" mkdir "%OUTPUT_DIR%"

echo Output directory: %OUTPUT_DIR%
echo.

REM Check for connected devices
echo Checking for connected devices...
adb devices -l
echo.

REM The logs are stored in /storage/self/primary/FateWeaver/Logs on the Android device
set ROBOT_LOG_PATH=/storage/self/primary/FateWeaver/Logs

echo Pulling logs from robot path: %ROBOT_LOG_PATH%
echo.

REM Pull only log files from the fateweaver logs directory
adb pull %ROBOT_LOG_PATH%/. "%OUTPUT_DIR%"

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ================================
    echo SUCCESS: Logs downloaded to:
    echo %OUTPUT_DIR%
    echo ================================
) else (
    echo.
    echo ================================
    echo ERROR: Failed to pull logs
    echo ================================
    echo.
    echo Troubleshooting:
    echo 1. Ensure robot is connected via USB or WiFi
    echo 2. Check that fateweaver logs exist at %ROBOT_LOG_PATH%
    echo 3. If using WiFi, run: adb connect 192.168.49.1:5555
)

echo.
pause
