@echo off
REM Pull and delete fateweaver logs from FTC Robot Control Hub
REM This script uses ADB to download log files from the robot and then deletes them

setlocal

echo ================================
echo Fateweaver Log Puller (Pull and Delete)
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
    echo.
    echo Now deleting logs from robot...
    echo.

    REM Delete all files in the logs directory on the robot
    adb shell rm -rf %ROBOT_LOG_PATH%/*

    if %ERRORLEVEL% EQU 0 (
        echo ================================
        echo SUCCESS: Logs deleted from robot
        echo ================================
    ) else (
        echo ================================
        echo WARNING: Failed to delete logs from robot
        echo ================================
        echo The logs were downloaded but may still be on the robot.
    )
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
    echo.
    echo Logs were NOT deleted from robot since pull failed.
)

echo.
pause
