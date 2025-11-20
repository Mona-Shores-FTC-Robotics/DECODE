@echo off
REM Deploy DECODE robot configuration to Control Hub
REM Usage: deploy_config.bat

echo ======================================
echo DECODE Configuration Deployment
echo ======================================
echo.

REM Set ADB path (adjust if your Android SDK is in a different location)
set ADB_PATH=%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe

REM Check if ADB exists
if not exist "%ADB_PATH%" (
    echo ERROR: ADB not found at %ADB_PATH%
    echo.
    echo Please install Android SDK Platform Tools or update ADB_PATH in this script
    echo.
    echo Alternative locations to check:
    echo - %USERPROFILE%\AppData\Local\Android\Sdk\platform-tools\adb.exe
    echo - C:\Android\Sdk\platform-tools\adb.exe
    pause
    exit /b 1
)

REM Check if config file exists
if not exist "DECODE_Robot_Config20245.xml" (
    echo ERROR: DECODE_Robot_Config.xml not found in current directory
    pause
    exit /b 1
)

echo Using ADB at: %ADB_PATH%
echo.
echo NOTE: Ensure you're already connected to Control Hub via ADB
echo ^(Use ADB WiFi plugin or run: adb connect 192.168.49.1:5555^)
echo.
echo Pushing configuration file...
"%ADB_PATH%" push DECODE_Robot_Config.xml /sdcard/FIRST/

if errorlevel 1 (
    echo ERROR: Failed to push configuration file
    pause
    exit /b 1
)

echo.
echo [32m✓ Configuration uploaded successfully![0m
echo.
echo Next steps:
echo 1. Open FTC Robot Controller app on Driver Station
echo 2. Tap Menu ^(≡^) → Configure Robot
echo 3. Tap ⋮ ^(three dots^) → Import Configuration from File
echo 4. Select 'DECODE_Robot_Config.xml' from /sdcard/FIRST/
echo 5. Set as Active Configuration
echo.
echo Done!
pause
