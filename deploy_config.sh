#!/bin/bash
# Deploy DECODE robot configuration to Control Hub
# Usage: ./deploy_config.sh

echo "======================================"
echo "DECODE Configuration Deployment"
echo "======================================"
echo ""

# Check if ADB is installed
if ! command -v adb &> /dev/null; then
    echo "ERROR: ADB is not installed or not in PATH"
    echo "Install Android SDK Platform Tools to get ADB"
    exit 1
fi

# Check if config file exists
if [ ! -f "DECODE_Robot_Config.xml" ]; then
    echo "ERROR: DECODE_Robot_Config.xml not found in current directory"
    exit 1
fi

echo "NOTE: Ensure you're already connected to Control Hub via ADB"
echo "(Use ADB WiFi plugin or run: adb connect 192.168.49.1:5555)"
echo ""
echo "Pushing configuration file..."
adb push DECODE_Robot_Config.xml /sdcard/FIRST/

if [ $? -ne 0 ]; then
    echo "ERROR: Failed to push configuration file"
    exit 1
fi

echo ""
echo "✅ Configuration uploaded successfully!"
echo ""
echo "Next steps:"
echo "1. Open FTC Robot Controller app on Driver Station"
echo "2. Tap Menu (≡) → Configure Robot"
echo "3. Tap ⋮ (three dots) → Import Configuration from File"
echo "4. Select 'DECODE_Robot_Config.xml' from /sdcard/FIRST/"
echo "5. Set as Active Configuration"
echo ""
echo "Done!"
