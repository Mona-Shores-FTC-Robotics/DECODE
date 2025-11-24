#!/bin/bash
# Script to generate autonomous command from .pp file

PP_FILE="${1:-trajectory.pp}"
COMMAND_NAME="${2:-TrajectoryAuto}"

ASSETS_PATH="/home/user/DECODE/TeamCode/src/main/assets"
OUTPUT_PATH="/home/user/DECODE/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/generated"

echo "Generating autonomous command from $PP_FILE..."
echo "Command name: $COMMAND_NAME"
echo ""

# Compile and run the generator
cd /home/user/DECODE
javac -cp ".:TeamCode/src/main/java:FtcRobotController/build/intermediates/javac/debug/classes" \
    -d /tmp/ppgen \
    TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/PpFileCommandGenerator.java

java -cp "/tmp/ppgen:." \
    org.firstinspires.ftc.teamcode.util.PpFileCommandGenerator \
    "$PP_FILE" "$COMMAND_NAME" "$ASSETS_PATH" "$OUTPUT_PATH"

echo ""
echo "Generated files in: $OUTPUT_PATH"
ls -lh "$OUTPUT_PATH/${COMMAND_NAME}"*.java
