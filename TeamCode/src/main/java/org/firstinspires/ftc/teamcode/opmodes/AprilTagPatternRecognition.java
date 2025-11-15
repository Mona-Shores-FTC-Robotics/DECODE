package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;

/**
 * OpMode that recognizes AprilTag IDs for DECODE patterns and allows operator
 * to offset the pattern using dpad controls.
 *
 * AprilTag ID Mappings:
 * - Tag 21: GPP (Green Purple Purple)
 * - Tag 22: PGP (Purple Green Purple)
 * - Tag 23: PPG (Purple Purple Green)
 *
 * Controls:
 * - Dpad Left: Offset = 0 (no offset)
 * - Dpad Up: Offset = 1
 * - Dpad Right: Offset = 2
 */
@TeleOp(name = "AprilTag Pattern Recognition", group = "Vision")
public class AprilTagPatternRecognition extends LinearOpMode {

    private VisionSubsystemLimelight vision;
    private int patternOffset = 0;

    // Pattern definitions
    private static final String[] BASE_PATTERNS = {"GPP", "PGP", "PPG"};

    @Override
    public void runOpMode() {
        // Initialize vision subsystem
        vision = new VisionSubsystemLimelight(hardwareMap, telemetry);
        vision.setAlliance(Alliance.UNKNOWN); // Detect all tags
        vision.initialize();

        telemetry.addLine("AprilTag Pattern Recognition Ready");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  Dpad Left: Offset = 0");
        telemetry.addLine("  Dpad Up: Offset = 1");
        telemetry.addLine("  Dpad Right: Offset = 2");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update vision subsystem
            vision.periodic();

            // Handle dpad input for pattern offset
            handleDpadInput();

            // Get current AprilTag ID
            int currentTagId = vision.getCurrentTagId();

            // Update telemetry
            updateTelemetry(currentTagId);
        }

        // Stop vision when done
        vision.stop();
    }

    /**
     * Handles dpad input from gamepad2 (operator controller)
     */
    private void handleDpadInput() {
        if (gamepad2.dpad_left) {
            patternOffset = 0;
        } else if (gamepad2.dpad_up) {
            patternOffset = 1;
        } else if (gamepad2.dpad_right) {
            patternOffset = 2;
        }
    }

    /**
     * Updates telemetry with current AprilTag detection and pattern information
     */
    private void updateTelemetry(int tagId) {
        telemetry.addLine("=== APRILTAG PATTERN RECOGNITION ===");
        telemetry.addLine();

        // Display current AprilTag detection
        if (tagId == -1) {
            telemetry.addData("AprilTag", "No tag detected");
            telemetry.addData("Base Pattern", "---");
        } else {
            telemetry.addData("AprilTag ID", tagId);

            // Map AprilTag ID to base pattern
            String basePattern = getBasePattern(tagId);
            telemetry.addData("Base Pattern", basePattern);

            // Calculate and display final pattern with offset
            if (!basePattern.equals("Unknown")) {
                String finalPattern = applyOffset(basePattern, patternOffset);
                telemetry.addData("Pattern Offset", patternOffset);
                telemetry.addData("Final Pattern", finalPattern);
            }
        }

        telemetry.addLine();
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addData("Current Offset", patternOffset);
        telemetry.addLine("Dpad Left = 0 | Dpad Up = 1 | Dpad Right = 2");
        telemetry.addLine();

        // Display vision status
        telemetry.addData("Vision State", vision.getState());
        telemetry.addData("Has Valid Tag", vision.hasValidTag());

        telemetry.update();
    }

    /**
     * Maps AprilTag ID to base pattern name
     */
    private String getBasePattern(int tagId) {
        if (tagId == FieldConstants.DECODE_PATTERN_GREEN_PURPLE_PURPLE_ID) {
            return "GPP";
        } else if (tagId == FieldConstants.DECODE_PATTERN_PURPLE_GREEN_PURPLE_ID) {
            return "PGP";
        } else if (tagId == FieldConstants.DECODE_PATTERN_PURPLE_PURPLE_GREEN_ID) {
            return "PPG";
        } else {
            return "Unknown";
        }
    }

    /**
     * Applies the pattern offset by rotating the pattern string
     * @param basePattern The base pattern (e.g., "GPP")
     * @param offset The offset (0, 1, or 2)
     * @return The rotated pattern
     */
    private String applyOffset(String basePattern, int offset) {
        if (basePattern.length() != 3) {
            return basePattern;
        }

        // Normalize offset to 0-2 range
        int normalizedOffset = offset % 3;

        // Rotate the pattern string by the offset
        // offset 0: no change
        // offset 1: rotate right by 1 (GPP -> PGP)
        // offset 2: rotate right by 2 (GPP -> PPG)
        char[] chars = basePattern.toCharArray();
        char[] result = new char[3];

        for (int i = 0; i < 3; i++) {
            result[(i + normalizedOffset) % 3] = chars[i];
        }

        return new String(result);
    }
}
