package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;

/**
 * OpMode that recognizes AprilTag IDs for DECODE patterns and allows operator
 * to offset the pattern using dpad controls. Includes drive functionality to
 * move the robot to see different obelisk patterns.
 *
 * AprilTag ID Mappings:
 * - Tag 21: GPP (Green Purple Purple)
 * - Tag 22: PGP (Purple Green Purple)
 * - Tag 23: PPG (Purple Purple Green)
 *
 * Controls:
 * Gamepad1 (Driver):
 * - Left Stick: Translation (forward/backward, left/right)
 * - Right Stick X: Rotation
 * - Right Bumper: Slow mode
 *
 * Gamepad2 (Operator):
 * - Dpad Left: Offset = 0 (no offset)
 * - Dpad Up: Offset = 1
 * - Dpad Right: Offset = 2
 */
@Disabled
@TeleOp(name = "AprilTag Pattern Recognition", group = "Vision")
public class AprilTagPatternRecognition extends LinearOpMode {

    private VisionSubsystemLimelight vision;
    private int patternOffset = 0;
    private int lastSeenTagId = -1;

    // Drive motors
    private DcMotorEx motorLf;
    private DcMotorEx motorRf;
    private DcMotorEx motorLb;
    private DcMotorEx motorRb;

    // Drive constants
    private static final double NORMAL_SPEED = 1.0;
    private static final double SLOW_SPEED = 0.3;
    private static final double DEADBAND = 0.05;

    @Override
    public void runOpMode() {
        // Initialize vision subsystem
        vision = new VisionSubsystemLimelight(hardwareMap, telemetry);
        vision.setAlliance(Alliance.UNKNOWN); // Detect all tags
        vision.initialize();

        // Initialize drive motors
        initializeDriveMotors();

        telemetry.addLine("AprilTag Pattern Recognition Ready");
        telemetry.addLine();
        telemetry.addLine("Gamepad1 Controls (Driver):");
        telemetry.addLine("  Left Stick: Move robot");
        telemetry.addLine("  Right Stick X: Rotate");
        telemetry.addLine("  Right Bumper: Slow mode");
        telemetry.addLine();
        telemetry.addLine("Gamepad2 Controls (Operator):");
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

            // Handle driver input for driving
            handleDriveInput();

            // Handle operator dpad input for pattern offset
            handleDpadInput();

            // Get current AprilTag ID
            int currentTagId = vision.getCurrentTagId();

            // Track last seen tag
            if (currentTagId != -1) {
                lastSeenTagId = currentTagId;
            }

            // Update telemetry
            updateTelemetry(currentTagId);
        }

        // Stop motors and vision when done
        stopAllMotors();
        vision.stop();
    }

    /**
     * Initializes drive motors from hardware map
     */
    private void initializeDriveMotors() {
        motorLf = hardwareMap.get(DcMotorEx.class, Constants.HardwareNames.LF);
        motorRf = hardwareMap.get(DcMotorEx.class, Constants.HardwareNames.RF);
        motorLb = hardwareMap.get(DcMotorEx.class, Constants.HardwareNames.LB);
        motorRb = hardwareMap.get(DcMotorEx.class, Constants.HardwareNames.RB);

        // Set motor directions (left side reversed for mecanum)
        motorLf.setDirection(DcMotor.Direction.REVERSE);
        motorRf.setDirection(DcMotor.Direction.FORWARD);
        motorLb.setDirection(DcMotor.Direction.REVERSE);
        motorRb.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior
        motorLf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Handles drive input from gamepad1 (driver controller)
     */
    private void handleDriveInput() {
        // Get gamepad inputs with deadband
        double y = applyDeadband(-gamepad1.left_stick_y);  // Forward/backward (inverted)
        double x = applyDeadband(gamepad1.left_stick_x);   // Strafe left/right
        double rx = applyDeadband(gamepad1.right_stick_x); // Rotation

        // Apply slow mode if right bumper is pressed
        double speedMultiplier = gamepad1.right_bumper ? SLOW_SPEED : NORMAL_SPEED;

        // Calculate motor powers for mecanum drive
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double frontLeftPower = (y + x + rx) / denominator * speedMultiplier;
        double backLeftPower = (y - x + rx) / denominator * speedMultiplier;
        double frontRightPower = (y - x - rx) / denominator * speedMultiplier;
        double backRightPower = (y + x - rx) / denominator * speedMultiplier;

        // Set motor powers
        motorLf.setPower(frontLeftPower);
        motorLb.setPower(backLeftPower);
        motorRf.setPower(frontRightPower);
        motorRb.setPower(backRightPower);
    }

    /**
     * Applies deadband to gamepad input
     */
    private double applyDeadband(double value) {
        return Math.abs(value) < DEADBAND ? 0.0 : value;
    }

    /**
     * Stops all drive motors
     */
    private void stopAllMotors() {
        motorLf.setPower(0);
        motorRf.setPower(0);
        motorLb.setPower(0);
        motorRb.setPower(0);
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
        telemetry.addLine("DRIVER (Gamepad1):");
        telemetry.addData("  Drive Mode", gamepad1.right_bumper ? "SLOW" : "NORMAL");
        telemetry.addLine("OPERATOR (Gamepad2):");
        telemetry.addData("  Current Offset", patternOffset);
        telemetry.addLine("  Dpad: Left=0 | Up=1 | Right=2");
        telemetry.addLine();

        // Display last seen tag
        telemetry.addLine("--- LAST SEEN TAG ---");
        if (lastSeenTagId == -1) {
            telemetry.addLine("No tag detected yet");
        } else {
            String lastPattern = getBasePattern(lastSeenTagId);
            telemetry.addData("Last Seen Tag", lastSeenTagId + " (" + lastPattern + ")");
        }
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

        // Rotate the pattern string by the offset (LEFT rotation)
        // offset 0: no change (GPP -> GPP)
        // offset 1: rotate left by 1 (GPP -> PPG)
        // offset 2: rotate left by 2 (GPP -> PGP)
        char[] chars = basePattern.toCharArray();
        char[] result = new char[3];

        for (int i = 0; i < 3; i++) {
            result[i] = chars[(i + normalizedOffset) % 3];
        }

        return new String(result);
    }
}
