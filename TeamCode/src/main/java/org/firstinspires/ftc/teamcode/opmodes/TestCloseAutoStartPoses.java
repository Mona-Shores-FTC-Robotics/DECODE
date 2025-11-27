package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;

import java.util.Optional;

/**
 * Test OpMode to find optimal Close Auto starting poses that:
 * 1. Can be aligned with physical field structures
 * 2. Can see the opposite alliance's basket AprilTag
 * 3. Provide reliable vision-based initialization
 *
 * Use D-pad to test different candidate poses:
 * - D-pad Up: Back-left corner alignment (X=24, Y=138)
 * - D-pad Right: Current default (X=26.4, Y=131.4)
 * - D-pad Down: Tile intersection (X=24, Y=120)
 * - D-pad Left: Far from wall (X=30, Y=130)
 *
 * For each pose, displays:
 * - Whether opposite basket tag is visible
 * - Tag ID detected
 * - Vision-calculated pose vs test pose
 * - Distance from test pose to vision pose
 */
@Autonomous(name = "Test Close Auto Start Poses", group = "Test")
@Configurable
public class TestCloseAutoStartPoses extends LinearOpMode {

    @Configurable
    public static class TestPoseConfig {
        // Candidate 1: Back-left corner (easy to align, close to wall)
        public double corner_X = 24.0;
        public double corner_Y = 138.0;
        public double corner_heading = 144.0;

        // Candidate 2: Current default
        public double default_X = 26.445;
        public double default_Y = 131.374;
        public double default_heading = 144.0;

        // Candidate 3: Tile intersection
        public double tile_X = 24.0;
        public double tile_Y = 120.0;
        public double tile_heading = 144.0;

        // Candidate 4: Further from wall
        public double far_X = 30.0;
        public double far_Y = 130.0;
        public double far_heading = 144.0;
    }

    public static TestPoseConfig config = new TestPoseConfig();

    private Robot robot;
    private Alliance testAlliance = Alliance.BLUE;
    private int selectedPoseIndex = 0;
    private final String[] poseNames = {
        "Corner (easy align)",
        "Default (current)",
        "Tile intersection",
        "Far from wall"
    };

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.setAlliance(testAlliance);

        telemetry.addLine("=== Close Auto Start Pose Tester ===");
        telemetry.addLine();
        telemetry.addLine("Instructions:");
        telemetry.addLine("1. Place robot at candidate position");
        telemetry.addLine("2. D-pad Up/Down to cycle test poses");
        telemetry.addLine("3. Check if AprilTag is visible");
        telemetry.addLine("4. Press START to test vision pose");
        telemetry.addLine();
        telemetry.addLine("Waiting for START...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Cycle through poses
            if (gamepad1.dpad_up && !gamepad1.dpad_down) {
                selectedPoseIndex = 0;
                sleep(200);
            } else if (gamepad1.dpad_right) {
                selectedPoseIndex = 1;
                sleep(200);
            } else if (gamepad1.dpad_down && !gamepad1.dpad_up) {
                selectedPoseIndex = 2;
                sleep(200);
            } else if (gamepad1.dpad_left) {
                selectedPoseIndex = 3;
                sleep(200);
            }

            // Get current test pose
            Pose testPose = getTestPose(selectedPoseIndex);

            // Update vision
            robot.vision.periodic();

            // Check vision detection
            Optional<VisionSubsystemLimelight.TagSnapshot> snapshotOpt = robot.vision.getLastSnapshot();

            telemetry.clear();
            telemetry.addLine("=== Close Auto Start Pose Tester ===");
            telemetry.addLine();
            telemetry.addData("Alliance", testAlliance.displayName());
            telemetry.addData("Test Pose", "%d: %s", selectedPoseIndex + 1, poseNames[selectedPoseIndex]);
            telemetry.addLine();

            telemetry.addLine("--- Expected Test Pose ---");
            telemetry.addData("Position", "X=%.1f Y=%.1f", testPose.getX(), testPose.getY());
            telemetry.addData("Heading", "%.0f°", Math.toDegrees(testPose.getHeading()));
            telemetry.addLine();

            int expectedTag = (testAlliance == Alliance.BLUE)
                ? FieldConstants.RED_GOAL_TAG_ID
                : FieldConstants.BLUE_GOAL_TAG_ID;

            if (snapshotOpt.isPresent()) {
                VisionSubsystemLimelight.TagSnapshot snapshot = snapshotOpt.get();
                int detectedTag = snapshot.getTagId();

                telemetry.addLine("--- Vision Detection ---");
                telemetry.addData("Tag Visible", "✓ YES");
                telemetry.addData("Tag ID", "%d %s", detectedTag,
                    detectedTag == expectedTag ? "✓ CORRECT" : "⚠ WRONG");

                Optional<Pose> visionPoseMT1 = snapshot.getRobotPosePedroMT1();
                Optional<Pose> visionPoseMT2 = snapshot.getRobotPosePeroMT2();

                if (visionPoseMT1.isPresent()) {
                    Pose vPose = visionPoseMT1.get();
                    telemetry.addLine();
                    telemetry.addLine("--- Vision MT1 Pose ---");
                    telemetry.addData("Position", "X=%.1f Y=%.1f", vPose.getX(), vPose.getY());
                    telemetry.addData("Heading", "%.0f°", Math.toDegrees(vPose.getHeading()));

                    double deltaX = vPose.getX() - testPose.getX();
                    double deltaY = vPose.getY() - testPose.getY();
                    double distance = Math.hypot(deltaX, deltaY);
                    double headingError = Math.toDegrees(vPose.getHeading() - testPose.getHeading());

                    telemetry.addLine();
                    telemetry.addLine("--- Error Analysis ---");
                    telemetry.addData("Position Error", "%.1f inches", distance);
                    telemetry.addData("ΔX", "%.1f in", deltaX);
                    telemetry.addData("ΔY", "%.1f in", deltaY);
                    telemetry.addData("Heading Error", "%.1f°", headingError);

                    String quality = distance < 3.0 ? "✓ EXCELLENT" :
                                   distance < 6.0 ? "✓ GOOD" :
                                   distance < 12.0 ? "⚠ OK" : "✗ POOR";
                    telemetry.addData("Quality", quality);
                }

                if (visionPoseMT2.isPresent()) {
                    Pose vPose = visionPoseMT2.get();
                    telemetry.addLine();
                    telemetry.addLine("--- Vision MT2 Pose ---");
                    telemetry.addData("Position", "X=%.1f Y=%.1f", vPose.getX(), vPose.getY());
                    telemetry.addData("Heading", "%.0f°", Math.toDegrees(vPose.getHeading()));
                }

                // Show MT1/MT2 agreement
                if (visionPoseMT1.isPresent() && visionPoseMT2.isPresent()) {
                    double dx = visionPoseMT1.get().getX() - visionPoseMT2.get().getX();
                    double dy = visionPoseMT1.get().getY() - visionPoseMT2.get().getY();
                    double mt1mt2Distance = Math.hypot(dx, dy);

                    telemetry.addLine();
                    telemetry.addData("MT1/MT2 Agreement", "%.1f inches", mt1mt2Distance);
                    String agreement = mt1mt2Distance < 12.0 ? "✓ GOOD (use MT2)" : "⚠ POOR (use MT1)";
                    telemetry.addData("Reliability", agreement);
                }

            } else {
                telemetry.addLine("--- Vision Detection ---");
                telemetry.addData("Tag Visible", "✗ NO");
                telemetry.addData("Expected Tag", "%d (%s basket)", expectedTag,
                    testAlliance == Alliance.BLUE ? "RED" : "BLUE");
                telemetry.addLine();
                telemetry.addLine("⚠ Adjust robot position/angle");
                telemetry.addLine("⚠ Check Limelight connection");
            }

            telemetry.addLine();
            telemetry.addLine("--- Controls ---");
            telemetry.addLine("D-pad: Select test pose");
            telemetry.addLine("A: Toggle alliance");

            if (gamepad1.a) {
                testAlliance = (testAlliance == Alliance.BLUE) ? Alliance.RED : Alliance.BLUE;
                robot.setAlliance(testAlliance);
                sleep(200);
            }

            telemetry.update();
            sleep(100);
        }
    }

    private Pose getTestPose(int index) {
        switch (index) {
            case 0:
                return new Pose(config.corner_X, config.corner_Y, Math.toRadians(config.corner_heading));
            case 1:
                return new Pose(config.default_X, config.default_Y, Math.toRadians(config.default_heading));
            case 2:
                return new Pose(config.tile_X, config.tile_Y, Math.toRadians(config.tile_heading));
            case 3:
                return new Pose(config.far_X, config.far_Y, Math.toRadians(config.far_heading));
            default:
                return new Pose(config.default_X, config.default_Y, Math.toRadians(config.default_heading));
        }
    }
}
