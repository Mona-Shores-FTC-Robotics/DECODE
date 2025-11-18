package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotState;

/**
 * Diagnostic OpMode to debug MegaTag2 heading transformation issues.
 *
 * INSTRUCTIONS:
 * 1. Place robot at a known position on the field (write it down!)
 * 2. Make sure you can see AprilTag 20 (blue basket) or 24 (red basket)
 * 3. Run this OpMode
 * 4. Open FTC Dashboard: http://192.168.49.1:8080/dash
 * 5. Go to Config tab → DiagnoseMegaTag2 → headingOffsetDeg
 * 6. Try these values: 0, 90, 180, 270, -90
 * 7. Find the offset where MT2 pose matches odometry (error < 3 inches)
 * 8. RECORD THAT VALUE!
 *
 * The correct offset will make "MT2 Pedro Pose" match "Odometry Pose" closely.
 */
@Config
@TeleOp(name = "Diagnose MegaTag2", group = "Diagnostics")
public class DiagnoseMegaTag2 extends NextFTCOpMode {

    /**
     * Heading offset to add to Pedro heading before sending to Limelight.
     * Adjust this via FTC Dashboard to find the correct value.
     *
     * Try these values in order:
     * - 0° = Send Pedro heading directly (camera handles everything)
     * - 90° = Pedro + 90 (FTC heading convention)
     * - 180° = Pedro + 180 (flip for camera orientation)
     * - 270° = Pedro + 270 (FTC + 180)
     * - -90° = Pedro - 90 (inverse FTC conversion)
     */
    public static double headingOffsetDeg = 0.0;

    private Robot robot;

    {
        addComponents(
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        robot = new Robot(hardwareMap);
        robot.attachPedroFollower();
        robot.setAlliance(Alliance.BLUE); // Default for testing
        robot.initializeForTeleOp();

        addComponents(
                new SubsystemComponent(robot.drive),
                new SubsystemComponent(robot.vision)
        );

        telemetry.addLine("=== MegaTag2 Diagnostic OpMode ===");
        telemetry.addLine();
        telemetry.addLine("1. Position robot at known location");
        telemetry.addLine("2. Ensure AprilTag 20 or 24 is visible");
        telemetry.addLine("3. Press PLAY");
        telemetry.addLine("4. Adjust headingOffsetDeg in FTC Dashboard");
        telemetry.addLine("5. Find offset where poses match");
        telemetry.addLine();
        telemetry.addData("Current Offset", "%.1f°", headingOffsetDeg);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Nothing special needed on start
    }

    @Override
    public void onUpdate() {
        // Get current odometry pose (ground truth)
        Pose odomPose = robot.drive.getFollower().getPose();
        double pedroHeadingRad = odomPose.getHeading();
        double pedroHeadingDeg = Math.toDegrees(pedroHeadingRad);

        // Calculate heading to send to Limelight using current offset
        double headingToSendDeg = AngleUnit.normalizeDegrees(pedroHeadingDeg + headingOffsetDeg);

        // Override the vision subsystem's heading with our test offset
        // (normally this happens in DriveSubsystem, but we're testing different offsets)
        double headingToSendRad = Math.toRadians(headingToSendDeg);
        robot.vision.limelight.updateRobotOrientation(headingToSendDeg);

        // Log heading data to AdvantageScope
        RobotState.packet.put("Diagnostic/pedroHeadingDeg", pedroHeadingDeg);
        RobotState.packet.put("Diagnostic/ftcHeadingDeg", AngleUnit.normalizeDegrees(pedroHeadingDeg + 90));
        RobotState.packet.put("Diagnostic/currentOffsetDeg", headingOffsetDeg);
        RobotState.packet.put("Diagnostic/sentToLimelightDeg", headingToSendDeg);

        // Log odometry pose to AdvantageScope
        RobotState.packet.put("Diagnostic/Odom/pedroX", odomPose.getX());
        RobotState.packet.put("Diagnostic/Odom/pedroY", odomPose.getY());
        RobotState.packet.put("Diagnostic/Odom/pedroHeading", pedroHeadingDeg);

        // Get MT2 result
        LLResult result = robot.vision.limelight.getLatestResult();

        // Clear telemetry and show diagnostic info
        telemetry.addLine("=== MegaTag2 Diagnostic ===");
        telemetry.addLine();

        // Show heading calculations
        telemetry.addLine("--- HEADING DEBUG ---");
        telemetry.addData("Pedro Heading", "%.1f°", pedroHeadingDeg);
        telemetry.addData("FTC Heading (Pedro+90)", "%.1f°", AngleUnit.normalizeDegrees(pedroHeadingDeg + 90));
        telemetry.addData("Current Offset", "%.1f° (adjust in Dashboard)", headingOffsetDeg);
        telemetry.addData("Sent to Limelight", "%.1f°", headingToSendDeg);
        telemetry.addLine();

        // Show odometry pose (ground truth)
        telemetry.addLine("--- ODOMETRY (Ground Truth) ---");
        telemetry.addData("Pedro Pose", "(%.1f, %.1f, %.1f°)",
            odomPose.getX(), odomPose.getY(), pedroHeadingDeg);
        double ftcX = 72 - odomPose.getY();
        double ftcY = odomPose.getX() - 72;
        double ftcHeading = AngleUnit.normalizeDegrees(pedroHeadingDeg + 90);
        telemetry.addData("FTC Pose", "(%.1f, %.1f, %.1f°)", ftcX, ftcY, ftcHeading);
        telemetry.addLine();

        // Show MT2 result
        if (result != null && result.isValid()) {
            Pose3D mt2Pose = result.getBotpose_MT2();

            if (mt2Pose != null && mt2Pose.getPosition() != null) {
                double mt2XIn = DistanceUnit.METER.toInches(mt2Pose.getPosition().x);
                double mt2YIn = DistanceUnit.METER.toInches(mt2Pose.getPosition().y);
                double mt2YawDeg = mt2Pose.getOrientation() != null ?
                    mt2Pose.getOrientation().getYaw(AngleUnit.DEGREES) : Double.NaN;

                telemetry.addLine("--- MEGATAG2 RESULT ---");
                telemetry.addData("Raw MT2 (FTC)", "(%.1f, %.1f, %.1f°)",
                    mt2XIn, mt2YIn, mt2YawDeg);

                // Log MT2 raw (FTC) data to AdvantageScope
                RobotState.packet.put("Diagnostic/MT2/ftcX", mt2XIn);
                RobotState.packet.put("Diagnostic/MT2/ftcY", mt2YIn);
                RobotState.packet.put("Diagnostic/MT2/ftcHeading", mt2YawDeg);

                // Convert MT2 FTC pose to Pedro
                double mt2PedroX = mt2YIn + 72; // ftcY + halfField
                double mt2PedroY = 72 - mt2XIn; // halfField - ftcX
                double mt2PedroHeading = AngleUnit.normalizeDegrees(mt2YawDeg - 90);

                telemetry.addData("MT2 Pedro Pose", "(%.1f, %.1f, %.1f°)",
                    mt2PedroX, mt2PedroY, mt2PedroHeading);
                telemetry.addLine();

                // Log MT2 Pedro data to AdvantageScope
                RobotState.packet.put("Diagnostic/MT2/pedroX", mt2PedroX);
                RobotState.packet.put("Diagnostic/MT2/pedroY", mt2PedroY);
                RobotState.packet.put("Diagnostic/MT2/pedroHeading", mt2PedroHeading);

                // Calculate errors
                double errorX = mt2PedroX - odomPose.getX();
                double errorY = mt2PedroY - odomPose.getY();
                double errorHeading = AngleUnit.normalizeDegrees(mt2PedroHeading - pedroHeadingDeg);
                double errorDistance = Math.hypot(errorX, errorY);

                // Log errors to AdvantageScope
                RobotState.packet.put("Diagnostic/Error/positionInches", errorDistance);
                RobotState.packet.put("Diagnostic/Error/xInches", errorX);
                RobotState.packet.put("Diagnostic/Error/yInches", errorY);
                RobotState.packet.put("Diagnostic/Error/headingDeg", errorHeading);

                telemetry.addLine("--- ERROR (MT2 - Odometry) ---");
                telemetry.addData("Position Error", "%.1f in (X: %.1f, Y: %.1f)",
                    errorDistance, errorX, errorY);
                telemetry.addData("Heading Error", "%.1f°", errorHeading);

                // Provide guidance
                telemetry.addLine();
                boolean offsetIsGood = errorDistance < 3.0 && Math.abs(errorHeading) < 5.0;
                RobotState.packet.put("Diagnostic/offsetIsGood", offsetIsGood);

                if (offsetIsGood) {
                    telemetry.addLine("✓✓✓ GOOD! Offset looks correct! ✓✓✓");
                    telemetry.addLine();
                    telemetry.addData("** RECORD THIS VALUE **", "%.1f°", headingOffsetDeg);
                    telemetry.addLine();
                } else {
                    telemetry.addLine("✗ Adjust headingOffsetDeg in FTC Dashboard");
                    if (Math.abs(errorHeading) > 45) {
                        telemetry.addLine("  Heading way off - try: 0°, 90°, 180°, or 270°");
                    } else if (Math.abs(errorHeading) > 15) {
                        telemetry.addLine("  Close! Try nearby values (±10° to ±30°)");
                    }
                    if (errorDistance > 50) {
                        telemetry.addLine("  Position way off - check coordinate conversion");
                    }
                }

                // Show AprilTag info
                telemetry.addLine();
                if (!result.getFiducialResults().isEmpty()) {
                    int tagId = result.getFiducialResults().get(0).getFiducialId();
                    int tagsVisible = result.getFiducialResults().size();
                    double tx = result.getTx();

                    telemetry.addData("Tag ID", tagId);
                    telemetry.addData("Tags Visible", tagsVisible);
                    telemetry.addData("tx", "%.2f°", tx);

                    // Log AprilTag info to AdvantageScope
                    RobotState.packet.put("Diagnostic/AprilTag/tagId", tagId);
                    RobotState.packet.put("Diagnostic/AprilTag/tagsVisible", tagsVisible);
                    RobotState.packet.put("Diagnostic/AprilTag/txDeg", tx);
                }

            } else {
                telemetry.addLine("--- MEGATAG2 RESULT ---");
                telemetry.addLine("MT2 pose is null");
                telemetry.addLine("Check Limelight web UI:");
                telemetry.addLine("  - MegaTag2 enabled?");
                telemetry.addLine("  - Field map uploaded?");
                telemetry.addLine("  - Robot-to-camera offsets set?");
            }
        } else {
            telemetry.addLine("--- MEGATAG2 RESULT ---");
            telemetry.addLine("No valid Limelight result");
            telemetry.addLine("Make sure you can see AprilTag 20 or 24!");
            telemetry.addLine("(Blue basket or Red basket tags)");
        }

        telemetry.addLine();
        telemetry.addLine("Go to FTC Dashboard to adjust offset:");
        telemetry.addLine("http://192.168.49.1:8080/dash → Config");

        telemetry.update();
    }

    @Override
    public void onStop() {
        if (robot != null) {
            robot.drive.stop();
        }
    }
}
