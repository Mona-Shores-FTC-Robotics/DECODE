package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Diagnostic OpMode to debug MegaTag2 heading transformation issues.
 *
 * INSTRUCTIONS:
 * 1. Place robot at a known position on the field
 * 2. Run this OpMode
 * 3. Adjust headingOffsetDeg via FTC Dashboard until MT2 pose matches odometry
 * 4. Record the working offset value
 *
 * The correct offset will make "MT2 Pedro Pose" match "Odometry Pose" closely.
 */
@Config
@TeleOp(name = "Diagnose MegaTag2", group = "Diagnostics")
public class DiagnoseMegaTag2 extends LinearOpMode {

    /**
     * Heading offset to add to Pedro heading before sending to Limelight.
     * Adjust this via FTC Dashboard to find the correct value.
     *
     * Try these values:
     * - 0° = Send Pedro heading directly
     * - 90° = Pedro + 90 (FTC heading)
     * - 180° = Pedro + 180 (flip)
     * - 270° = Pedro + 270 (FTC - 90)
     * - -90° = Pedro - 90
     */
    public static double headingOffsetDeg = 0.0;

    private Follower follower;
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        // Set up telemetry to show on both Driver Station and FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Pedro follower for odometry
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0)); // Default start position

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("=== MegaTag2 Diagnostic OpMode ===");
        telemetry.addLine();
        telemetry.addLine("1. Place robot at known position");
        telemetry.addLine("2. Press PLAY");
        telemetry.addLine("3. Adjust 'headingOffsetDeg' in FTC Dashboard");
        telemetry.addLine("4. Find offset where MT2 pose matches odometry");
        telemetry.addLine();
        telemetry.addLine("Press PLAY to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Update odometry
            follower.update();
            Pose odomPose = follower.getPose();

            // Get Pedro heading and calculate various heading options
            double pedroHeadingRad = odomPose.getHeading();
            double pedroHeadingDeg = Math.toDegrees(pedroHeadingRad);

            // Calculate heading to send to Limelight (Pedro + offset)
            double headingToSendDeg = AngleUnit.normalizeDegrees(pedroHeadingDeg + headingOffsetDeg);

            // Send heading to Limelight for MT2 fusion
            limelight.updateRobotOrientation(headingToSendDeg);

            // Get MT2 result
            LLResult result = limelight.getLatestResult();

            // Clear telemetry
            telemetry.addLine("=== MegaTag2 Diagnostic ===");
            telemetry.addLine();

            // Show heading calculations
            telemetry.addLine("--- HEADING DEBUG ---");
            telemetry.addData("Pedro Heading", "%.1f°", pedroHeadingDeg);
            telemetry.addData("FTC Heading (Pedro+90)", "%.1f°", AngleUnit.normalizeDegrees(pedroHeadingDeg + 90));
            telemetry.addData("Current Offset", "%.1f°", headingOffsetDeg);
            telemetry.addData("Sent to Limelight", "%.1f°", headingToSendDeg);
            telemetry.addLine();

            // Show odometry pose
            telemetry.addLine("--- ODOMETRY (Ground Truth) ---");
            telemetry.addData("Pedro Pose", "(%.1f, %.1f, %.1f°)",
                odomPose.getX(), odomPose.getY(), pedroHeadingDeg);
            telemetry.addData("FTC Pose", "(%.1f, %.1f, %.1f°)",
                pedroToFtcX(odomPose), pedroToFtcY(odomPose), AngleUnit.normalizeDegrees(pedroHeadingDeg + 90));
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

                    // Convert MT2 FTC pose to Pedro
                    double mt2PedroX = mt2YIn + 72; // ftcY + halfField
                    double mt2PedroY = 72 - mt2XIn; // halfField - ftcX
                    double mt2PedroHeading = AngleUnit.normalizeDegrees(mt2YawDeg - 90);

                    telemetry.addData("MT2 Pedro Pose", "(%.1f, %.1f, %.1f°)",
                        mt2PedroX, mt2PedroY, mt2PedroHeading);
                    telemetry.addLine();

                    // Calculate errors
                    double errorX = mt2PedroX - odomPose.getX();
                    double errorY = mt2PedroY - odomPose.getY();
                    double errorHeading = AngleUnit.normalizeDegrees(mt2PedroHeading - pedroHeadingDeg);
                    double errorDistance = Math.hypot(errorX, errorY);

                    telemetry.addLine("--- ERROR (MT2 - Odometry) ---");
                    telemetry.addData("Position Error", "%.1f in (X: %.1f, Y: %.1f)",
                        errorDistance, errorX, errorY);
                    telemetry.addData("Heading Error", "%.1f°", errorHeading);

                    // Provide guidance
                    telemetry.addLine();
                    if (errorDistance < 3.0 && Math.abs(errorHeading) < 5.0) {
                        telemetry.addLine("✓ GOOD! Offset looks correct!");
                        telemetry.addData("Record this value", "%.1f°", headingOffsetDeg);
                    } else {
                        telemetry.addLine("✗ Adjust headingOffsetDeg in FTC Dashboard");
                        if (Math.abs(errorHeading) > 45) {
                            telemetry.addLine("  Try: 0°, 90°, 180°, or 270°");
                        }
                    }

                    // Show AprilTag info
                    telemetry.addLine();
                    telemetry.addData("Tag ID", result.getFiducialResults().size() > 0 ?
                        result.getFiducialResults().get(0).getFiducialId() : "None");
                    telemetry.addData("Tags Visible", result.getFiducialResults().size());

                } else {
                    telemetry.addLine("--- MEGATAG2 RESULT ---");
                    telemetry.addLine("MT2 pose is null");
                }
            } else {
                telemetry.addLine("--- MEGATAG2 RESULT ---");
                telemetry.addLine("No valid Limelight result");
                telemetry.addLine("Make sure you can see AprilTag 20 or 24");
            }

            telemetry.addLine();
            telemetry.addLine("Adjust 'headingOffsetDeg' in FTC Dashboard Config");
            telemetry.update();

            sleep(100); // Update at 10 Hz
        }

        limelight.stop();
    }

    private double pedroToFtcX(Pose pedroPose) {
        return 72 - pedroPose.getY();
    }

    private double pedroToFtcY(Pose pedroPose) {
        return pedroPose.getX() - 72;
    }
}
