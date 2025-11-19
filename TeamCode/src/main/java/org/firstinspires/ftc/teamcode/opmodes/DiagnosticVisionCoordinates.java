package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.AutoField;

/**
 * Diagnostic OpMode to see RAW Limelight output and understand coordinate transformations.
 *
 * How to use:
 * 1. Place robot at a KNOWN position on the field
 * 2. Make sure AprilTag is visible
 * 3. Run this OpMode
 * 4. Record the output values
 *
 * Test positions:
 * - Field center: Pedro (72, 72)
 * - Blue basket: Pedro (7, 140)
 * - Red basket: Pedro (137, 140)
 * - Blue corner (origin): Pedro (0, 0)
 * - Red corner: Pedro (144, 0)
 */
@TeleOp(name = "Diagnostic: Vision Coordinates", group = "Diagnostic")
public class DiagnosticVisionCoordinates extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Limelight3A limelight;
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize Limelight");
            telemetry.addData("Message", e.getMessage());
            telemetry.update();
            return;
        }

        telemetry.addData("Status", "Initialized - waiting for start");
        telemetry.addData("Instructions", "Place robot at KNOWN field position");
        telemetry.addData("", "Make sure AprilTag is visible");
        telemetry.update();

        waitForStart();

        double halfField = AutoField.waypoints.fieldWidthIn / 2.0;

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                telemetry.addData("Status", "NO APRILTAG VISIBLE");
                telemetry.addData("", "");
                telemetry.addData("Tip", "Point robot at AprilTag");
                telemetry.update();
                sleep(100);
                continue;
            }

            Pose3D botpose = result.getBotpose();
            Pose3D botpose_mt2 = result.getBotpose_MT2();

            telemetry.addData("=== RAW LIMELIGHT OUTPUT ===", "");
            telemetry.addData("", "");

            // Show raw botpose (regular MegaTag)
            if (botpose != null && botpose.getPosition() != null) {
                double xMeters = botpose.getPosition().x;
                double yMeters = botpose.getPosition().y;
                double xInches = DistanceUnit.METER.toInches(xMeters);
                double yInches = DistanceUnit.METER.toInches(yMeters);
                double headingDeg = botpose.getOrientation() == null ? 0 : botpose.getOrientation().getYaw();

                telemetry.addData("Botpose X (m)", "%.3f", xMeters);
                telemetry.addData("Botpose Y (m)", "%.3f", yMeters);
                telemetry.addData("Botpose X (in)", "%.1f", xInches);
                telemetry.addData("Botpose Y (in)", "%.1f", yInches);
                telemetry.addData("Botpose Heading (deg)", "%.1f", headingDeg);
                telemetry.addData("", "");

                // Show what current code does (swaps X/Y + 90° rotation)
                telemetry.addData("=== CURRENT TRANSFORM ===", "");
                double currentPedroX = yInches + halfField;  // Swaps Y to X
                double currentPedroY = halfField - xInches;  // Swaps X to Y (inverted)
                double currentPedroHeading = AngleUnit.normalizeDegrees(headingDeg + 90.0);
                telemetry.addData("Current Pedro X", "%.1f", currentPedroX);
                telemetry.addData("Current Pedro Y", "%.1f", currentPedroY);
                telemetry.addData("Current Pedro Heading", "%.1f", currentPedroHeading);
                telemetry.addData("", "");

                // Show what simple transform would give (no swap, no rotation)
                telemetry.addData("=== SIMPLE TRANSFORM ===", "");
                double simplePedroX = halfField - xInches;  // FTC +X toward red = Pedro low X
                double simplePedroY = yInches + halfField;  // FTC +Y toward far = Pedro high Y
                double simplePedroHeading = headingDeg;  // No rotation
                telemetry.addData("Simple Pedro X", "%.1f", simplePedroX);
                telemetry.addData("Simple Pedro Y", "%.1f", simplePedroY);
                telemetry.addData("Simple Pedro Heading", "%.1f", simplePedroHeading);
                telemetry.addData("", "");

                // Show expected positions for reference
                telemetry.addData("=== REFERENCE ===", "");
                telemetry.addData("Field center", "Pedro (72, 72)");
                telemetry.addData("Blue basket", "Pedro (7, 140)");
                telemetry.addData("Red basket", "Pedro (137, 140)");

            } else {
                telemetry.addData("Botpose", "NULL or invalid");
            }

            telemetry.addData("", "");
            telemetry.addData("Press STOP to exit", "");
            telemetry.update();

            sleep(100);
        }

        limelight.stop();
    }
}
