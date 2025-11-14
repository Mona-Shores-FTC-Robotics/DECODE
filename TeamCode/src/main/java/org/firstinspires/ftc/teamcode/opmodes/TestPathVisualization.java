package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryService;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldLayout;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldPoint;
import org.firstinspires.ftc.teamcode.util.PoseTransforms;

/**
 * Simple test OpMode for visualizing autonomous paths on test bench
 *
 * This OpMode:
 * - Only requires Control Hub (drive motors)
 * - No expansion hub needed
 * - Shows paths on FTC Dashboard
 * - Useful for testing path planning without full robot
 */
@Autonomous(name = "Test: Path Visualization", group = "Test")
public class TestPathVisualization extends LinearOpMode {

    private Alliance activeAlliance = Alliance.BLUE;
    private FtcDashboard dashboard;
    private TelemetryService telemetryService;

    @Override
    public void runOpMode() throws InterruptedException {


        // Initialize telemetry
        telemetryService = new TelemetryService();
        telemetryService.startSession();

        // Initialize Panels for visualization
        PanelsBridge.preparePanels();

        // Initialize FTC Dashboard for AdvantageScope
        dashboard = FtcDashboard.getInstance();

        // Initialize motors manually (minimal setup)
        DcMotor lf = hardwareMap.get(DcMotor.class, Constants.HardwareNames.LF);
        DcMotor rf = hardwareMap.get(DcMotor.class, Constants.HardwareNames.RF);
        DcMotor lb = hardwareMap.get(DcMotor.class, Constants.HardwareNames.LB);
        DcMotor rb = hardwareMap.get(DcMotor.class, Constants.HardwareNames.RB);

        // Set motor directions
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize follower
        Follower follower = Constants.createFollower(hardwareMap);

        // Set up field layout
        FieldLayout layout = AutoField.layoutForAlliance(activeAlliance);
        Pose startPose = layout.pose(FieldPoint.START_FAR);

        follower.setStartingPose(startPose);

        // Build all paths from DecodeAutonomousFarCommand
        PathChain[] allPaths = DecodeAutonomousFarCommand.buildAllPathsForVisualization(follower, activeAlliance);

        // Use first path for visualization (can cycle through them if needed)
        PathChain currentPath = allPaths[0];

        telemetry.addLine("Path Visualization Test");
        telemetry.addLine("Real-time Simulation with AdvantageScope");
        telemetry.addLine();
        telemetry.addData("CSV Logging", "ENABLED - saves to /RoadRunner/logs/");
        telemetry.addLine();
        telemetry.addData("Alliance", activeAlliance);
        telemetry.addData("Total Paths", allPaths.length);
        telemetry.addLine();
        telemetry.addLine("Path Sequence:");
        telemetry.addLine("  1. Start → Launch");
        telemetry.addLine("  2. Launch → Alliance Wall");
        telemetry.addLine("  3. Alliance Wall → Launch");
        telemetry.addLine("  4. Launch → Parking");
        telemetry.addLine("  5. Parking → Launch");
        telemetry.addLine("  6. Launch → Gate Far");
        telemetry.addLine("  7. Gate Far → Launch");
        telemetry.addLine();
        telemetry.addLine("Gamepad controls:");
        telemetry.addLine("  L Bumper: BLUE | R Bumper: RED");
        telemetry.addLine();
        telemetry.addLine("Visualization:");
        telemetry.addLine("  FTC Dashboard: http://192.168.49.1:8080/dash");
        telemetry.addLine("  AdvantageScope: Connect to robot IP");
        telemetry.update();

        // Start following the first path so it can be drawn
        follower.followPath(currentPath);

        // Wait for start
        while (!isStarted() && !isStopRequested()) {
            // Allow alliance switching during init
            if (gamepad1.left_bumper) {
                activeAlliance = Alliance.BLUE;
                FieldLayout newLayout = AutoField.layoutForAlliance(activeAlliance);
                Pose newStart = newLayout.pose(FieldPoint.START_FAR);

                // Rebuild all paths for new alliance
                allPaths = DecodeAutonomousFarCommand.buildAllPathsForVisualization(follower, activeAlliance);
                currentPath = allPaths[0];

                follower.setStartingPose(newStart);
                follower.followPath(currentPath);
            } else if (gamepad1.right_bumper) {
                activeAlliance = Alliance.RED;
                FieldLayout newLayout = AutoField.layoutForAlliance(activeAlliance);
                Pose newStart = newLayout.pose(FieldPoint.START_FAR);

                // Rebuild all paths for new alliance
                allPaths = DecodeAutonomousFarCommand.buildAllPathsForVisualization(follower, activeAlliance);
                currentPath = allPaths[0];

                follower.setStartingPose(newStart);
                follower.followPath(currentPath);
            }


                       // Update follower (this processes the path)
            follower.update();

            // Draw ALL the paths as preview (not just current)
            PanelsBridge.drawPreview(allPaths, follower.getPose(), activeAlliance == Alliance.RED);

            // Send telemetry packet for AdvantageScope
            Pose currentPose = follower.getPose();
            sendPathSimPacket(currentPose, 0, "Init Preview", false);

            telemetry.addData("Alliance", activeAlliance);
            telemetry.addData("Status", "Initialized - Showing all paths");
            telemetry.addData("Total Paths", allPaths.length);
            telemetry.addData("Pose", String.format("(%.1f, %.1f, %.1f°)",
                    currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));
            telemetry.update();
        }

        // If started, simulate following all paths sequentially
        if (isStarted()) {
            for (int i = 0; i < allPaths.length; i++) {
                telemetry.addData("Simulating path", (i + 1) + " of " + allPaths.length);
                telemetry.update();

                // Simulate path following with manual pose updates
                simulatePathFollowing(follower, allPaths[i], i);
            }

            telemetry.addLine("Simulation complete!");
            telemetry.addLine("Check AdvantageScope for visualization");
            telemetry.update();

            sleep(2000);
        }

        // Cleanup
        telemetryService.stopSession();
    }

    /**
     * Simulates following a path by manually updating the pose over time.
     * This allows visualization on test bench without actual motor movement.
     * Uses realistic heading interpolation that matches the actual path behavior.
     */
    private void simulatePathFollowing(Follower follower, PathChain path, int pathIndex) throws InterruptedException {
        // Get start and end poses
        Pose startPose = path.getPath(0).getPoint(0);
        Pose endPose = path.getPath(path.size() - 1).getPoint(1);

        // Calculate path length (approximate as straight line for simplicity)
        double deltaX = endPose.getX() - startPose.getX();
        double deltaY = endPose.getY() - startPose.getY();
        double pathLength = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // Simulation parameters
        double simulatedVelocity = 30.0; // inches per second
        double totalTime = pathLength / simulatedVelocity; // seconds
        double startTime = getRuntime();

        // Heading interpolation parameters (matches path's setLinearHeadingInterpolation)
        double headingInterpolationPoint = 0.7; // When heading changes most (matches DecodeAutonomousFarCommand)
        double startHeading = startPose.getHeading();
        double endHeading = endPose.getHeading();

        // Normalize heading difference to take shortest path
        double headingDelta = endHeading - startHeading;
        while (headingDelta > Math.PI) headingDelta -= 2 * Math.PI;
        while (headingDelta < -Math.PI) headingDelta += 2 * Math.PI;

        // Simulate path following
        while (opModeIsActive() && (getRuntime() - startTime) < totalTime) {
            double elapsedTime = getRuntime() - startTime;
            double progress = Math.min(elapsedTime / totalTime, 1.0);

            // Interpolate position along path (linear)
            double currentX = startPose.getX() + deltaX * progress;
            double currentY = startPose.getY() + deltaY * progress;

            // Interpolate heading using sigmoid-like curve centered at interpolationPoint
            // This matches Pedro's heading interpolation behavior more closely
            double headingProgress;
            if (progress < headingInterpolationPoint) {
                // Slower heading change before interpolation point
                headingProgress = 0.5 * (progress / headingInterpolationPoint);
            } else {
                // Faster heading change after interpolation point
                headingProgress = 0.5 + 0.5 * ((progress - headingInterpolationPoint) / (1.0 - headingInterpolationPoint));
            }

            // Apply smooth ease-in-out to heading interpolation for realistic motion
            headingProgress = smoothStep(headingProgress);
            double currentHeading = startHeading + headingDelta * headingProgress;

            Pose simulatedPose = new Pose(currentX, currentY, currentHeading);

            // Manually set the follower's pose for visualization
            follower.setPose(simulatedPose);

            // Update follower (needed for internal state even without movement)
            follower.update();

            // Draw on Panels
            PanelsBridge.drawFollowerDebug(follower);

            // Send telemetry packet to AdvantageScope
            boolean isBusy = progress < 0.99;
            sendPathSimPacket(simulatedPose, pathIndex, getPathName(pathIndex), isBusy);

            telemetry.addData("Path", getPathName(pathIndex));
            telemetry.addData("Progress", String.format("%.0f%%", progress * 100));
            telemetry.addData("X", String.format("%.1f", simulatedPose.getX()));
            telemetry.addData("Y", String.format("%.1f", simulatedPose.getY()));
            telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(simulatedPose.getHeading())));
            telemetry.addData("Elapsed", String.format("%.1fs", elapsedTime));
            telemetry.update();

            sleep(20); // 50 Hz update rate
        }

        // Ensure we end at the exact final pose
        follower.setPose(endPose);
        sendPathSimPacket(endPose, pathIndex, getPathName(pathIndex), false);
        sleep(500); // Pause between paths
    }

    /**
     * Smooth step function (3x^2 - 2x^3) for ease-in-out interpolation
     */
    private double smoothStep(double x) {
        x = Math.max(0.0, Math.min(1.0, x));
        return x * x * (3.0 - 2.0 * x);
    }

    private void sendPathSimPacket(Pose pedroPose, int pathIndex, String pathName, boolean isBusy) {
        if (dashboard == null) {
            return;
        }

        TelemetryPacket packet = new TelemetryPacket();

        // Convert Pedro pose to FTC frame for AdvantageScope field visualization
        Pose ftcPose = PoseTransforms.toFtcPose(pedroPose);

        // Add Pedro frame pose data
        packet.put("PathSim/Pose x", pedroPose.getX());  // Inches (Pedro frame)
        packet.put("PathSim/Pose y", pedroPose.getY());  // Inches (Pedro frame)
        packet.put("PathSim/Pose heading", pedroPose.getHeading());  // Radians (Pedro frame)

        // Add FTC frame pose data - AdvantageScope uses this for field visualization
        packet.put("PathSim/FTC Pose x", ftcPose.getX());  // Inches (FTC frame)
        packet.put("PathSim/FTC Pose y", ftcPose.getY());  // Inches (FTC frame)
        packet.put("PathSim/FTC Pose heading", ftcPose.getHeading());  // Radians (FTC frame)

        // Add path progress
        packet.put("PathSim/currentPathIndex", pathIndex);
        packet.put("PathSim/currentPathName", pathName);
        packet.put("PathSim/isBusy", isBusy);
        packet.put("PathSim/totalPaths", 7);

        // Add to field overlay with robot indicator and heading line (using Pedro coords for Dashboard)
        packet.fieldOverlay()
            .setStroke(isBusy ? "#00FF00" : "#FFFF00")  // Green when moving, yellow when stopped
            .setStrokeWidth(2)
            .strokeCircle(pedroPose.getX(), pedroPose.getY(), 9);  // Robot position

        // Draw heading indicator line showing robot orientation
        double headingLineLength = 15;
        double endX = pedroPose.getX() + headingLineLength * Math.cos(pedroPose.getHeading());
        double endY = pedroPose.getY() + headingLineLength * Math.sin(pedroPose.getHeading());
        packet.fieldOverlay()
            .setStroke(isBusy ? "#00FF00" : "#FFFF00")
            .setStrokeWidth(3)
            .strokeLine(pedroPose.getX(), pedroPose.getY(), endX, endY);  // Heading direction

        dashboard.sendTelemetryPacket(packet);
    }

    private String getPathName(int index) {
        String[] names = {
            "Start → Launch",
            "Launch → Alliance Wall",
            "Alliance Wall → Launch",
            "Launch → Parking",
            "Parking → Launch",
            "Launch → Gate Far",
            "Gate Far → Launch"
        };
        return index < names.length ? names[index] : "Unknown";
    }
}
