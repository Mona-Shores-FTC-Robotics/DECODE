package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoRoutineBuilder;
import org.firstinspires.ftc.teamcode.util.PedroPathLoader;

import java.util.List;

/**
 * PATH VISUALIZATION TOOL - NO HARDWARE REQUIRED
 *
 * This OpMode visualizes .pp file paths on FTControl Panels without needing to move the robot.
 * Perfect for:
 * - Testing new paths without hardware
 * - Previewing path changes
 * - Debugging path issues
 * - Planning autonomous strategies
 *
 * How to use:
 * 1. Connect to FTControl Panels (http://192.168.49.1:5800 or your robot IP)
 * 2. Configure ppFileName in FTC Dashboard
 * 3. Init this OpMode
 * 4. Paths will be drawn on the field visualization
 * 5. Change alliance to see red/blue mirroring
 * 6. Never press START - just visualize!
 */
@Autonomous(name = "Visualize .pp Paths (NO HARDWARE)", group = "Visualization")
@Configurable
public class VisualizePathsFromPpFile extends LinearOpMode {

    @Configurable
    public static class VisualizationConfig {
        /** .pp file to visualize (in TeamCode/src/main/assets/) */
        public String ppFileName = "trajectory.pp";

        /** Alliance for path coloring and mirroring */
        public Alliance alliance = Alliance.BLUE;

        /** Show detailed segment info on driver station */
        public boolean showSegmentDetails = true;
    }

    public static VisualizationConfig config = new VisualizationConfig();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.attachPedroFollower();
        robot.telemetry.startSession();

        telemetry.addLine("=== PATH VISUALIZATION TOOL ===");
        telemetry.addLine();
        telemetry.addData("File", config.ppFileName);
        telemetry.addData("Alliance", config.alliance.displayName());
        telemetry.addLine();

        try {
            // Load the .pp file
            AutoRoutineBuilder builder = AutoRoutineBuilder.fromPpFile(
                config.ppFileName,
                robot,
                hardwareMap,
                config.alliance
            );

            // Visualize on PanelsBridge
            builder.visualizePaths(config.alliance);

            // Show segment details on driver station
            if (config.showSegmentDetails) {
                telemetry.addLine("--- Segments ---");
                List<PedroPathLoader.PathSegment> segments = builder.getSegments();
                for (int i = 0; i < segments.size(); i++) {
                    PedroPathLoader.PathSegment seg = segments.get(i);
                    telemetry.addLine(String.format("%d. %s", i + 1, seg.name));
                    telemetry.addLine(String.format("   Start: (%.1f, %.1f) @ %.0f°",
                        seg.startPose.getX(),
                        seg.startPose.getY(),
                        Math.toDegrees(seg.startPose.getHeading())));
                    telemetry.addLine(String.format("   End:   (%.1f, %.1f) @ %.0f°",
                        seg.endPose.getX(),
                        seg.endPose.getY(),
                        Math.toDegrees(seg.endPose.getHeading())));
                    if (!seg.controlPoints.isEmpty()) {
                        telemetry.addLine(String.format("   Curve: %d control points",
                            seg.controlPoints.size()));
                    }
                    if (seg.isReverse) {
                        telemetry.addLine("   REVERSE");
                    }
                }
                telemetry.addLine();
            }

            telemetry.addLine("✓ Paths drawn on FTControl Panels");
            telemetry.addLine();
            telemetry.addLine("Connect to Panels to see visualization:");
            telemetry.addLine("http://192.168.49.1:5800");
            telemetry.addLine();
            telemetry.addLine("Change ppFileName/alliance in FTC Dashboard Config");
            telemetry.addLine("Press STOP and re-INIT to refresh");
            telemetry.addLine();
            telemetry.addLine("DO NOT PRESS START - visualization only!");

        } catch (Exception e) {
            telemetry.addLine("❌ ERROR loading .pp file:");
            telemetry.addLine(e.getMessage());
            telemetry.addLine();
            telemetry.addLine("Check:");
            telemetry.addLine("1. File exists in TeamCode/src/main/assets/");
            telemetry.addLine("2. Filename is correct (case-sensitive)");
            telemetry.addLine("3. File has valid .pp JSON format");
        }

        telemetry.update();

        // Wait for stop - don't allow START
        while (!isStopRequested()) {
            idle();
        }
    }
}
