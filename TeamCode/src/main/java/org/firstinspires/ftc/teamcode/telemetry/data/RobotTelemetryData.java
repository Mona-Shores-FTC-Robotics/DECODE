package org.firstinspires.ftc.teamcode.telemetry.data;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;

/**
 * Root container for all robot telemetry data.
 * Captures a complete snapshot of robot state for telemetry publishing.
 * <p>
 * This centralized data model makes it easy to:
 * - See all available telemetry fields in one place
 * - Extract data once and format it multiple ways (driver station, dashboard, panels)
 * - Add new telemetry fields without touching formatter code
 * - Maintain consistency across telemetry levels (MATCH/PRACTICE/DEBUG)
 * </p>
 */
public class RobotTelemetryData {
    public final MatchContextData context;
    public final PoseTelemetryData pose;
    public final DriveTelemetryData drive;
    public final LauncherTelemetryData launcher;
    public final VisionTelemetryData vision;
    public final IntakeTelemetryData intake;

    public RobotTelemetryData(
            MatchContextData context,
            PoseTelemetryData pose,
            DriveTelemetryData drive,
            LauncherTelemetryData launcher,
            VisionTelemetryData vision,
            IntakeTelemetryData intake
    ) {
        this.context = context;
        this.pose = pose;
        this.drive = drive;
        this.launcher = launcher;
        this.vision = vision;
        this.intake = intake;
    }

    /**
     * Capture a complete snapshot of robot telemetry data.
     *
     * @param drive Drive subsystem
     * @param launcher Launcher subsystem
     * @param intake Intake subsystem
     * @param vision Vision subsystem
     * @param coordinator Launcher coordinator (for artifact tracking)
     * @param driveRequest Current drive request from bindings
     * @param alliance Current alliance
     * @param runtimeSec OpMode runtime in seconds
     * @param matchTimeSec Match time remaining in seconds
     * @param opMode OpMode name (e.g., "TeleOp", "Autonomous")
     * @param isAutonomous Whether this is autonomous mode
     * @param poseOverride Optional pose override (for autonomous routines)
     * @return Complete robot telemetry snapshot
     */
    public static RobotTelemetryData capture(
            DriveSubsystem drive,
            LauncherSubsystem launcher,
            IntakeSubsystem intake,
            VisionSubsystemLimelight vision,
            LauncherCoordinator coordinator,
            DriverBindings.DriveRequest driveRequest,
            Alliance alliance,
            double runtimeSec,
            double matchTimeSec,
            String opMode,
            boolean isAutonomous,
            Pose poseOverride
    ) {
        // Capture match context
        MatchContextData context = new MatchContextData(
                alliance,
                runtimeSec,
                matchTimeSec,
                opMode,
                isAutonomous
        );

        // Capture vision data (needed for pose)
        VisionTelemetryData visionData = VisionTelemetryData.capture(vision, alliance);

        // Build vision pose for PoseTelemetryData
        Pose2D visionPose = null;
        if (visionData.hasTag && !Double.isNaN(visionData.poseXIn) && !Double.isNaN(visionData.poseYIn)) {
            visionPose = vision.getRobotPoseFromTag().orElse(null);
        }

        // Capture pose data (with optional override)
        PoseTelemetryData poseData;
        if (poseOverride != null) {
            Pose2D overridePose2D = new Pose2D(
                    org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH,
                    poseOverride.getX(),
                    poseOverride.getY(),
                    org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS,
                    poseOverride.getHeading()
            );
            poseData = new PoseTelemetryData(overridePose2D, poseOverride, visionPose);
        } else {
            poseData = PoseTelemetryData.capture(drive, visionPose);
        }

        // Capture subsystem data
        DriveTelemetryData driveData = DriveTelemetryData.capture(drive, driveRequest);
        LauncherTelemetryData launcherData = LauncherTelemetryData.capture(launcher);
        IntakeTelemetryData intakeData = IntakeTelemetryData.capture(intake, coordinator);

        return new RobotTelemetryData(
                context,
                poseData,
                driveData,
                launcherData,
                visionData,
                intakeData
        );
    }
}
