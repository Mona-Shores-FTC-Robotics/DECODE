package org.firstinspires.ftc.teamcode.telemetry.data;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotState;

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
    public final GamepadTelemetryData gamepad;
    public final LoopTimingTelemetryData timing;
    public final LightingTelemetryData lighting;

    public RobotTelemetryData(
            MatchContextData context,
            PoseTelemetryData pose,
            DriveTelemetryData drive,
            LauncherTelemetryData launcher,
            VisionTelemetryData vision,
            IntakeTelemetryData intake,
            GamepadTelemetryData gamepad,
            LoopTimingTelemetryData timing,
            LightingTelemetryData lighting
    ) {
        this.context = context;
        this.pose = pose;
        this.drive = drive;
        this.launcher = launcher;
        this.vision = vision;
        this.intake = intake;
        this.gamepad = gamepad;
        this.timing = timing;
        this.lighting = lighting;
    }

    /**
     * Capture a complete snapshot of robot telemetry data.
     *
     * @param drive Drive subsystem
     * @param launcher Launcher subsystem
     * @param intake Intake subsystem
     * @param vision Vision subsystem
     * @param lighting Lighting subsystem (may be null)
     * @param driveRequest Current drive request from bindings
     * @param gamepad1 Driver gamepad (may be null)
     * @param gamepad2 Operator gamepad (may be null)
     * @param alliance Current alliance
     * @param runtimeSec OpMode runtime in seconds
     * @param matchTimeSec Match time remaining in seconds
     * @param opMode OpMode name (e.g., "TeleOp", "Autonomous")
     * @param isAutonomous Whether this is autonomous mode
     * @param poseOverride Optional pose override (for autonomous routines)
     * @param prevMainLoopMs Main loop overhead from previous iteration (0 if not available)
     * @param telemetryStartNs Telemetry overhead from previous iteration (0 if not available)
     */
    public static RobotTelemetryData capture(
            DriveSubsystem drive,
            LauncherSubsystem launcher,
            IntakeSubsystem intake,
            VisionSubsystemLimelight vision,
            LightingSubsystem lighting,
            DriverBindings.DriveRequest driveRequest,
            Gamepad gamepad1,
            Gamepad gamepad2,
            Alliance alliance,
            double runtimeSec,
            double matchTimeSec,
            String opMode,
            boolean isAutonomous,
            Pose poseOverride,
            double prevMainLoopMs,
            long telemetryStartNs
    ) {
        // Capture match context
        MatchContextData context = new MatchContextData(
                alliance,
                runtimeSec,
                matchTimeSec,
                opMode,
                isAutonomous,
                org.firstinspires.ftc.teamcode.util.RobotConfigs.getActiveConfigName(),
                RobotState.getLauncherMode().name(),
                RobotState.getMotif().name(),
                RobotState.getMotifTail()
        );

        // Capture vision data (needed for pose)
        VisionTelemetryData visionData = VisionTelemetryData.capture(vision, alliance);

        // Build vision pose for PoseTelemetryData (FTC Pose type from vision subsystem)
        Pose visionPose = null;
        if (visionData.hasTag && !Double.isNaN(visionData.poseXIn) && !Double.isNaN(visionData.poseYIn)) {
            visionPose = vision.getRobotPoseFromTagFtc().orElse(null);
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
        DriveTelemetryData driveData = DriveTelemetryData.capture(drive, driveRequest, vision);
        LauncherTelemetryData launcherData = LauncherTelemetryData.capture(launcher);
        IntakeTelemetryData intakeData = IntakeTelemetryData.capture(intake);

        // Capture gamepad data
        GamepadTelemetryData gamepadData = GamepadTelemetryData.capture(gamepad1, gamepad2);

        // Capture loop timing data (includes previous loop's overhead + current subsystem times)
        LoopTimingTelemetryData timingData = LoopTimingTelemetryData.capture(
                prevMainLoopMs,
                telemetryStartNs,
                drive,
                intake,
                launcher,
                lighting,
                vision
        );

        // Capture lighting data
        LightingTelemetryData lightingData = LightingTelemetryData.capture(lighting);

        return new RobotTelemetryData(
                context,
                poseData,
                driveData,
                launcherData,
                visionData,
                intakeData,
                gamepadData,
                timingData,
                lightingData
        );
    }
}
