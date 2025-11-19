package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Light;

import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryService;

@Configurable
public class Robot {

    @Configurable
    public static class VisionConfig {
        /**
         * Enable automatic vision relocalization during TeleOp.
         *
         * IMPORTANT: Currently using MegaTag1 (MT1) which is less reliable than MT2.
         * MT1 provides single-tag poses with 4-8" accuracy and can be noisy/jumpy.
         *
         * Recommendation: Keep DISABLED until MegaTag2 (MT2) is fixed.
         * - MT1 relocalization can introduce more error than odometry drift
         * - Manual relocalization (A button) is still available when needed
         * - Once MT2 is working (2-3" accuracy, multi-tag fusion), re-enable this
         */
        public static boolean enableAutoRelocalization = false;
    }
    public final DriveSubsystem drive;
    public final LauncherSubsystem launcher;
    public final IntakeSubsystem intake;
    public final LightingSubsystem lighting;
    public final VisionSubsystemLimelight vision;
    public final TelemetryService telemetry;

    public final LauncherCommands launcherCommands;
    public final IntakeCommands intakeCommands;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, new TelemetryService());
    }

    public Robot(HardwareMap hardwareMap, TelemetryService telemetryService) {
        telemetry = telemetryService == null ? new TelemetryService() : telemetryService;
        vision = new VisionSubsystemLimelight(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, vision);
        launcher = new LauncherSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        lighting = new LightingSubsystem(hardwareMap);

        launcherCommands = new LauncherCommands(launcher, intake);
        intakeCommands = new IntakeCommands(intake);
    }

    public void setAlliance(Alliance alliance) {
        RobotState.setAlliance(alliance);
        vision.setAlliance(alliance);
        lighting.setAlliance(alliance);
    }

    public void initialize() {
        initializeForTeleOp();
    }

    public void initializeForAuto() {
        configureInitialization(false, false);
    }

    public void initializeForTeleOp() {
        // Use configurable flag for vision relocalization (disabled by default with MT1)
        configureInitialization(true, VisionConfig.enableAutoRelocalization);
    }

    private void configureInitialization(boolean enableTeleOpControl, boolean enableVisionRelocalization) {
        drive.setTeleOpControlEnabled(enableTeleOpControl);
        drive.setVisionRelocalizationEnabled(enableVisionRelocalization);
        vision.initialize();
        drive.initialize();
        launcher.initialize();
        lighting.initialize();
        intake.initialize();


        // Wire lighting to receive lane color updates from intake
        intake.addLaneColorListener(lighting);
    }

    public void attachPedroFollower() {
        drive.attachFollower();
    }
}
