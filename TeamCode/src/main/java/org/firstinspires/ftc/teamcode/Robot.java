package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Light;

import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.ManualSpinController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryService;

import Ori.Coval.Logging.Logger.KoalaLog;

public class Robot {
    public final DriveSubsystem drive;
    public final LauncherSubsystem launcher;
    public final IntakeSubsystem intake;
    public final LightingSubsystem lighting;
    public final VisionSubsystemLimelight vision;
    public final LauncherCoordinator launcherCoordinator;
    public final TelemetryService telemetry;

    public final LauncherCommands launcherCommands;
    public final IntakeCommands intakeCommands;

    public ManualSpinController manualSpinController;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, new TelemetryService());
    }

    public Robot(HardwareMap hardwareMap, TelemetryService telemetryService) {
        telemetry = telemetryService == null ? new TelemetryService() : telemetryService;

        // Initialize KoalaLog for WPILOG file logging
        KoalaLog.setup(hardwareMap);

        // All subsystems use AutoLogged versions for automatic WPILOG and FTC Dashboard logging
        vision = new VisionSubsystemLimelight(hardwareMap);
        drive = new org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemAutoLogged(hardwareMap, vision);
        launcher = new org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemAutoLogged(hardwareMap);
        intake = new org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemAutoLogged(hardwareMap);
        lighting = new LightingSubsystem(hardwareMap);
        launcherCoordinator = new org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinatorAutoLogged(launcher, intake, lighting);
        launcherCommands = new LauncherCommands(launcher, launcherCoordinator);
        intakeCommands = new IntakeCommands(intake);

        manualSpinController = launcherCoordinator.createManualSpinController();

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
        configureInitialization(true, true);
    }

    private void configureInitialization(boolean enableTeleOpControl, boolean enableVisionRelocalization) {
        drive.setTeleOpControlEnabled(enableTeleOpControl);
        drive.setVisionRelocalizationEnabled(enableVisionRelocalization);
        drive.initialize();
        launcher.initialize();
        lighting.initialize();
        intake.initialize();
        vision.initialize();
        launcherCoordinator.initialize();
    }

    public void attachPedroFollower() {
        drive.attachFollower();
    }
}
