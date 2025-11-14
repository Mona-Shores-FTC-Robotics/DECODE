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

        // All subsystems use AutoLogged versions for automatic WPILOG and FTC Dashboard logging
        vision = new VisionSubsystemLimelight(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, vision);
        launcher = new LauncherSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        lighting = new LightingSubsystem(hardwareMap);
        launcherCoordinator = new LauncherCoordinator(launcher, intake, lighting);
        manualSpinController = launcherCoordinator.createManualSpinController();

        launcherCommands = new LauncherCommands(launcher, launcherCoordinator, manualSpinController);
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
