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
import org.firstinspires.ftc.teamcode.util.ControlHubIdentifierUtil;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryService;

public class Robot {
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
        // Note: setRobotName() is now called from OpMode.onInit() BEFORE attachPedroFollower()
        // to give WiFi more time to initialize on first boot
        telemetry = telemetryService == null ? new TelemetryService() : telemetryService;
        vision = new VisionSubsystemLimelight(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, vision);
        launcher = new LauncherSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        lighting = new LightingSubsystem(hardwareMap);

        drive.setLightingSubsystem(lighting);

        launcherCommands = new LauncherCommands(launcher, intake);
        intakeCommands = new IntakeCommands(intake);


    }

    public void setAlliance(Alliance alliance) {
        RobotState.setAlliance(alliance);
        vision.setAlliance(alliance);
        lighting.setAlliance(alliance);
    }

    public void initializeForAuto() {
        drive.setTeleOpControlEnabled(false);
        vision.initialize();
        drive.initialize();
        launcher.initialize();
        lighting.initialize();
        intake.initialize();

        // Wire lighting to receive lane color updates from intake
//        intake.addLaneColorListener(lighting);
    }

    public void initializeForTeleOp() {
        drive.setTeleOpControlEnabled(true);
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
