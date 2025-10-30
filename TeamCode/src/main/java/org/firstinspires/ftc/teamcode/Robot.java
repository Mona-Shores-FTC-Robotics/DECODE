package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.teamcode.util.TelemetryService;
import org.firstinspires.ftc.teamcode.util.TelemetrySettings;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class Robot {
    public final DriveSubsystem drive;
    public final ShooterSubsystem shooter;
    public final IntakeSubsystem intake;
    public final LightingSubsystem lighting;
    public final VisionSubsystem vision;
    public final TelemetryService telemetry;
    public final RobotLogger logger;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, new TelemetryService(TelemetrySettings.enablePsiKitLogging));
    }

    public Robot(HardwareMap hardwareMap, TelemetryService telemetryService) {
        telemetry = telemetryService == null ? new TelemetryService(TelemetrySettings.enablePsiKitLogging) : telemetryService;
        logger = new RobotLogger(telemetry);
        vision = new VisionSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, vision);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        lighting = new LightingSubsystem(hardwareMap);
        intake.addLaneColorListener(lighting);
    }

    public void setAlliance(Alliance alliance) {
        RobotState.setAlliance(alliance);
        vision.setAlliance(alliance);
        lighting.setAlliance(alliance);
    }

    public void initialize(){
        drive.initialize();
        shooter.initialize();
        lighting.initialize();
        intake.initialize();
        vision.initialize();
    }
}
