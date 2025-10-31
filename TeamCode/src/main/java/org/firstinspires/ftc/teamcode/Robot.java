package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.teamcode.util.TelemetryService;
import org.firstinspires.ftc.teamcode.util.TelemetrySettings;

public class Robot {
    public final DriveSubsystem drive;
    public final ShooterSubsystem shooter;
    public final IntakeSubsystem intake;
    public final LightingSubsystem lighting;
    public final VisionSubsystem vision;
    public final TelemetryService telemetry;
    public final RobotLogger logger;

    private final DriveSubsystem.Inputs driveInputs = new DriveSubsystem.Inputs();
    private final ShooterSubsystem.Inputs shooterInputs = new ShooterSubsystem.Inputs();
    private final IntakeSubsystem.Inputs intakeInputs = new IntakeSubsystem.Inputs();
    private final LightingSubsystem.Inputs lightingInputs = new LightingSubsystem.Inputs();
    private final VisionSubsystem.Inputs visionInputs = new VisionSubsystem.Inputs();

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
        registerLoggingSources();
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

    private void registerLoggingSources() {
        logger.registerSource(new RobotLogger.Source() {
            @Override
            public String subsystem() {
                return "Drive";
            }

            @Override
            public void collect(RobotLogger.Frame frame) {
                drive.populateInputs(driveInputs);
                logger.logInputs("Drive", driveInputs);
            }
        });
        logger.registerSource(new RobotLogger.Source() {
            @Override
            public String subsystem() {
                return "Shooter";
            }

            @Override
            public void collect(RobotLogger.Frame frame) {
                shooter.populateInputs(shooterInputs);
                logger.logInputs("Shooter", shooterInputs);
            }
        });
        logger.registerSource(new RobotLogger.Source() {
            @Override
            public String subsystem() {
                return "Intake";
            }

            @Override
            public void collect(RobotLogger.Frame frame) {
                intake.populateInputs(intakeInputs);
                logger.logInputs("Intake", intakeInputs);
            }
        });
        logger.registerSource(new RobotLogger.Source() {
            @Override
            public String subsystem() {
                return "Lighting";
            }

            @Override
            public void collect(RobotLogger.Frame frame) {
                lighting.populateInputs(lightingInputs);
                logger.logInputs("Lighting", lightingInputs);
            }
        });
        logger.registerSource(new RobotLogger.Source() {
            @Override
            public String subsystem() {
                return "Vision";
            }

            @Override
            public void collect(RobotLogger.Frame frame) {
                vision.populateInputs(visionInputs);
                logger.logInputs("Vision", visionInputs);
            }
        });
    }

}
