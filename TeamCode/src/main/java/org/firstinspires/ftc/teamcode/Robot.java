package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotMode;
import org.firstinspires.ftc.teamcode.telemetry.RobotLogger;
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
    public final RobotLogger logger;

    public final LauncherCommands launcherCommands;
    public final IntakeCommands intakeCommands;

    private RobotMode robotMode = RobotMode.DEBUG;

    private final DriveSubsystem.Inputs driveInputs = new DriveSubsystem.Inputs();
    private final LauncherSubsystem.Inputs shooterInputs = new LauncherSubsystem.Inputs();
    private final IntakeSubsystem.Inputs intakeInputs = new IntakeSubsystem.Inputs();
    private final LightingSubsystem.Inputs lightingInputs = new LightingSubsystem.Inputs();
    private final VisionSubsystemLimelight.Inputs visionInputs = new VisionSubsystemLimelight.Inputs();

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, new TelemetryService());
    }

    public Robot(HardwareMap hardwareMap, TelemetryService telemetryService) {
        telemetry = telemetryService == null ? new TelemetryService() : telemetryService;
        logger = new RobotLogger(telemetry);

        // Initialize KoalaLog for WPILOG file logging
        KoalaLog.setup(hardwareMap);

        vision = new VisionSubsystemLimelight(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, vision);
        launcher = new LauncherSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        lighting = new LightingSubsystem(hardwareMap);
        launcherCoordinator = new LauncherCoordinator(launcher, intake, lighting);
        launcherCommands = new LauncherCommands(launcher, launcherCoordinator);
        intakeCommands = new IntakeCommands(intake);
        applyRobotMode(robotMode);
        registerLoggingSources();
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

    public void setRobotMode(RobotMode mode) {
        robotMode = RobotMode.orDefault(mode);
        applyRobotMode(robotMode);
    }

    public RobotMode getRobotMode() {
        return robotMode;
    }

    private void applyRobotMode(RobotMode mode) {
        drive.setRobotMode(mode);
        intake.setRobotMode(mode);
        lighting.setRobotMode(mode);
        vision.setRobotMode(mode);
        launcherCoordinator.setRobotMode(mode);
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
                drive.logPoseFusion(logger);
            }
        });
        logger.registerSource(new RobotLogger.Source() {
            @Override
            public String subsystem() {
                return "Shooter";
            }

            @Override
            public void collect(RobotLogger.Frame frame) {
                launcher.populateInputs(shooterInputs);
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
