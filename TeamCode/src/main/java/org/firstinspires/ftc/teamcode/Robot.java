package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
import org.firstinspires.ftc.teamcode.util.RobotMode;
import org.firstinspires.ftc.teamcode.telemetry.RobotLogger;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryService;
import org.firstinspires.ftc.teamcode.telemetry.TelemetrySettings;

@Configurable
public class Robot {

    @Configurable
    public static class TestBenchConfig {
        /**
         * Enable test bench mode (no expansion hub required)
         * Set to true when testing on bench without full robot hardware
         */
        public static boolean testBenchMode = false;
    }
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

    public final ManualSpinController manualSpinController;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, new TelemetryService(TelemetrySettings.enablePsiKitLogging));
    }

    public Robot(HardwareMap hardwareMap, TelemetryService telemetryService) {
        telemetry = telemetryService == null ? new TelemetryService(TelemetrySettings.enablePsiKitLogging) : telemetryService;
        logger = new RobotLogger(telemetry);

        // Vision and Drive always initialize (needed for path visualization)
        vision = initializeVision(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, vision);

        // Expansion hub subsystems - optional in test bench mode
        if (TestBenchConfig.testBenchMode) {
            logger.logString("Robot", "TestBenchMode", "ENABLED - expansion hub hardware skipped");
            launcher = null; // Will be handled by null checks
            intake = null;
            lighting = null;
        } else {
            launcher = new LauncherSubsystem(hardwareMap);
            intake = new IntakeSubsystem(hardwareMap);
            lighting = new LightingSubsystem(hardwareMap);
        }

        // Create coordinators/commands with null checks
        launcherCoordinator = (launcher != null && intake != null && lighting != null)
                ? new LauncherCoordinator(launcher, intake, lighting)
                : null;
        launcherCommands = launcher != null
                ? new LauncherCommands(launcher, launcherCoordinator)
                : null;
        intakeCommands = intake != null
                ? new IntakeCommands(intake)
                : null;

        manualSpinController = createManualSpinController();
        applyRobotMode(robotMode);
        registerLoggingSources();
    }

    private VisionSubsystemLimelight initializeVision(HardwareMap hardwareMap) {
        return new VisionSubsystemLimelight(hardwareMap);
    }

    public void setAlliance(Alliance alliance) {
        RobotState.setAlliance(alliance);
        vision.setAlliance(alliance);
        if (lighting != null) {
            lighting.setAlliance(alliance);
        }
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
        if (launcher != null) launcher.initialize();
        if (lighting != null) lighting.initialize();
        if (intake != null) intake.initialize();
        vision.initialize();
        if (launcherCoordinator != null) launcherCoordinator.initialize();
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
        if (intake != null) intake.setRobotMode(mode);
        if (lighting != null) lighting.setRobotMode(mode);
        vision.setRobotMode(mode);
        if (launcherCoordinator != null) launcherCoordinator.setRobotMode(mode);
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

        if (launcher != null) {
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
        }

        if (intake != null) {
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
        }

        if (lighting != null) {
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
        }

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

    private ManualSpinController createManualSpinController() {
        if (launcherCoordinator == null) {
            return ManualSpinController.NO_OP;
        }
        return new ManualSpinController() {
            private int activeSources = 0;

            @Override
            public void enterManualSpin() {
                if (activeSources++ == 0) {
                    launcherCoordinator.setManualSpinOverride(true);
                }
            }

            @Override
            public void exitManualSpin() {
                if (activeSources <= 0) {
                    return;
                }
                if (--activeSources == 0) {
                    launcherCoordinator.setManualSpinOverride(false);
                }
            }
        };
    }
}
