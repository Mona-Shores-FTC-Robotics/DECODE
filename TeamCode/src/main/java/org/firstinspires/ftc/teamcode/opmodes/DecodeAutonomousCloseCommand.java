package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldLayout;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldPoint;
import org.firstinspires.ftc.teamcode.util.RobotMode;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.ParallelCommandGroup;
import dev.nextftc.core.commands.SequentialCommandGroup;
import dev.nextftc.core.commands.WaitCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroCommand;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Command-based version of DecodeAutonomousClose using Sequential and Parallel Command Groups
 *
 * This autonomous routine:
 * 1. Starts at LAUNCH_CLOSE position
 * 2. Collects samples from Gate Close, Gate Far, and Parking Zone
 * 3. Scores each sample set at LAUNCH_CLOSE
 *
 * Uses NextFTC command framework to coordinate:
 * - Driving with intake/launcher positioning (parallel)
 * - Sequential scoring and collection routines
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Decode Auto Close (Command)", group = "Command")
public class DecodeAutonomousCloseCommand extends NextFTCOpMode {

    private static final Alliance DEFAULT_ALLIANCE = Alliance.BLUE;
    private static final RobotMode ACTIVE_MODE = RobotMode.MATCH;

    @Configurable
    public static class AutoMotionConfig {
        public static double maxPathPower = .6;
        public static double intakeTimeSeconds = 2.0;
        public static double launchDelaySeconds = 0.5;
    }

    private Robot robot;
    private AllianceSelector allianceSelector;
    private Alliance activeAlliance = Alliance.BLUE;
    private FieldLayout currentLayout;
    private IntakeCommands intakeCommands;
    private LauncherCommands launcherCommands;

    @Override
    public void onInit() {
        BindingManager.reset();
        robot = new Robot(hardwareMap);
        robot.setRobotMode(ACTIVE_MODE);
        robot.drive.setRobotCentric(DriveSubsystem.robotCentricConfig);
        robot.telemetry.startSession();
        robot.logger.startSession(hardwareMap.appContext, getClass().getSimpleName(), DEFAULT_ALLIANCE, "AutonomousInit");

        robot.launcherCoordinator.lockIntake();
        robot.launcherCoordinator.setIntakeAutomationEnabled(false);
        robot.initializeForAuto();

        // Initialize command factories
        intakeCommands = new IntakeCommands(robot.intake);
        launcherCommands = new LauncherCommands(robot.launcher, robot.launcherCoordinator);

        // Register init-phase controls
        GamepadEx driverPad = new GamepadEx(() -> gamepad1);
        allianceSelector = new AllianceSelector(driverPad, DEFAULT_ALLIANCE);
        driverPad.y().whenBecomesTrue(() -> applyAlliance(allianceSelector.getSelectedAlliance()));
        driverPad.leftBumper().whenBecomesTrue(() -> drawPreviewForAlliance(Alliance.BLUE));
        driverPad.rightBumper().whenBecomesTrue(() -> drawPreviewForAlliance(Alliance.RED));

        activeAlliance = allianceSelector.getSelectedAlliance();
        applyAlliance(activeAlliance);
        allianceSelector.applySelection(robot, robot.lighting);

        addComponents(
                BulkReadComponent.INSTANCE,
                CommandManager.INSTANCE,
                new SubsystemComponent(robot.drive),
                new SubsystemComponent(robot.launcher),
                new SubsystemComponent(robot.intake),
                new SubsystemComponent(robot.lighting),
                new SubsystemComponent(robot.vision),
                new SubsystemComponent(robot.launcherCoordinator)
        );
    }

    @Override
    public void onWaitForStart() {
        BindingManager.update();
        allianceSelector.updateFromVision(robot.vision);
        allianceSelector.applySelection(robot, robot.lighting);

        Alliance selectedAlliance = allianceSelector.getSelectedAlliance();
        if (selectedAlliance != activeAlliance) {
            activeAlliance = selectedAlliance;
            applyAlliance(activeAlliance);
        }

        robot.telemetry.updateDriverStation(telemetry);
    }

    @Override
    public void onStartButtonPressed() {
        BindingManager.reset();
        allianceSelector.lockSelection();
        robot.logger.updateAlliance(activeAlliance);
        robot.logger.logEvent("Autonomous", "Start");

        robot.launcherCoordinator.setIntakeAutomationEnabled(true);
        robot.launcherCoordinator.unlockIntake();
        robot.launcher.requestSpinUp();

        // Build and schedule the complete autonomous routine
        Command autoRoutine = buildAutonomousRoutine();
        CommandManager.scheduleCommand(autoRoutine);
    }

    @Override
    public void onUpdate() {
        robot.logger.logNumber("Autonomous", "RuntimeSec", getRuntime());
        robot.logger.sampleSources();
        robot.telemetry.updateDriverStation(telemetry);
    }

    @Override
    public void onStop() {
        allianceSelector.unlockSelection();
        BindingManager.reset();
        robot.launcher.abort();
        robot.drive.stop();
        robot.vision.stop();
        robot.logger.logEvent("Autonomous", "Stop");
        robot.logger.stopSession();
    }

    /**
     * Builds the complete autonomous routine using command groups
     */
    private Command buildAutonomousRoutine() {
        if (currentLayout == null) {
            return new WaitCommand(0.01);
        }

        Pose launchClosePose = currentLayout.pose(FieldPoint.LAUNCH_CLOSE);
        Pose gateClosePose = currentLayout.pose(FieldPoint.GATE_CLOSE_ARTIFACTS_PICKUP);
        Pose gateFar270DegPose = currentLayout.pose(FieldPoint.GATE_FAR_ARTIFACTS_PICKUP_270_DEG);
        Pose parking270DegPose = currentLayout.pose(FieldPoint.PARKING_ARTIFACTS_PICKUP_270_DEG);

        return new SequentialCommandGroup(
            // Start already at LAUNCH_CLOSE, so spin up launcher
            spinUpLauncher(),

            // Phase 1: Collect from Gate Close and score
            collectAndScore(launchClosePose, gateClosePose, launchClosePose),

            // Phase 2: Collect from Gate Far and score
            collectAndScore(launchClosePose, gateFar270DegPose, launchClosePose),

            // Phase 3: Collect from Parking Zone and score
            collectAndScore(launchClosePose, parking270DegPose, launchClosePose)
        );
    }

    /**
     * Creates a collect-and-score sequence
     * @param fromPose Starting pose
     * @param pickupPose Pickup location
     * @param scorePose Score location
     */
    private Command collectAndScore(Pose fromPose, Pose pickupPose, Pose scorePose) {
        return new SequentialCommandGroup(
            // Drive to pickup while preparing intake
            new ParallelCommandGroup(
                followPath(fromPose, pickupPose),
                new SequentialCommandGroup(
                    new WaitCommand(0.3),
                    prepareIntake()
                )
            ),

            // Collect samples
            intakeCommands.timedIntake(AutoMotionConfig.intakeTimeSeconds),

            // Drive to score while spinning up launcher
            new ParallelCommandGroup(
                followPath(pickupPose, scorePose),
                new SequentialCommandGroup(
                    new WaitCommand(0.3),
                    spinUpLauncher()
                )
            ),

            // Score the samples
            scoreSequence()
        );
    }

    /**
     * Creates a path following command
     */
    private Command followPath(Pose startPose, Pose endPose) {
        PathChain path;

        // Use curved path if poses are far apart, straight line otherwise
        double distance = Math.hypot(endPose.getX() - startPose.getX(), endPose.getY() - startPose.getY());
        if (distance > 30.0) {
            // Use curved path for longer distances
            path = robot.drive.getFollower().pathBuilder()
                    .addPath(new BezierCurve(startPose, endPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading(), 0.7)
                    .build();
        } else {
            // Straight line for shorter distances
            path = robot.drive.getFollower().pathBuilder()
                    .addPath(new BezierLine(startPose, endPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading(), 0.7)
                    .build();
        }

        double maxPower = Range.clip(AutoMotionConfig.maxPathPower, 0.0, 1.0);
        return new PedroCommand(robot.drive.getFollower(), path, maxPower, false);
    }

    /**
     * Prepares the intake for collecting samples
     */
    private Command prepareIntake() {
        return new Command() {
            @Override
            public void init() {
                // Set intake mode if needed
                robot.intake.setRunning(true);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    /**
     * Spins up the launcher
     */
    private Command spinUpLauncher() {
        return new Command() {
            @Override
            public void init() {
                robot.launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    /**
     * Scores samples using the launcher
     */
    private Command scoreSequence() {
        return new SequentialCommandGroup(
            new WaitCommand(AutoMotionConfig.launchDelaySeconds),
            launcherCommands.launchDetectedBurst()
        );
    }

    private void applyAlliance(Alliance alliance) {
        Alliance safeAlliance = alliance != null && alliance != Alliance.UNKNOWN ? alliance : DEFAULT_ALLIANCE;
        activeAlliance = safeAlliance;
        robot.setAlliance(activeAlliance);
        robot.logger.updateAlliance(activeAlliance);

        currentLayout = AutoField.layoutForAlliance(activeAlliance);
        Pose startPose = currentLayout.pose(FieldPoint.LAUNCH_CLOSE);
        robot.drive.getFollower().setStartingPose(startPose);
        robot.drive.getFollower().setPose(startPose);
    }

    private void drawPreviewForAlliance(Alliance alliance) {
        FieldLayout layout = AutoField.layoutForAlliance(alliance);
        // Preview paths would be drawn here using PanelsBridge
        PanelsBridge.drawPreview(new PathChain[0], layout.pose(FieldPoint.LAUNCH_CLOSE), alliance == Alliance.RED);
    }
}
