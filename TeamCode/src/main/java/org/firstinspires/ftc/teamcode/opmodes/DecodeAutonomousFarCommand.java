package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeUntilFullCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.FireAllCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.SpinHoldCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldLayout;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldPoint;
import org.firstinspires.ftc.teamcode.util.RobotMode;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Command-based version of DecodeAutonomousFar using Sequential and Parallel Command Groups
 *
 * This autonomous routine:
 * 1. Drives from START_FAR to LAUNCH_FAR and scores preload
 * 2. Collects samples from Alliance Wall, Parking Zone, and Gate Far
 * 3. Scores each sample set at LAUNCH_FAR
 *
 * Uses NextFTC command framework to coordinate:
 * - Driving with intake/launcher positioning (parallel)
 * - Sequential scoring and collection routines
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Decode Auto Far (Command)", group = "Command")
public class DecodeAutonomousFarCommand extends NextFTCOpMode {

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
        CommandManager.INSTANCE.scheduleCommand(autoRoutine);
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
            return new Delay(0.01);
        }

        Pose startFarPose = currentLayout.pose(FieldPoint.START_FAR);
        Pose launchFarPose = currentLayout.pose(FieldPoint.LAUNCH_FAR);
        Pose allianceWallPose = currentLayout.pose(FieldPoint.ALLIANCE_WALL_ARTIFACTS_PICKUP);
        Pose parking90DegPose = currentLayout.pose(FieldPoint.PARKING_ARTIFACTS_PICKUP_90_DEG);
        Pose gateFar90DegPose = currentLayout.pose(FieldPoint.GATE_FAR_ARTIFACTS_PICKUP_90_DEG);
        Pose parkingControlPoint = AutoField.parkingArtifactsControlPoint(activeAlliance);
        Pose gateFarControlPoint = AutoField.gateFarArtifactsControlPoint(activeAlliance);

        return new SequentialGroup(
                // Phase 1: Drive to launch position and score preload
                new ParallelGroup(
                    new SpinHoldCommand(robot.launcher, robot.manualSpinController),
                    followPath(startFarPose, launchFarPose)
                ),
                new FireAllCommand(robot.launcher, false, robot.manualSpinController),

                // Phase 2: Collect from Alliance Wall and score
                collectAndScore(launchFarPose, allianceWallPose, launchFarPose),

                // Phase 3: Collect from Parking Zone and score
                collectAndScore(launchFarPose, parking90DegPose, launchFarPose, parkingControlPoint),

                // Phase 4: Collect from Gate Far and score
                collectAndScore(launchFarPose, gateFar90DegPose, launchFarPose, gateFarControlPoint)
        );
    }

    /**
     * Creates a collect-and-score sequence
     * @param fromPose Starting pose
     * @param pickupPose Pickup location
     * @param scorePose Score location
     * @param controlPoints Optional control points for curved paths
     */
    private Command collectAndScore(Pose fromPose, Pose pickupPose, Pose scorePose, Pose... controlPoints) {
        return new SequentialGroup(
            // Drive to pickup while preparing intake
            new ParallelGroup(
                followPath(fromPose, pickupPose, controlPoints),
                new SequentialGroup(
                    new Delay(0.3),
                    new IntakeUntilFullCommand(robot.intake, 3)
                )
            ),

            // Drive to score while spinning up launcher
            new ParallelGroup(
                    followPath(pickupPose, scorePose),
                    spinUpLauncher()
            ),

            // Score the samples
            scoreSequence()
        );
    }

    /**
     * Creates a path following command
     */
    private Command followPath(Pose startPose, Pose endPose, Pose... controlPoints) {
        PathChain path;
        if (controlPoints.length > 0) {
            // Curved path with control points
            path = robot.drive.getFollower().pathBuilder()
                    .addPath(new BezierCurve(startPose, controlPoints[0], endPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading(), 0.7)
                    .build();
        } else {
            // Straight line
            path = robot.drive.getFollower().pathBuilder()
                    .addPath(new BezierLine(startPose, endPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading(), 0.7)
                    .build();
        }

        double maxPower = Range.clip(AutoMotionConfig.maxPathPower, 0.0, 1.0);
        return new FollowPath(path, false, maxPower);
    }


    private void applyAlliance(Alliance alliance) {
        Alliance safeAlliance = alliance != null && alliance != Alliance.UNKNOWN ? alliance : DEFAULT_ALLIANCE;
        activeAlliance = safeAlliance;
        robot.setAlliance(activeAlliance);
        robot.logger.updateAlliance(activeAlliance);

        currentLayout = AutoField.layoutForAlliance(activeAlliance);
        Pose startPose = currentLayout.pose(FieldPoint.START_FAR);
        robot.drive.getFollower().setStartingPose(startPose);
        robot.drive.getFollower().setPose(startPose);
    }

    private void drawPreviewForAlliance(Alliance alliance) {
        FieldLayout layout = AutoField.layoutForAlliance(alliance);
        // Preview paths would be drawn here using PanelsBridge
        PanelsBridge.drawPreview(new PathChain[0], layout.pose(FieldPoint.START_FAR), alliance == Alliance.RED);
    }


}
