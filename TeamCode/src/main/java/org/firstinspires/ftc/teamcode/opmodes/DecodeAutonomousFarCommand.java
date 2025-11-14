package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeUntilFullCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldLayout;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldPoint;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
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
@Configurable
public class DecodeAutonomousFarCommand extends NextFTCOpMode {

    private static final Alliance DEFAULT_ALLIANCE = Alliance.BLUE;

    @Configurable
    public static class AutoMotionConfig {
        public double maxPathPower = .6;
        public double intakeTimeSeconds = 2.0;
        public double launchDelaySeconds = 0.5;
    }

    public static DecodeAutonomousFarCommand.AutoMotionConfig config = new DecodeAutonomousFarCommand.AutoMotionConfig();


    private Robot robot;
    private AllianceSelector allianceSelector;
    private Alliance activeAlliance = Alliance.BLUE;
    private FieldLayout currentLayout;
    private IntakeCommands intakeCommands;
    private LauncherCommands launcherCommands;

    // AprilTag-based start pose detection
    private Pose lastAppliedStartPosePedro;
    private Pose lastDetectedStartPosePedro;
    {
        addComponents(
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                BindingsComponent.INSTANCE,
                CommandManager.INSTANCE
        );
    }
    @Override
    public void onInit() {
        BindingManager.reset();
        robot = new Robot(hardwareMap);

        robot.attachPedroFollower();

        robot.drive.setRobotCentric(DriveSubsystem.robotCentricConfig);
        robot.telemetry.startSession();

        robot.launcherCoordinator.lockIntake();
        robot.launcherCoordinator.setIntakeAutomationEnabled(false);
        robot.initializeForAuto();

        // Initialize command factories
        intakeCommands = new IntakeCommands(robot.intake);
        launcherCommands = new LauncherCommands(robot.launcher, robot.launcherCoordinator, robot.manualSpinController);

        // Register init-phase controls
        GamepadEx driverPad = new GamepadEx(() -> gamepad1);
        allianceSelector = new AllianceSelector(driverPad, DEFAULT_ALLIANCE);
        driverPad.y().whenBecomesTrue(() -> applyAlliance(allianceSelector.getSelectedAlliance(), lastAppliedStartPosePedro));
        driverPad.leftBumper().whenBecomesTrue(() -> drawPreviewForAlliance(Alliance.BLUE));
        driverPad.rightBumper().whenBecomesTrue(() -> drawPreviewForAlliance(Alliance.RED));
        driverPad.a().whenBecomesTrue(this::applyLastDetectedStartPose);

        activeAlliance = allianceSelector.getSelectedAlliance();
        applyAlliance(activeAlliance, null);
        allianceSelector.applySelection(robot, robot.lighting);

        addComponents(
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

        // Detect alliance and start pose from AprilTag vision
        java.util.Optional<VisionSubsystemLimelight.TagSnapshot> snapshotOpt =
                allianceSelector.updateFromVision(robot.vision);
        allianceSelector.applySelection(robot, robot.lighting);

        // Extract detected start pose from vision
        if (snapshotOpt.isPresent()) {
            VisionSubsystemLimelight.TagSnapshot snapshot = snapshotOpt.get();
            java.util.Optional<Pose> detectedPose = snapshot.getRobotPose();
            if (detectedPose.isPresent()) {
                lastDetectedStartPosePedro = copyPose(detectedPose.get());
            }
        }

        Alliance selectedAlliance = allianceSelector.getSelectedAlliance();
        if (selectedAlliance != activeAlliance) {
            activeAlliance = selectedAlliance;
            applyAlliance(activeAlliance, null);
        }

        robot.telemetry.updateDriverStation(telemetry);
    }

    @Override
    public void onStartButtonPressed() {
        BindingManager.reset();
        allianceSelector.lockSelection();

        robot.launcherCoordinator.setIntakeAutomationEnabled(true);
        robot.launcherCoordinator.unlockIntake();
        robot.launcher.requestSpinUp();

        // Build and schedule the complete autonomous routine
        Command autoRoutine = buildAutonomousRoutine();
        CommandManager.INSTANCE.scheduleCommand(autoRoutine);
    }

    @Override
    public void onUpdate() {
        robot.telemetry.updateDriverStation(telemetry);
    }

    @Override
    public void onStop() {
        allianceSelector.unlockSelection();
        BindingManager.reset();
        robot.launcher.abort();
        robot.drive.stop();
        robot.vision.stop();
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
                    spinUpLauncher(),
                    followPath(startFarPose, launchFarPose)
                ),
                scoreSequence(),

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
        PathChain path = buildPath(robot.drive.getFollower(), startPose, endPose, controlPoints);
        double maxPower = Range.clip(config.maxPathPower, 0.0, 1.0);
        return new FollowPath(path, false, maxPower);
    }

    /**
     * Builds a PathChain from start to end with optional control points.
     * This is public static so test OpModes can visualize the same paths.
     */
    public static PathChain buildPath(Follower follower, Pose startPose, Pose endPose, Pose... controlPoints) {
        if (controlPoints.length > 0) {
            // Curved path with control points
            return follower.pathBuilder()
                    .addPath(new BezierCurve(startPose, controlPoints[0], endPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading(), 0.7)
                    .build();
        } else {
            // Straight line
            return follower.pathBuilder()
                    .addPath(new BezierLine(startPose, endPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading(), 0.7)
                    .build();
        }
    }

    /**
     * Builds all paths for the autonomous routine for visualization.
     * Returns an array of PathChains in order: [startToLaunch, launchToWall, wallToLaunch,
     * launchToParking, parkingToLaunch, launchToGate, gateToLaunch]
     */
    public static PathChain[] buildAllPathsForVisualization(Follower follower, Alliance alliance) {
        FieldLayout layout = AutoField.layoutForAlliance(alliance);

        Pose startFarPose = layout.pose(FieldPoint.START_FAR);
        Pose launchFarPose = layout.pose(FieldPoint.LAUNCH_FAR);
        Pose allianceWallPose = layout.pose(FieldPoint.ALLIANCE_WALL_ARTIFACTS_PICKUP);
        Pose parking90DegPose = layout.pose(FieldPoint.PARKING_ARTIFACTS_PICKUP_90_DEG);
        Pose gateFar90DegPose = layout.pose(FieldPoint.GATE_FAR_ARTIFACTS_PICKUP_90_DEG);
        Pose parkingControlPoint = AutoField.parkingArtifactsControlPoint(alliance);
        Pose gateFarControlPoint = AutoField.gateFarArtifactsControlPoint(alliance);

        return new PathChain[] {
            buildPath(follower, startFarPose, launchFarPose),
            buildPath(follower, launchFarPose, allianceWallPose),
            buildPath(follower, allianceWallPose, launchFarPose),
            buildPath(follower, launchFarPose, parking90DegPose, parkingControlPoint),
            buildPath(follower, parking90DegPose, launchFarPose),
            buildPath(follower, launchFarPose, gateFar90DegPose, gateFarControlPoint),
            buildPath(follower, gateFar90DegPose, launchFarPose)
        };
    }


    private void applyAlliance(Alliance alliance, Pose startOverride) {
        activeAlliance = alliance != null && alliance != Alliance.UNKNOWN ? alliance : DEFAULT_ALLIANCE;
        robot.setAlliance(activeAlliance);

        currentLayout = AutoField.layoutForAlliance(activeAlliance);

        // Apply AprilTag-detected start pose override if provided
        if (startOverride != null) {
            currentLayout.overrideStart(startOverride);
            lastAppliedStartPosePedro = copyPose(startOverride);
        }

        Pose startPose = currentLayout.pose(FieldPoint.START_FAR);
        robot.drive.getFollower().setStartingPose(startPose);
        robot.drive.getFollower().setPose(startPose);
    }

    private void drawPreviewForAlliance(Alliance alliance) {
        FieldLayout layout = AutoField.layoutForAlliance(alliance);
        // Preview paths would be drawn here using PanelsBridge
        PanelsBridge.drawPreview(new PathChain[0], layout.pose(FieldPoint.START_FAR), alliance == Alliance.RED);
    }

    /**
     * Spins up the launcher and waits until all launchers reach target RPM.
     * Phase 1: Uses position-specific tunable RPM from LaunchAtPositionCommand.PositionRpmConfig
     */
    private Command spinUpLauncher() {
        return launcherCommands.spinUpForPosition(FieldPoint.LAUNCH_FAR);
    }

    /**
     * Scores samples using the launcher
     */
    private Command scoreSequence() {
        return new SequentialGroup(
            new Delay(config.launchDelaySeconds),
            launcherCommands.launchDetectedBurst()
        );
    }

    /**
     * Applies the last AprilTag-detected start pose (bound to dpad A during init)
     */
    private void applyLastDetectedStartPose() {
        if (!shouldUpdateStartPose(lastDetectedStartPosePedro)) {
            return;
        }
        applyAlliance(activeAlliance, lastDetectedStartPosePedro);
    }

    /**
     * Validates that a detected pose is reasonable before applying it
     */
    private boolean shouldUpdateStartPose(Pose candidate) {
        if (candidate == null) {
            return false;
        }

        // Sanity check: pose should be on the field
        double fieldWidthIn = AutoField.waypoints.fieldWidthIn;
        if (candidate.getX() < 0 || candidate.getX() > fieldWidthIn ||
            candidate.getY() < 0 || candidate.getY() > fieldWidthIn) {
            return false;
        }

        // Additional check: if we have an applied pose, the detected pose shouldn't be wildly different
        if (lastAppliedStartPosePedro != null) {
            double deltaX = Math.abs(candidate.getX() - lastAppliedStartPosePedro.getX());
            double deltaY = Math.abs(candidate.getY() - lastAppliedStartPosePedro.getY());
            double maxDelta = 12.0; // 12 inches tolerance

            if (deltaX > maxDelta || deltaY > maxDelta) {
                return false; // Detected pose too different from expected
            }
        }

        return true;
    }

    /**
     * Creates a defensive copy of a Pose
     */
    private Pose copyPose(Pose pose) {
        if (pose == null) {
            return null;
        }
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

}
