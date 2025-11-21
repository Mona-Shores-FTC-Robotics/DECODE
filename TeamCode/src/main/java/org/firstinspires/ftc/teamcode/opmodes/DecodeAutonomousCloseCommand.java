package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.DecodeAutonomousFarCommand.buildPath;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldLayout;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldPoint;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.LauncherMode;
import org.firstinspires.ftc.teamcode.util.LauncherRange;
import org.firstinspires.ftc.teamcode.util.RobotState;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
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
@Configurable
public class DecodeAutonomousCloseCommand extends NextFTCOpMode {

    private static final Alliance DEFAULT_ALLIANCE = Alliance.BLUE;

    @Configurable
    public static class AutoMotionConfig {
        public double maxPathPower = .6;
        public double intakeDelaySeconds = .1;
        /**
         * Starting launcher mode for autonomous.
         * DECODE: Fire in obelisk pattern sequence (recommended for endgame scoring)
         * THROUGHPUT: Rapid fire all lanes (recommended for early match throughput)
         */
        public LauncherMode startingLauncherMode = LauncherMode.DECODE;
    }

    // Public static instance for FTC Dashboard Config tab
    public static DecodeAutonomousCloseCommand.AutoMotionConfig config = new DecodeAutonomousCloseCommand.AutoMotionConfig();

    private Robot robot;
    private AllianceSelector allianceSelector;
    private Alliance activeAlliance = Alliance.BLUE;
    private FieldLayout currentLayout;
    private IntakeCommands intakeCommands;
    private LauncherCommands launcherCommands;
    private LightingSubsystem.InitController lightingInitController;

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

        robot.initializeForAuto();

        // Initialize command factories
        intakeCommands = new IntakeCommands(robot.intake);
        launcherCommands = new LauncherCommands(robot.launcher, robot.intake);

        // Register init-phase controls
        // Use Alliance.UNKNOWN as default to enable automatic vision detection
        // Manual overrides (D-pad left/right) still work as expected
        GamepadEx driverPad = new GamepadEx(() -> gamepad1);
        allianceSelector = new AllianceSelector(driverPad, Alliance.UNKNOWN);
        driverPad.y().whenBecomesTrue(() -> applyAlliance(allianceSelector.getSelectedAlliance(), lastAppliedStartPosePedro));
        driverPad.leftBumper().whenBecomesTrue(() -> drawPreviewForAlliance(Alliance.BLUE));
        driverPad.rightBumper().whenBecomesTrue(() -> drawPreviewForAlliance(Alliance.RED));
        driverPad.a().whenBecomesTrue(this::applyLastDetectedStartPose);
        lightingInitController = new LightingSubsystem.InitController(robot, allianceSelector, robot.lighting);
        lightingInitController.initialize();

        activeAlliance = allianceSelector.getSelectedAlliance();
        applyAlliance(activeAlliance, null);
        allianceSelector.applySelection(robot, robot.lighting);

        addComponents(
                new SubsystemComponent(robot.drive),
                new SubsystemComponent(robot.launcher),
                new SubsystemComponent(robot.intake),
                new SubsystemComponent(robot.lighting),
                new SubsystemComponent(robot.vision)
        );
    }

    @Override
    public void onWaitForStart() {
        BindingManager.update();
        if (lightingInitController != null) {
            lightingInitController.updateDuringInit(gamepad1.dpad_up);
        }

        // Update subsystems to poll sensors (especially intake color sensors)
        robot.intake.periodic();
        robot.vision.periodic();

        // Detect alliance and start pose from AprilTag vision
        java.util.Optional<VisionSubsystemLimelight.TagSnapshot> snapshotOpt =
                allianceSelector.updateFromVision(robot.vision);
        allianceSelector.applySelection(robot, robot.lighting);

        // Extract detected start pose from vision
        if (snapshotOpt.isPresent()) {
            VisionSubsystemLimelight.TagSnapshot snapshot = snapshotOpt.get();
            java.util.Optional<Pose> detectedPose = snapshot.getRobotPosePedroMT1();
            detectedPose.ifPresent(pose -> lastDetectedStartPosePedro = copyPose(pose));
        }

        Alliance selectedAlliance = allianceSelector.getSelectedAlliance();
        if (selectedAlliance != activeAlliance) {
            activeAlliance = selectedAlliance;
            applyAlliance(activeAlliance, null);
        }

        telemetry.clear();
        telemetry.addData("Alliance", activeAlliance.displayName());
        telemetry.addData("Artifacts", "%d detected", robot.intake.getArtifactCount());
        telemetry.addLine("D-pad Left/Right override, Down uses vision, Up returns to default");
        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        BindingManager.reset();
        allianceSelector.lockSelection();
        if (lightingInitController != null) {
            lightingInitController.onStart();
        }

        // Initialize launcher mode from config (defaults to DECODE, can be changed via Dashboard)
        RobotState.setLauncherMode(config.startingLauncherMode);
        RobotState.resetMotifTail(); // Start with fresh motif tail (0)

        robot.launcher.setSpinMode(LauncherSubsystem.SpinMode.FULL);

        // Build and schedule the complete autonomous routine
        Command autoRoutine = buildAutonomousRoutine();
        CommandManager.INSTANCE.scheduleCommand(autoRoutine);

        robot.intake.forwardRoller();
        robot.intake.setGateAllowArtifacts();
    }

    @Override
    public void onUpdate() {
        if (lightingInitController != null) {
            lightingInitController.updateDuringMatch();
        }
    }

    @Override
    public void onStop() {
        allianceSelector.unlockSelection();
        BindingManager.reset();
        robot.launcher.abort();
        robot.drive.stop();
        robot.vision.stop();

        // Save final pose for TeleOp transition
        RobotState.setHandoffPose(robot.drive.getFollower().getPose());
    }

    /**
     * Builds the complete autonomous routine using command groups
     */
    private Command buildAutonomousRoutine() {
        if (currentLayout == null) {
            return new Delay(0.01);
        }

        Pose startClosePose = currentLayout.pose(FieldPoint.START_CLOSE);
        Pose launchClosePose = currentLayout.pose(FieldPoint.LAUNCH_CLOSE);
        Pose preArtifacts1Pose = currentLayout.pose(FieldPoint.PRE_GATE_ARTIFACTS_PICKUP_270_DEG);
        Pose artifactsSet1Pose = currentLayout.pose(FieldPoint.ARTIFACTS_SET_1_270);

        Pose artifactsSet2Pose = currentLayout.pose(FieldPoint.ARTIFACTS_SET_2_270);
        Pose artifactsSet2ControlPoint = AutoField.artifactsSet2ControlPoint(activeAlliance);

        Pose artifactsSet3Pose = currentLayout.pose(FieldPoint.ARTIFACTS_SET_3_270);
        Pose artifactsSet3Control = AutoField.parkingArtifactsControlPoint(activeAlliance);
        Pose moveToGatePose = currentLayout.pose(FieldPoint.MOVE_TO_GATE);

        return new SequentialGroup(
            // Start already at LAUNCH_CLOSE, so spin up launcher
                new ParallelGroup(
                        spinUpLauncher(),
                        followPath(startClosePose, launchClosePose)
                ),
                scoreSequence(),

                followPath(launchClosePose, preArtifacts1Pose),

                // Phase 1: Collect from Gate Close and score
            collectAndScore(preArtifacts1Pose, artifactsSet1Pose, launchClosePose),

            // Phase 2: Collect from Gate Far and score
            collectAndScore(launchClosePose, artifactsSet2Pose, launchClosePose, artifactsSet2ControlPoint),

            // Phase 3: Collect from Parking Zone and score
            collectAndScore(launchClosePose, artifactsSet3Pose, launchClosePose, artifactsSet3Control),

                followPath(launchClosePose, moveToGatePose)
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
                        new InstantCommand(()->robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))
                ),
                new SequentialGroup(
                        new Delay(config.intakeDelaySeconds)
//                    new InstantCommand(()->robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE))
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
     * Prepares the intake for collecting samples
     */
    private Command prepareIntake() {
        return new Command() {
            @Override
            public void start() {
                // Set intake mode if needed
                robot.intake.startForward();
            }

            @Override
            public boolean isDone() {
                return true;
            }
        };
    }

    /**
     * Spins up the launcher and waits until all launchers reach target RPM.
     * Phase 1: Uses position-specific tunable RPM from LaunchAtPositionCommand.PositionRpmConfig
     */
    private Command spinUpLauncher() {
        return launcherCommands.spinUpForPosition(AutoField.FieldPoint.LAUNCH_CLOSE);
    }

    /**
     * Scores samples using the launcher
     */
    private Command scoreSequence() {
        return new SequentialGroup(
            launcherCommands.launchAllAtRangePreset(LauncherRange.SHORT, false)  // Fires all enabled lanes at long range
        );
    }

    private void applyAlliance(Alliance alliance, Pose startOverride) {
        Alliance safeAlliance = alliance != null && alliance != Alliance.UNKNOWN ? alliance : DEFAULT_ALLIANCE;
        activeAlliance = safeAlliance;
        robot.setAlliance(activeAlliance);

        currentLayout = AutoField.layoutForAlliance(activeAlliance);

        // Apply AprilTag-detected start pose override if provided
        if (startOverride != null) {
            currentLayout.overrideStart(startOverride);
            lastAppliedStartPosePedro = copyPose(startOverride);
        }

        Pose startPose = currentLayout.pose(FieldPoint.START_CLOSE);
        robot.drive.getFollower().setStartingPose(startPose);
        robot.drive.getFollower().setPose(startPose);
    }

    private void drawPreviewForAlliance(Alliance alliance) {
        FieldLayout layout = AutoField.layoutForAlliance(alliance);
        // Preview paths would be drawn here using PanelsBridge
        PanelsBridge.drawPreview(new PathChain[0], layout.pose(FieldPoint.LAUNCH_CLOSE), alliance == Alliance.RED);
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
        double fieldWidthIn = FieldConstants.FIELD_WIDTH_INCHES ;
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
