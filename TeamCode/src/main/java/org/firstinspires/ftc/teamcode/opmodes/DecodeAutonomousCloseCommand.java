package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.util.AutoField.poseForAlliance;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
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
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Command-based version of DecodeAutonomousClose using Sequential and Parallel Command Groups
 * This autonomous routine:
 * 1. Starts at LAUNCH_CLOSE position
 * 2. Collects samples from Gate Close, Gate Far, and Parking Zone
 * 3. Scores each sample set at LAUNCH_CLOSE
 *m
 * Uses NextFTC command framework to coordinate:
 * - Driving with intake/launcher positioning (parallel)
 * - Sequential scoring and collection routines
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Fixed Auto Close (Command)", group = "Command")
@Configurable
public class DecodeAutonomousCloseCommand extends NextFTCOpMode {

    private static final Alliance DEFAULT_ALLIANCE = Alliance.BLUE;

    /**
     * Named waypoints for the close autonomous routine.
     * This enum provides type-safe references to all positions used in the routine.
     */
    public enum Waypoint {
        START_CLOSE,
        LAUNCH_CLOSE,
        PRE_ARTIFACTS,
        ARTIFACTS_SET_1,
        TRANSITION_TO_SET_2,
        ARTIFACTS_SET_2,
        TRANSITION_TO_SET_3,
        ARTIFACTS_SET_3,
        FINAL_POSITION
    }

    @Configurable
    public static class AutoMotionConfig {
        public double maxPathPower = .79;
        public double intakeDelaySeconds = 2.5;
        public int relocalizeMaxAttempts = 10;
        /**
         * Starting launcher mode for autonomous.
         * DECODE: Fire in obelisk pattern sequence (recommended for endgame scoring)
         * THROUGHPUT: Rapid fire all lanes (recommended for early match throughput)
         */
        public LauncherMode startingLauncherMode = LauncherMode.THROUGHPUT;
    }

    /**
     * All waypoint coordinates for the close autonomous routine.
     * Editable via FTC Dashboard for quick field adjustments.
     */
    @Configurable
    public static class CloseAutoWaypoints {
        // Start position (set by AutoField)
        public double startX = 26.445;
        public double startY = 131.374;
        public double startHeading = 144;

        // Launch position for scoring
        public double launchX = 30.19905213270142;
        public double launchY = 112.9478672985782;
        public double launchHeading = 136;

        // Pre-artifacts waypoint (transition after first score)
        public double preArtifactsX = 28;
        public double preArtifactsY = 112;
        public double preArtifactsHeading = 270;

        // Artifacts Set 1 (gate close)
        public double artifacts1X = 22;
        public double artifacts1Y = 87;
        public double artifacts1Heading = 270;

        // Transition to Set 2
        public double transition2X = 40;
        public double transition2Y = 90;
        public double transition2Heading = 270;

        // Artifacts Set 2 (gate far)
        public double artifacts2X = 34;
        public double artifacts2Y = 70;
        public double artifacts2Heading = 270;

        // Transition to Set 3
        public double transition3X = 40;
        public double transition3Y = 62; // Why was 6 afraid of 7? Because 7 ate 9!
        public double transition3Heading = 270;

        // Artifacts Set 3 (parking zone)
        public double artifacts3X = 40;
        public double artifacts3Y = 32.5;
        public double artifacts3Heading = 270;

        // Final position
        public double finalX = 37;
        public double finalY = 128;
        public double finalHeading = 142;
    }

    // Public static instances for FTC Dashboard Config tab
    public static AutoMotionConfig config = new AutoMotionConfig();
    public static CloseAutoWaypoints waypoints = new CloseAutoWaypoints();

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
//        driverPad.y().whenBecomesTrue(() -> applyAlliance(allianceSelector.getSelectedAlliance(), lastAppliedStartPosePedro));
//        driverPad.leftBumper().whenBecomesTrue(() -> drawPreviewForAlliance(Alliance.BLUE));
//        driverPad.rightBumper().whenBecomesTrue(() -> drawPreviewForAlliance(Alliance.RED));
//        driverPad.a().whenBecomesTrue(this::applyLastDetectedStartPose);
//        lightingInitController = new LightingSubsystem.InitController(robot, allianceSelector, robot.lighting);
//        lightingInitController.initialize();

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

        robot.launcher.spinUpAllLanesToLaunch();

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
        publishTelemetry();
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
     * Gets a pose for a waypoint, mirrored for the current alliance.
     * @param waypoint The waypoint enum value
     * @return Alliance-mirrored pose for the waypoint
     */
    private Pose pose(Waypoint waypoint) {
        CloseAutoWaypoints w = waypoints;
        switch (waypoint) {
            case START_CLOSE:
                return poseForAlliance(w.startX, w.startY, w.startHeading, activeAlliance);
            case LAUNCH_CLOSE:
                return poseForAlliance(w.launchX, w.launchY, w.launchHeading, activeAlliance);
            case PRE_ARTIFACTS:
                return poseForAlliance(w.preArtifactsX, w.preArtifactsY, w.preArtifactsHeading, activeAlliance);
            case ARTIFACTS_SET_1:
                return poseForAlliance(w.artifacts1X, w.artifacts1Y, w.artifacts1Heading, activeAlliance);
            case TRANSITION_TO_SET_2:
                return poseForAlliance(w.transition2X, w.transition2Y, w.transition2Heading, activeAlliance);
            case ARTIFACTS_SET_2:
                return poseForAlliance(w.artifacts2X, w.artifacts2Y, w.artifacts2Heading, activeAlliance);
            case TRANSITION_TO_SET_3:
                return poseForAlliance(w.transition3X, w.transition3Y, w.transition3Heading, activeAlliance);
            case ARTIFACTS_SET_3:
                return poseForAlliance(w.artifacts3X, w.artifacts3Y, w.artifacts3Heading, activeAlliance);
            case FINAL_POSITION:
                return poseForAlliance(w.finalX, w.finalY, w.finalHeading, activeAlliance);
            default:
                throw new IllegalArgumentException("Unknown waypoint: " + waypoint);
        }
    }

    /**
     * Creates a relocalization command that uses AprilTag vision to correct odometry drift.
     * This command attempts to update the robot's pose using vision for a configured number of attempts.
     * It's designed to run at the launch position where AprilTags are visible.
     *
     * @return Command that relocalizes using vision
     */
    private Command relocalize() {
        return new Command() {
            private int attempts = 0;
            private boolean success = false;

            @Override
            public void start() {
                attempts = 0;
                success = false;
            }

            @Override
            public void update() {
                if (robot.vision.hasValidTag()) {
                    success = robot.drive.forceRelocalizeFromVision();
                    if (success) {
                        // Exit early on success
                        attempts = config.relocalizeMaxAttempts;
                    }
                }
                attempts++;
            }

            @Override
            public boolean isDone() {
                return attempts >= config.relocalizeMaxAttempts || success;
            }
        };
    }

    /**
     * Builds the complete autonomous routine using command groups
     */
    private Command buildAutonomousRoutine() {
        if (currentLayout == null) {
            return new Delay(0.01);
        }

        return new SequentialGroup(
                // Move to launch position and spin up
                new ParallelGroup(
                        launcherCommands.presetRangeSpinUp(LauncherRange.SHORT, true),
                        followPath(pose(Waypoint.START_CLOSE), pose(Waypoint.LAUNCH_CLOSE))
                ),
                // Relocalize using AprilTag at launch position
                relocalize(),

                launcherCommands.launchAll(false),

                // Transition to artifact collection zone
                followPath(pose(Waypoint.LAUNCH_CLOSE), pose(Waypoint.PRE_ARTIFACTS)),

                // Phase 1: Collect from Gate Close and score
                collectAndScore(
                        pose(Waypoint.PRE_ARTIFACTS),
                        pose(Waypoint.ARTIFACTS_SET_1),
                        pose(Waypoint.LAUNCH_CLOSE)
                ),

                // Transition to Set 2
                followPath(pose(Waypoint.LAUNCH_CLOSE), pose(Waypoint.TRANSITION_TO_SET_2)),

                // Phase 2: Collect from Gate Far and score
                collectAndScore(
                        pose(Waypoint.TRANSITION_TO_SET_2),
                        pose(Waypoint.ARTIFACTS_SET_2),
                        pose(Waypoint.LAUNCH_CLOSE)
                ),

                // Transition to Set 3
                followPath(pose(Waypoint.LAUNCH_CLOSE), pose(Waypoint.TRANSITION_TO_SET_3)),

                // Phase 3: Collect from Parking Zone and finish
                collectAndScore(
                        pose(Waypoint.TRANSITION_TO_SET_3),
                        pose(Waypoint.ARTIFACTS_SET_3),
                        pose(Waypoint.FINAL_POSITION)
                )
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


                // Drive to score while spinning up launcher
                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(config.intakeDelaySeconds),
                                new InstantCommand(()->robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE))
                        ),
                        followPath(pickupPose, scorePose),
                        launcherCommands.presetRangeSpinUp(LauncherRange.SHORT, true)
                ),

                // Score the samples
                launcherCommands.launchAll(false)
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
        PanelsBridge.drawPreview(new PathChain[0], layout.pose(FieldPoint.START_CLOSE), alliance == Alliance.RED);
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


    private void publishTelemetry() {
        robot.telemetry.publishLoopTelemetry(
                robot.drive,
                robot.launcher,
                robot.intake,
                robot.vision,
                robot.lighting,
                null,
                gamepad1,
                gamepad2,
                RobotState.getAlliance(),
                getRuntime(),
                Math.max(0.0, 150.0 - getRuntime()),
                telemetry,
                "Auto",
                true,
                null,
                0,
                0
        );
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
                    .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading(), .7)
                    .build();
        } else {
            // Straight line
            return follower.pathBuilder()
                    .addPath(new BezierLine(startPose, endPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading(), .7)
                    .build();
        }
    }
}
