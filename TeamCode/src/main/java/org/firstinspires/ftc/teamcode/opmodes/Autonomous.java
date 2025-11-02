package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldLayout;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldPoint;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.DecodePatterns;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.Arrays;
import java.util.Locale;
import java.util.Optional;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
public class Autonomous extends OpMode {

    private static final int AUTO_BURST_RINGS = 1;
    private static final double POSE_POSITION_TOLERANCE = 1.0; // inches
    private final double POSE_HEADING_TOLERANCE = Math.toRadians(10.0);
    private final Alliance DEFAULT_ALLIANCE = Alliance.BLUE;


    private Robot robot;
    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private Timer stepTimer;
    private ShooterSubsystem shooterSubsystem;
    private LightingSubsystem lighting;
    private VisionSubsystemLimelight vision;

    private Alliance activeAlliance = DEFAULT_ALLIANCE;
    private RoutineStep routineStep = RoutineStep.NOT_STARTED;
    private boolean allianceLocked;

    private PathChain pathToScore;
    private PathChain scoreToPickup;
    private PathChain pickupToStackEnd;
    private PathChain stackToScore;
    private FieldLayout currentLayout;
    private Pose lastAppliedStartPose;
    private Pose lastDetectedStartPose;
    private VisionSubsystemLimelight.TagSnapshot lastTagSnapshot;
    private boolean manualAllianceOverride;
    private Alliance manualAlliance = Alliance.UNKNOWN;
    private boolean lastInitLoopY = false;
    private boolean lastInitLoopLeftBumper = false;
    private boolean lastInitLoopRightBumper = false;
    private boolean lastInitLoopX = false;
    private boolean lastInitLoopB = false;
    private boolean lastInitLoopA = false;
    private boolean lastInitLoopDpadUp = false;
    private boolean lastInitLoopDpadDown = false;
    private String pathToScoreSummary;
    private String scoreToPickupSummary;
    private String pickupToStackEndSummary;
    private String stackToScoreSummary;
    private int manualDecodeIndex = -1;
    private ArtifactColor[] activeDecodePattern = new ArtifactColor[0];
    private boolean opModeStarted = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        shooterSubsystem = robot.shooter;
        lighting = robot.lighting;
        vision = robot.vision;
        follower = robot.drive.getFollower();

        robot.logger.startSession(hardwareMap.appContext, getClass().getSimpleName(), DEFAULT_ALLIANCE, "AutonomousInit");
        panelsTelemetry = robot.telemetry.panelsTelemetry();
        stepTimer = new Timer();

        robot.drive.initialize();
        shooterSubsystem.initialize();
        lighting.initialize();
        vision.initialize();

        allianceLocked = false;
        manualAllianceOverride = false;
        manualAlliance = Alliance.UNKNOWN;

        robot.setAlliance(DEFAULT_ALLIANCE);
        applyAlliance(DEFAULT_ALLIANCE);
    }

    @Override
    public void init_loop() {
        boolean xPressed = gamepad1.x;
        if (xPressed && !lastInitLoopX) {
            setManualAlliance(Alliance.BLUE);
        }
        lastInitLoopX = xPressed;

        boolean bPressed = gamepad1.b;
        if (bPressed && !lastInitLoopB) {
            setManualAlliance(Alliance.RED);
        }
        lastInitLoopB = bPressed;

        boolean aPressed = gamepad1.a;
        if (aPressed && !lastInitLoopA) {
            clearManualAllianceOverride();
        }
        lastInitLoopA = aPressed;

        boolean yPressed = gamepad1.y;
        if (yPressed && !lastInitLoopY) {
            rebuildCurrentAllianceLayout();
        }
        lastInitLoopY = yPressed;

        boolean leftPreview = gamepad1.left_bumper;
        if (leftPreview && !lastInitLoopLeftBumper) {
            drawPreviewForAlliance(Alliance.BLUE);
        }
        lastInitLoopLeftBumper = leftPreview;

        boolean rightPreview = gamepad1.right_bumper;
        if (rightPreview && !lastInitLoopRightBumper) {
            drawPreviewForAlliance(Alliance.RED);
        }
        lastInitLoopRightBumper = rightPreview;

        boolean dpadUp = gamepad1.dpad_up;
        if (dpadUp && !lastInitLoopDpadUp) {
            cycleManualDecodePattern();
        }
        lastInitLoopDpadUp = dpadUp;

        boolean dpadDown = gamepad1.dpad_down;
        if (dpadDown && !lastInitLoopDpadDown) {
            clearManualDecodePattern();
        }
        lastInitLoopDpadDown = dpadDown;

        updateAllianceAndPoseFromAprilTags();

//        if (panelsTelemetry != null) {
//            pushFieldPointPanels();
//            panelsTelemetry.debug("Select alliance: X=Blue, B=Red");
//            panelsTelemetry.debug("Active alliance: " + activeAlliance.displayName());
//            panelsTelemetry.debug("Press Y to rebuild paths after tweaking points");
//            panelsTelemetry.debug("LB: preview blue, RB: preview red");
//            panelsTelemetry.update(telemetry);
//        }
        telemetry.addLine("Press X for Blue or B for Red alliance");
        telemetry.addLine("Press A to return to AprilTag auto-select");
        telemetry.addData("Active alliance", activeAlliance.displayName());
        telemetry.addLine("Press Y to rebuild paths after tweaking waypoints");
        telemetry.addData("Manual override", manualAllianceOverride);
        if (manualAllianceOverride) {
            telemetry.addData("Manual alliance", manualAlliance.displayName());
        }
        telemetry.addData("Detected alliance", lastTagSnapshot == null ? Alliance.UNKNOWN.displayName() : lastTagSnapshot.getAlliance().displayName());
        telemetry.addData("Detected tag", lastTagSnapshot == null ? "none" : lastTagSnapshot.getTagId());
        telemetry.addData("Detected start", formatPose(lastDetectedStartPose));
        telemetry.addData("Raw ftc XYZ", formatRawFtc());
        telemetry.addData("Raw robot XYZ", formatRawRobot());
        telemetry.addData("Decode pattern", formatDecodePattern(activeDecodePattern));
        telemetry.addLine("D-pad ↑ cycle decode pattern, ↓ to clear (testing)");
        telemetry.update();

        robot.logger.logNumber("Autonomous", "RoutineStep", routineStep.ordinal());
        robot.logger.logNumber("Autonomous", "RuntimeSec", getRuntime());
        robot.logger.sampleSources();
        robot.telemetry.updateDriverStation(telemetry);
    }

    @Override
    public void start() {
        allianceLocked = true;
        opModeStarted = true;
        robot.logger.updateAlliance(activeAlliance);
        robot.logger.logEvent("Autonomous", "Start");
        if (lighting != null) {
            if (activeDecodePattern.length > 0) {
                lighting.showDecodePattern(activeDecodePattern);
            } else {
                lighting.indicateIdle();
            }
        }
        if (shooterSubsystem != null) {
            shooterSubsystem.requestSpinUp();
        }
        transitionTo(RoutineStep.DRIVE_TO_PRELOAD_SCORE);
    }

    @Override
    public void loop() {
        robot.drive.updateFollower();
        updateShootingRoutine();
        autonomousStep();

        PanelsBridge.drawFollowerDebug(follower);
        if (panelsTelemetry != null) {
            pushFieldPointPanels();
            panelsTelemetry.debug("Routine step", routineStep.getDisplayName());
            panelsTelemetry.debug("Alliance", activeAlliance.displayName());
            panelsTelemetry.debug("Pose X", follower.getPose().getX());
            panelsTelemetry.debug("Pose Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.debug("Step timer", getStepTimerSeconds());
            if (shooterSubsystem != null) {
                panelsTelemetry.debug("Flywheel state", shooterSubsystem.getState());
                panelsTelemetry.debug("Flywheel busy", isWaitingForShot());
                panelsTelemetry.debug("Flywheel timer", getShotTimerSeconds());
            }
            panelsTelemetry.update(telemetry);
        }

        telemetry.addData("routine step", routineStep.getDisplayName());
        telemetry.addData("alliance", activeAlliance.displayName());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("step timer", getStepTimerSeconds());
        telemetry.addData("flywheel state", getFlywheelStateName());
        telemetry.addData("flywheel busy", isWaitingForShot());
        telemetry.addData("flywheel timer", getShotTimerSeconds());
        if (currentLayout != null) {
            Pose start = currentLayout.pose(FieldPoint.START);
            Pose launch = currentLayout.pose(FieldPoint.LAUNCH_FAR);
            Pose setup = currentLayout.pose(FieldPoint.SETUP_PARKING_ARTIFACTS);
            Pose parking = currentLayout.pose(FieldPoint.PARKING_ARTIFACTS);
            telemetry.addData("Start pose", formatPose(start));
            telemetry.addData("Launch pose", formatPose(launch));
            telemetry.addData("Setup pose", formatPose(setup));
            telemetry.addData("Parking pose", formatPose(parking));
            telemetry.addData("Path Start→Launch", pathToScoreSummary);
            telemetry.addData("Path Launch→Setup", scoreToPickupSummary);
            telemetry.addData("Path Setup→Parking", pickupToStackEndSummary);
            telemetry.addData("Path Parking→Launch", stackToScoreSummary);
        }
//        telemetry.addData("Detected tag", lastDetectedTagId == -1 ? "none" : lastDetectedTagId);
        telemetry.addData("Detected start", formatPose(lastDetectedStartPose));
        telemetry.addData("Raw ftc XYZ", formatRawFtc());
        telemetry.addData("Raw robot XYZ", formatRawRobot());
        telemetry.update();
        robot.logger.sampleSources();
        robot.telemetry.updateDriverStation(telemetry);
    }

    @Override
    public void stop() {
        if (shooterSubsystem != null) {
            shooterSubsystem.abort();
        }
        if (robot != null) {
            robot.drive.stop();
        }
        opModeStarted = false;
        if (vision != null) {
            vision.stop();
        }
        robot.logger.logEvent("Autonomous", "Stop");
        robot.logger.stopSession();
    }

    private void autonomousStep() {
        switch (routineStep) {
            case DRIVE_TO_PRELOAD_SCORE:
                startPath(pathToScore, RoutineStep.WAIT_FOR_PRELOAD_SCORE_PATH);
                break;
            case WAIT_FOR_PRELOAD_SCORE_PATH:
                if (!robot.drive.isFollowerBusy()) {
                    beginShootingRoutine();
                    transitionTo(RoutineStep.WAIT_FOR_PRELOAD_SHOT);
                }
                break;
            case WAIT_FOR_PRELOAD_SHOT:
                if (!isWaitingForShot()) {
                    transitionTo(RoutineStep.DRIVE_TO_PICKUP);
                }
                break;
            case DRIVE_TO_PICKUP:
                startPath(scoreToPickup, RoutineStep.WAIT_FOR_PICKUP_PATH);
                break;
            case WAIT_FOR_PICKUP_PATH:
                if (!robot.drive.isFollowerBusy()) {
                    transitionTo(RoutineStep.DRIVE_THROUGH_STACK);
                }
                break;
            case DRIVE_THROUGH_STACK:
                startPath(pickupToStackEnd, RoutineStep.WAIT_FOR_STACK_PATH);
                break;
            case WAIT_FOR_STACK_PATH:
                if (!robot.drive.isFollowerBusy()) {
                    transitionTo(RoutineStep.DRIVE_BACK_TO_SCORE);
                }
                break;
            case DRIVE_BACK_TO_SCORE:
                startPath(stackToScore, RoutineStep.WAIT_FOR_RETURN_TO_SCORE);
                break;
            case WAIT_FOR_RETURN_TO_SCORE:
                if (!robot.drive.isFollowerBusy()) {
                    beginShootingRoutine();
                    transitionTo(RoutineStep.WAIT_FOR_FINAL_SHOT);
                }
                break;
            case WAIT_FOR_FINAL_SHOT:
                if (!isWaitingForShot()) {
                    transitionTo(RoutineStep.FINISHED);
                }
                break;
            case NOT_STARTED:
            case FINISHED:
                break;
        }
    }


    private void drawPreviewForAlliance(Alliance alliance) {
        FieldLayout layout = AutoField.layoutForAlliance(alliance);
        if (alliance == activeAlliance && lastAppliedStartPose != null) {
            layout.overrideStart(lastAppliedStartPose);
        } else if (lastTagSnapshot != null && lastTagSnapshot.getAlliance() == alliance && lastDetectedStartPose != null) {
            layout.overrideStart(lastDetectedStartPose);
        }
        PathChain[] chains = createPreviewPathChains(layout);
        PanelsBridge.drawPreview(chains, layout.pose(FieldPoint.START), alliance == Alliance.RED);
    }

    private void setManualAlliance(Alliance alliance) {
        manualAllianceOverride = true;
        manualAlliance = alliance;
        Pose override = null;
        if (lastTagSnapshot != null && lastTagSnapshot.getAlliance() == alliance && lastDetectedStartPose != null) {
            override = lastDetectedStartPose;
        } else if (lastAppliedStartPose != null) {
            override = lastAppliedStartPose;
        }
        applyAlliance(alliance, override);
    }

    private void clearManualAllianceOverride() {
        if (!manualAllianceOverride) {
            return;
        }
        manualAllianceOverride = false;
        manualAlliance = Alliance.UNKNOWN;
        updateAllianceAndPoseFromAprilTags();
        if (!manualAllianceOverride && lastAppliedStartPose != null) {
            applyAlliance(activeAlliance, lastAppliedStartPose);
        }
    }

    private void rebuildCurrentAllianceLayout() {
        applyAlliance(activeAlliance, lastAppliedStartPose);
    }

    private void updateAllianceAndPoseFromAprilTags() {
        if (allianceLocked || vision == null || !vision.isStreaming()) {
            return;
        }

        Alliance requiredAlliance = manualAllianceOverride ? manualAlliance : null;
        Optional<VisionSubsystemLimelight.TagSnapshot> snapshotOpt = vision.findAllianceSnapshot(requiredAlliance);
        if (!snapshotOpt.isPresent()) {
            lastTagSnapshot = null;
            lastDetectedStartPose = null;
            return;
        }

        VisionSubsystemLimelight.TagSnapshot snapshot = snapshotOpt.get();
        lastTagSnapshot = snapshot;
        Pose detectedPose = snapshot.getRobotPose().orElse(null);
        lastDetectedStartPose = detectedPose == null ? null : copyPose(detectedPose);

        Alliance detectedAlliance = snapshot.getAlliance();

        if (manualAllianceOverride) {
            if (detectedAlliance == manualAlliance && detectedPose != null && shouldUpdateStartPose(detectedPose)) {
                applyAlliance(activeAlliance, detectedPose);
            }
            return;
        }

        if (detectedAlliance != activeAlliance) {
            applyAlliance(detectedAlliance, detectedPose);
            return;
        }

        if (detectedPose != null && shouldUpdateStartPose(detectedPose)) {
            applyAlliance(activeAlliance, detectedPose);
        }
    }

    private boolean shouldUpdateStartPose(Pose candidate) {
        if (candidate == null) {
            return false;
        }
        if (lastAppliedStartPose == null) {
            return true;
        }

        double dx = candidate.getX() - lastAppliedStartPose.getX();
        double dy = candidate.getY() - lastAppliedStartPose.getY();
        double distance = Math.hypot(dx, dy);
        double headingError = AngleUnit.normalizeRadians(candidate.getHeading() - lastAppliedStartPose.getHeading());
        headingError = Math.abs(headingError);

        return distance > POSE_POSITION_TOLERANCE || headingError > POSE_HEADING_TOLERANCE;
    }

    private PathChain[] createPreviewPathChains(FieldLayout layout) {
        Pose start = layout.pose(FieldPoint.START);
        Pose launch = layout.pose(FieldPoint.LAUNCH_FAR);
        Pose setup = layout.pose(FieldPoint.SETUP_PARKING_ARTIFACTS);
        Pose parking = layout.pose(FieldPoint.PARKING_ARTIFACTS);

        PathChain previewToScore = follower.pathBuilder()
                .addPath(new BezierLine(start, launch))
                .setLinearHeadingInterpolation(start.getHeading(), launch.getHeading())
                .build();

        PathChain previewScoreToPickup = follower.pathBuilder()
                .addPath(new BezierLine(launch, setup))
                .setLinearHeadingInterpolation(launch.getHeading(), setup.getHeading())
                .build();

        PathChain previewPickupToStack = follower.pathBuilder()
                .addPath(new BezierLine(setup, parking))
                .setTangentHeadingInterpolation()
                .build();

        PathChain previewStackToScore = follower.pathBuilder()
                .addPath(new BezierLine(parking, launch))
                .setLinearHeadingInterpolation(parking.getHeading(), launch.getHeading())
                .build();

        return new PathChain[]{previewToScore, previewScoreToPickup, previewPickupToStack, previewStackToScore};
    }

    private void applyAlliance(Alliance alliance) {
        applyAlliance(alliance, null);
    }

    private void applyAlliance(Alliance alliance, Pose startOverride) {
        activeAlliance = alliance;
        if (robot != null) {
            robot.setAlliance(alliance);
            robot.logger.updateAlliance(alliance);
            robot.logger.logEvent("Autonomous", "Alliance-" + alliance.name());
        } else {
            RobotState.setAlliance(alliance);
            if (vision != null) {
                vision.setAlliance(alliance);
            }
            if (lighting != null) {
                lighting.setAlliance(alliance);
            }
        }
        currentLayout = AutoField.layoutForAlliance(alliance);
        if (startOverride != null) {
            currentLayout.overrideStart(startOverride);
        }
        Pose startPose = currentLayout.pose(FieldPoint.START);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        buildPaths(currentLayout);
        cachePathSummaries(currentLayout);
        publishLayoutTelemetry(currentLayout);
        lastAppliedStartPose = copyPose(startPose);
    }

    private void cycleManualDecodePattern() {
        manualDecodeIndex = (manualDecodeIndex + 1) % DecodePatterns.PATTERNS.length;
        applyDecodePattern(DecodePatterns.PATTERNS[manualDecodeIndex]);
    }

    private void clearManualDecodePattern() {
        manualDecodeIndex = -1;
        applyDecodePattern();
    }

    private void applyDecodePattern(ArtifactColor... pattern) {
        if (pattern == null || pattern.length == 0) {
            activeDecodePattern = new ArtifactColor[0];
            if (lighting != null) {
                if (opModeStarted) {
                    lighting.indicateIdle();
                } else {
                    lighting.indicateAllianceInit();
                }
            }
            return;
        }
        activeDecodePattern = Arrays.copyOf(pattern, pattern.length);
        if (lighting != null) {
            lighting.showDecodePattern(activeDecodePattern);
        }
    }

    private void cachePathSummaries(FieldLayout layout) {
        Pose start = layout.pose(FieldPoint.START);
        Pose launch = layout.pose(FieldPoint.LAUNCH_FAR);
        Pose setup = layout.pose(FieldPoint.SETUP_PARKING_ARTIFACTS);
        Pose parking = layout.pose(FieldPoint.PARKING_ARTIFACTS);

        pathToScoreSummary = formatSegment("Start→Launch", start, launch);
        scoreToPickupSummary = formatSegment("Launch→Setup", launch, setup);
        pickupToStackEndSummary = formatSegment("Setup→Parking", setup, parking);
        stackToScoreSummary = formatSegment("Parking→Launch", parking, launch);
    }

    private void publishLayoutTelemetry(FieldLayout layout) {
        if (panelsTelemetry == null) {
            return;
        }
        Pose start = layout.pose(FieldPoint.START);
        Pose launch = layout.pose(FieldPoint.LAUNCH_FAR);
        Pose setup = layout.pose(FieldPoint.SETUP_PARKING_ARTIFACTS);
        Pose parking = layout.pose(FieldPoint.PARKING_ARTIFACTS);

        panelsTelemetry.debug("Start pose", formatPose(start));
        panelsTelemetry.debug("Launch pose", formatPose(launch));
        panelsTelemetry.debug("Setup pose", formatPose(setup));
        panelsTelemetry.debug("Parking pose", formatPose(parking));
        panelsTelemetry.debug("Path Start→Launch", pathToScoreSummary);
        panelsTelemetry.debug("Path Launch→Setup", scoreToPickupSummary);
        panelsTelemetry.debug("Path Setup→Parking", pickupToStackEndSummary);
        panelsTelemetry.debug("Path Parking→Launch", stackToScoreSummary);
    }

    private void buildPaths(FieldLayout layout) {
        Pose start = layout.pose(FieldPoint.START);
        Pose launch = layout.pose(FieldPoint.LAUNCH_FAR);
        Pose setup = layout.pose(FieldPoint.SETUP_PARKING_ARTIFACTS);
        Pose parking = layout.pose(FieldPoint.PARKING_ARTIFACTS);

        pathToScore = follower.pathBuilder()
                .addPath(new BezierLine(start, launch))
                .setLinearHeadingInterpolation(start.getHeading(), launch.getHeading())
                .build();

        scoreToPickup = follower.pathBuilder()
                .addPath(new BezierLine(launch, setup))
                .setLinearHeadingInterpolation(launch.getHeading(), setup.getHeading())
                .build();

        pickupToStackEnd = follower.pathBuilder()
                .addPath(new BezierLine(setup, parking))
                .setTangentHeadingInterpolation()
                .build();

        stackToScore = follower.pathBuilder()
                .addPath(new BezierLine(parking, launch))
                .setLinearHeadingInterpolation(parking.getHeading(), launch.getHeading())
                .build();
    }

    private void transitionTo(RoutineStep nextStep) {
        routineStep = nextStep;
        if (stepTimer != null) {
            stepTimer.resetTimer();
        }
    }

    private void startPath(PathChain pathChain, RoutineStep waitingStep) {
        robot.drive.followPath(pathChain, true);
        transitionTo(waitingStep);
    }

    private void beginShootingRoutine() {
        if (shooterSubsystem != null) {
            shooterSubsystem.requestBurst(AUTO_BURST_RINGS);
        }
    }

    private void updateShootingRoutine() {
        if (shooterSubsystem != null) {
            shooterSubsystem.periodic();
        }
    }

    private boolean isWaitingForShot() {
        return shooterSubsystem != null && shooterSubsystem.isBusy();
    }

    private double getShotTimerSeconds() {
        return shooterSubsystem != null ? shooterSubsystem.getStateElapsedSeconds() : 0.0;
    }

    private double getStepTimerSeconds() {
        return stepTimer != null ? stepTimer.getElapsedTimeSeconds() : 0.0;
    }

    private String getFlywheelStateName() {
        return shooterSubsystem != null ? shooterSubsystem.getState().name() : "N/A";
    }

    private static String formatSegment(String label, Pose start, Pose end) {
        return label + ": " + formatPose(start) + " → " + formatPose(end);
    }

    private String formatDecodePattern(ArtifactColor[] pattern) {
        if (pattern == null || pattern.length == 0) {
            return "(none)";
        }
        StringBuilder sb = new StringBuilder();
        for (ArtifactColor color : pattern) {
            char code;
            if (color == ArtifactColor.GREEN) {
                code = 'G';
            } else if (color == ArtifactColor.PURPLE) {
                code = 'P';
            } else if (color == ArtifactColor.UNKNOWN) {
                code = '?';
            } else {
                code = '-';
            }
            sb.append(code);
        }
        return sb.toString();
    }

    private static String formatPose(Pose pose) {
        if (pose == null) {
            return "(null)";
        }
        return String.format(Locale.US, "(%.3f, %.3f, %.1f°)",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }

    private static Pose copyPose(Pose pose) {
        if (pose == null) {
            return null;
        }
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    private String formatRawFtc() {
        if (lastTagSnapshot == null) {
            return "(--, --, --)";
        }
        double x = lastTagSnapshot.getFtcX();
        double y = lastTagSnapshot.getFtcY();
        double yaw = lastTagSnapshot.getFtcYaw();
        if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(yaw)) {
            return "(--, --, --)";
        }
        return String.format(Locale.US, "(%.2f, %.2f, %.1f°)", x, y, yaw);
    }

    private String formatRawRobot() {
        if (lastTagSnapshot == null) {
            return "(--, --, --)";
        }
        double x = lastTagSnapshot.getRobotX();
        double y = lastTagSnapshot.getRobotY();
        double yaw = lastTagSnapshot.getRobotYaw();
        if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(yaw)) {
            return "(--, --, --)";
        }
        return String.format(Locale.US, "(%.2f, %.2f, %.1f°)", x, y, yaw);
    }

    private void pushFieldPointPanels() {
        if (panelsTelemetry == null || currentLayout == null) {
            return;
        }
        Pose start = currentLayout.pose(FieldPoint.START);
        Pose launch = currentLayout.pose(FieldPoint.LAUNCH_FAR);
        Pose setup = currentLayout.pose(FieldPoint.SETUP_PARKING_ARTIFACTS);
        Pose parking = currentLayout.pose(FieldPoint.PARKING_ARTIFACTS);

        panelsTelemetry.debug("Start pose", formatPose(start));
        panelsTelemetry.debug("Launch pose", formatPose(launch));
        panelsTelemetry.debug("Setup pose", formatPose(setup));
        panelsTelemetry.debug("Parking pose", formatPose(parking));
        panelsTelemetry.debug("Path Start→Launch", pathToScoreSummary);
        panelsTelemetry.debug("Path Launch→Setup", scoreToPickupSummary);
        panelsTelemetry.debug("Path Setup→Parking", pickupToStackEndSummary);
        panelsTelemetry.debug("Path Parking→Launch", stackToScoreSummary);
    }

    private enum RoutineStep {
        NOT_STARTED("Not started"),
        DRIVE_TO_PRELOAD_SCORE("Drive to preload score"),
        WAIT_FOR_PRELOAD_SCORE_PATH("Wait for preload score path"),
        WAIT_FOR_PRELOAD_SHOT("Wait for preload shot"),
        DRIVE_TO_PICKUP("Drive to pickup"),
        WAIT_FOR_PICKUP_PATH("Wait for pickup path"),
        DRIVE_THROUGH_STACK("Drive through stack"),
        WAIT_FOR_STACK_PATH("Wait for stack path"),
        DRIVE_BACK_TO_SCORE("Drive back to score"),
        WAIT_FOR_RETURN_TO_SCORE("Wait for return to score path"),
        WAIT_FOR_FINAL_SHOT("Wait for final shot"),
        FINISHED("Finished");

        private final String displayName;

        RoutineStep(String displayName) {
            this.displayName = displayName;
        }

        String getDisplayName() {
            return displayName;
        }
    }

}
