package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldLayout;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldPoint;
import org.firstinspires.ftc.teamcode.util.DecodePatternController;

import java.util.Arrays;
import java.util.Locale;
import java.util.Optional;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Decode Autonomous", group = "Autonomous")
public class Autonomous extends OpMode {

    private static final int AUTO_BURST_RINGS = 1;
    private static final Alliance DEFAULT_ALLIANCE = Alliance.BLUE;

    private static final double POSE_POSITION_TOLERANCE = 1.0; // inches
    private static final double POSE_HEADING_TOLERANCE = Math.toRadians(10.0);

    private Robot robot;
    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private AllianceSelector allianceSelector;

    private Timer stepTimer;
    private RoutineStep routineStep = RoutineStep.NOT_STARTED;

    private FieldLayout currentLayout;
    private Pose lastAppliedStartPose;
    private Pose lastDetectedStartPose;
    private VisionSubsystemLimelight.TagSnapshot lastTagSnapshot;
    private Alliance activeAlliance = Alliance.BLUE;

    private PathChain pathToScore;
    private PathChain scoreToPickup;
    private PathChain pickupToStackEnd;
    private PathChain stackToScore;

    private String pathToScoreSummary;
    private String scoreToPickupSummary;
    private String pickupToStackEndSummary;
    private String stackToScoreSummary;

    private final DecodePatternController decodeController = new DecodePatternController();
    private ArtifactColor[] activeDecodePattern = new ArtifactColor[0];
    private boolean opModeStarted;

    @Override
    public void init() {
        BindingManager.reset();
        robot = new Robot(hardwareMap);
        robot.logger.startSession(hardwareMap.appContext, getClass().getSimpleName(), DEFAULT_ALLIANCE, "AutonomousInit");
        panelsTelemetry = robot.telemetry.panelsTelemetry();
        stepTimer = new Timer();

        robot.initialize();
        follower = robot.drive.getFollower();

        // Register init-phase controls with NextFTC (selector handles alliance overrides automatically).
        GamepadEx driverPad = new GamepadEx(() -> gamepad1);
        allianceSelector = new AllianceSelector(driverPad, DEFAULT_ALLIANCE);
        driverPad.y().whenBecomesTrue(() -> applyAlliance(activeAlliance, lastAppliedStartPose));
        driverPad.leftBumper().whenBecomesTrue(() -> drawPreviewForAlliance(Alliance.BLUE));
        driverPad.rightBumper().whenBecomesTrue(() -> drawPreviewForAlliance(Alliance.RED));
        driverPad.dpadUp().whenBecomesTrue(() -> applyDecodePattern(decodeController.cycleNext()));
        driverPad.dpadDown().whenBecomesTrue(() -> applyDecodePattern(decodeController.clear()));

        activeAlliance = allianceSelector.getSelectedAlliance();
        applyAlliance(activeAlliance, null);
        allianceSelector.applySelection(robot, robot.lighting);
        applyDecodePattern(decodeController.current()); // default to alliance colour until a pattern is chosen
    }

    @Override
    public void init_loop() {
        BindingManager.update();

        // Blend camera detections with any driver override and apply the combined alliance immediately.
        Optional<VisionSubsystemLimelight.TagSnapshot> snapshotOpt =
                allianceSelector.updateFromVision(robot.vision);

        allianceSelector.applySelection(robot, robot.lighting);

        lastTagSnapshot = snapshotOpt.orElse(null);
        Pose snapshotPose = lastTagSnapshot == null ? null : lastTagSnapshot.getRobotPose().orElse(null);
        lastDetectedStartPose = snapshotPose == null ? null : copyPose(snapshotPose);

        Alliance selectedAlliance = allianceSelector.getSelectedAlliance();
        if (selectedAlliance != activeAlliance) {
            activeAlliance = selectedAlliance;
            applyAlliance(activeAlliance, snapshotPose);
        } else if (snapshotPose != null && shouldUpdateStartPose(snapshotPose)) {
            applyAlliance(activeAlliance, snapshotPose);
        }

        telemetry.addData("Selected alliance", selectedAlliance);
        telemetry.addData("Active alliance", activeAlliance);
        telemetry.addData("Detected alliance", allianceSelector.getDetectedAlliance());
        telemetry.addData("Manual override", allianceSelector.isManualOverrideActive());
        telemetry.addData("Detected tag", allianceSelector.getDetectedTagId() == -1 ? "none" : allianceSelector.getDetectedTagId());
        telemetry.addData("Detected range", formatDouble(allianceSelector.getDetectedRange()));
        telemetry.addData("Detected bearing", formatDouble(allianceSelector.getDetectedYaw()) + "°");
        telemetry.addData("Detected start", formatPose(lastDetectedStartPose));
        telemetry.addData("Applied start", formatPose(lastAppliedStartPose));
        telemetry.addData(" Layout start", currentLayout == null ? "(null)" : formatPose(currentLayout.pose(FieldPoint.START)));
        telemetry.addData("Raw ftc XYZ", formatRawFtc());
        telemetry.addData("Raw robot XYZ", formatRawRobot());
        telemetry.addData("Decode pattern", formatDecodePattern(activeDecodePattern));
        telemetry.addLine("D-pad Left/Right override alliance, Down returns to auto");
        telemetry.addLine("LB/RB preview other alliance, Y rebuilds layout");
        telemetry.addLine("D-pad Up cycles decode pattern, Down clears it");
        telemetry.update();

        robot.logger.logNumber("AutonomousDHS", "RoutineStep", routineStep.ordinal());
        robot.logger.logNumber("AutonomousDHS", "RuntimeSec", getRuntime());
        robot.logger.sampleSources();
        robot.telemetry.updateDriverStation(telemetry);
    }

    @Override
    public void start() {
        BindingManager.reset();
        opModeStarted = true;
        allianceSelector.lockSelection();
        allianceSelector.applySelection(robot, robot.lighting);
        robot.logger.updateAlliance(activeAlliance);
        robot.logger.logEvent("AutonomousDHS", "Start");

        LightingSubsystem lighting = robot.lighting;
        if (lighting != null) {
            if (activeDecodePattern.length > 0) {
                lighting.showDecodePattern(activeDecodePattern);
            } else {
                lighting.indicateIdle();
            }
        }

        ShooterSubsystem shooter = robot.shooter;
        if (shooter != null) {
            shooter.requestSpinUp();
        }

        transitionTo(RoutineStep.DRIVE_TO_PRELOAD_SCORE);
    }

    @Override
    public void loop() {
        BindingManager.update();
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
            ShooterSubsystem shooter = robot.shooter;
            if (shooter != null) {
                panelsTelemetry.debug("Flywheel state", shooter.getState());
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
        telemetry.addData("Detected start", formatPose(lastDetectedStartPose));
        telemetry.addData("Applied start", formatPose(lastAppliedStartPose));
        telemetry.addData("Layout start", currentLayout == null ? "(null)" : formatPose(currentLayout.pose(FieldPoint.START)));
        telemetry.addData("Raw ftc XYZ", formatRawFtc());
        telemetry.addData("Raw robot XYZ", formatRawRobot());
        telemetry.update();

        robot.logger.logNumber("AutonomousDHS", "RoutineStep", routineStep.ordinal());
        robot.logger.logNumber("AutonomousDHS", "RuntimeSec", getRuntime());
        robot.logger.sampleSources();
        robot.telemetry.updateDriverStation(telemetry);
    }

    @Override
    public void stop() {
        opModeStarted = false;
        allianceSelector.unlockSelection();
        BindingManager.reset();

        ShooterSubsystem shooter = robot.shooter;
        if (shooter != null) {
            shooter.abort();
        }
        robot.drive.stop();
        robot.vision.stop();
        robot.logger.logEvent("AutonomousDHS", "Stop");
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
        Alliance safeAlliance = alliance;
        if (safeAlliance == null || safeAlliance == Alliance.UNKNOWN) {
            safeAlliance = DEFAULT_ALLIANCE;
        }
        activeAlliance = safeAlliance;
        robot.setAlliance(activeAlliance);
        robot.logger.updateAlliance(activeAlliance);
        robot.logger.logEvent("AutonomousDHS", "Alliance-" + activeAlliance.name());

        currentLayout = AutoField.layoutForAlliance(activeAlliance);
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

    private void applyDecodePattern(ArtifactColor[] pattern) {
        LightingSubsystem lighting = robot.lighting;
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
        ShooterSubsystem shooter = robot.shooter;
        if (shooter != null) {
            shooter.requestBurst(AUTO_BURST_RINGS);
        }
    }

    private void updateShootingRoutine() {
        ShooterSubsystem shooter = robot.shooter;
        if (shooter != null) {
            shooter.periodic();
        }
    }

    private boolean isWaitingForShot() {
        ShooterSubsystem shooter = robot.shooter;
        return shooter != null && shooter.isBusy();
    }

    private double getShotTimerSeconds() {
        ShooterSubsystem shooter = robot.shooter;
        return shooter != null ? shooter.getStateElapsedSeconds() : 0.0;
    }

    private double getStepTimerSeconds() {
        return stepTimer != null ? stepTimer.getElapsedTimeSeconds() : 0.0;
    }

    private String getFlywheelStateName() {
        ShooterSubsystem shooter = robot.shooter;
        return shooter != null ? shooter.getState().name() : "N/A";
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

    private static String formatDouble(double value) {
        if (Double.isNaN(value)) {
            return "--";
        }
        return String.format(Locale.US, "%.2f", value);
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
