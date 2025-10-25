package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.Alliance;
import org.firstinspires.ftc.teamcode.auto.AllianceSelector;
import org.firstinspires.ftc.teamcode.auto.AprilTagPoseUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;

/**
 * Demonstrates an autonomous routine that uses AprilTags for alliance selection, initial
 * localization, and pre-shot alignment.
 */
@Autonomous(name = "Example Auto 4", group = "Examples")
public class ExampleAuto4 extends OpMode {

    private static final double SHOT_SETTLE_SECONDS = 1.0;
    private static final double MIN_IDLE_TIME_FOR_ZERO_LENGTH_PATH = 0.05;
    private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(2.5);
    private static final double MAX_ALIGNMENT_SECONDS = 2.5;

    private static final int BLUE_ALLIANCE_TAG_ID = 20;
    private static final int RED_ALLIANCE_TAG_ID = 24;

    private static final Alliance DEFAULT_ALLIANCE = Alliance.BLUE;

    private static final Position CAMERA_POSITION = new Position(
            DistanceUnit.INCH,
            0.0,   // X right
            7.0,   // Y forward
            8.5,   // Z up
            0
    );
    private static final YawPitchRollAngles CAMERA_ORIENTATION = new YawPitchRollAngles(
            AngleUnit.DEGREES,
            0.0,   // yaw (camera faces forward)
            -90.0, // pitch to make the camera horizontal
            0.0,
            0
    );

    private Follower follower;
    private Timer stepTimer;
    private Timer shotTimer;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    private PathChain scorePreload;
    private PathChain grabPickup1;
    private PathChain scorePickup1;
    private PathChain grabPickup2;
    private PathChain scorePickup2;
    private PathChain grabPickup3;
    private PathChain scorePickup3;

    private final Deque<AutoStep> routineSteps = new ArrayDeque<>();

    private AutoStep activeStep;
    private boolean routineStarted;
    private boolean routineFinished;
    private String activeStepLabel = "Queued";
    private int totalSteps;
    private int completedSteps;

    private AllianceSelector allianceSelector;
    private Alliance lastAppliedAlliance;
    private GamepadEx driverPad;
    private FieldLayout activeLayout;
    private Pose lastBuiltStartPose;
    private Pose seededStartPose;
    private Pose lastLocalizationPose;

    private boolean aligningForShot;
    private String alignmentStatus = "idle";
    private int lastAlignmentTagId = -1;
    private double lastHeadingErrorDeg = Double.NaN;
    private boolean alignmentTimedOut;

    private interface AutoStep {
        String name();

        void start();

        boolean isComplete();

        void onComplete();
    }

    private static class FieldLayout {
        final Pose startPose;
        final Pose scorePose;
        final Pose pickup1Pose;
        final Pose pickup2Pose;
        final Pose pickup3Pose;

        FieldLayout(Pose startPose, Pose scorePose, Pose pickup1Pose, Pose pickup2Pose, Pose pickup3Pose) {
            this.startPose = startPose;
            this.scorePose = scorePose;
            this.pickup1Pose = pickup1Pose;
            this.pickup2Pose = pickup2Pose;
            this.pickup3Pose = pickup3Pose;
        }

        static FieldLayout forAlliance(Alliance alliance, Pose startOverride) {
            Pose blueStart = new Pose(57, 9, Math.toRadians(90));
            Pose blueScore = new Pose(57, 18, Math.toRadians(115));
            Pose bluePickup1 = new Pose(12, 12, Math.toRadians(180));
            Pose bluePickup2 = new Pose(24, 36, Math.toRadians(90));
            Pose bluePickup3 = new Pose(24, 60, Math.toRadians(90));

            if (alliance == Alliance.RED) {
                Pose startPose = startOverride != null ? startOverride : mirrorPoseAcrossField(blueStart);
                return new FieldLayout(
                        startPose,
                        mirrorPoseAcrossField(blueScore),
                        mirrorPoseAcrossField(bluePickup1),
                        mirrorPoseAcrossField(bluePickup2),
                        mirrorPoseAcrossField(bluePickup3)
                );
            }

            Pose startPose = startOverride != null ? startOverride : blueStart;
            return new FieldLayout(startPose, blueScore, bluePickup1, bluePickup2, bluePickup3);
        }

        private static Pose mirrorPoseAcrossField(Pose pose) {
            double mirroredHeading = AngleUnit.normalizeRadians(Math.PI - pose.getHeading());
            return new Pose(-pose.getX(), pose.getY(), mirroredHeading);
        }
    }

    private class FollowPathStep implements AutoStep {
        private final String label;
        private final Runnable command;
        private boolean seenBusy;
        private boolean started;

        FollowPathStep(String label, Runnable command) {
            this.label = label;
            this.command = command;
        }

        @Override
        public String name() {
            return label;
        }

        @Override
        public void start() {
            command.run();
            started = true;
            seenBusy = follower.isBusy();
        }

        @Override
        public boolean isComplete() {
            if (!started) {
                return false;
            }
            if (follower.isBusy()) {
                seenBusy = true;
                return false;
            }
            if (!seenBusy) {
                return stepTimer.getElapsedTimeSeconds() > MIN_IDLE_TIME_FOR_ZERO_LENGTH_PATH;
            }
            return true;
        }

        @Override
        public void onComplete() {
            // No additional teardown required.
        }
    }

    private class AlignAndShootStep implements AutoStep {
        private final String label;
        private final double settleSeconds;
        private final Timer alignTimer = new Timer();

        private boolean started;
        private boolean alignmentComplete;
        private boolean correctionInProgress;

        AlignAndShootStep(String label, double settleSeconds) {
            this.label = label;
            this.settleSeconds = settleSeconds;
        }

        @Override
        public String name() {
            return label;
        }

        @Override
        public void start() {
            started = true;
            alignmentComplete = false;
            correctionInProgress = false;
            alignTimer.resetTimer();
            shotTimer.resetTimer();
            beginShotAlignment(label);
            follower.pausePathFollowing();
        }

        @Override
        public boolean isComplete() {
            if (!started) {
                return false;
            }

            if (correctionInProgress) {
                if (follower.isBusy()) {
                    alignmentStatus = "executing heading correction";
                    return false;
                }
                correctionInProgress = false;
                follower.pausePathFollowing();
                alignmentStatus = "rechecking AprilTag";
            }

            AprilTagDetection detection = getDetectionForSelectedAlliance();
            if (detection != null) {
                lastAlignmentTagId = detection.id;
                Pose pose = AprilTagPoseUtil.toPedroPose(detection);
                if (pose != null) {
                    lastLocalizationPose = pose;
                    follower.setPose(pose);
                }

                double targetHeading = activeLayout != null
                        ? activeLayout.scorePose.getHeading()
                        : follower.getPose().getHeading();

                double headingError = AngleUnit.normalizeRadians(targetHeading - follower.getPose().getHeading());
                lastHeadingErrorDeg = Math.toDegrees(headingError);

                if (Math.abs(headingError) <= HEADING_TOLERANCE_RADIANS) {
                    if (!alignmentComplete) {
                        alignmentComplete = true;
                        shotTimer.resetTimer();
                    }
                    alignmentStatus = "aligned; waiting to shoot";
                } else if (!correctionInProgress) {
                    PathChain correction = buildHeadingCorrectionPath(targetHeading);
                    if (correction != null) {
                        follower.resumePathFollowing();
                        follower.followPath(correction);
                        correctionInProgress = true;
                        alignmentStatus = String.format("correcting heading (%.1f°)", Math.toDegrees(headingError));
                    } else {
                        alignmentStatus = "unable to build correction";
                    }
                }
            } else {
                lastAlignmentTagId = -1;
                lastHeadingErrorDeg = Double.NaN;
                alignmentStatus = "waiting for AprilTag";
            }

            if (!alignmentComplete) {
                if (alignTimer.getElapsedTimeSeconds() >= MAX_ALIGNMENT_SECONDS) {
                    alignmentComplete = true;
                    alignmentTimedOut = true;
                    alignmentStatus = "alignment timeout - continuing";
                    shotTimer.resetTimer();
                }
                return false;
            }

            if (shotTimer.getElapsedTimeSeconds() >= settleSeconds) {
                return true;
            }
            return false;
        }

        @Override
        public void onComplete() {
            finishShotAlignment();
            follower.resumePathFollowing();
            alignmentStatus = "shot window complete";
        }
    }

    private void beginShotAlignment(String label) {
        aligningForShot = true;
        alignmentTimedOut = false;
        alignmentStatus = "acquiring AprilTag";
        activeStepLabel = label;
    }

    private void finishShotAlignment() {
        aligningForShot = false;
    }

    private PathChain buildHeadingCorrectionPath(double targetHeading) {
        if (follower == null) {
            return null;
        }
        Pose current = follower.getPose();
        Pose start = new Pose(current.getX(), current.getY(), current.getHeading());
        Pose end = new Pose(current.getX(), current.getY(), targetHeading);
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    private void buildPaths(Pose startPose, FieldLayout layout) {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, layout.scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), layout.scorePose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(layout.scorePose, layout.pickup1Pose))
                .setLinearHeadingInterpolation(layout.scorePose.getHeading(), layout.pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(layout.pickup1Pose, layout.scorePose))
                .setLinearHeadingInterpolation(layout.pickup1Pose.getHeading(), layout.scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(layout.scorePose, layout.pickup2Pose))
                .setLinearHeadingInterpolation(layout.scorePose.getHeading(), layout.pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(layout.pickup2Pose, layout.scorePose))
                .setLinearHeadingInterpolation(layout.pickup2Pose.getHeading(), layout.scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(layout.scorePose, layout.pickup3Pose))
                .setLinearHeadingInterpolation(layout.scorePose.getHeading(), layout.pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(layout.pickup3Pose, layout.scorePose))
                .setLinearHeadingInterpolation(layout.pickup3Pose.getHeading(), layout.scorePose.getHeading())
                .build();
    }

    private void queueFollow(PathChain chain, String label) {
        routineSteps.addLast(new FollowPathStep(label, () -> follower.followPath(chain)));
    }

    private void queueAlignAndShoot(String label) {
        routineSteps.addLast(new AlignAndShootStep(label, SHOT_SETTLE_SECONDS));
    }

    private void buildRoutine() {
        routineSteps.clear();
        activeStep = null;
        routineStarted = false;
        routineFinished = false;
        completedSteps = 0;
        activeStepLabel = "Queued";
        stepTimer.resetTimer();
        shotTimer.resetTimer();

        queueFollow(scorePreload, "Score preload");
        queueAlignAndShoot("Align & shoot preload");

        queueFollow(grabPickup1, "Drive to pickup 1");
        queueFollow(scorePickup1, "Return to score 1");
        queueAlignAndShoot("Align & shoot pickup 1");

        queueFollow(grabPickup2, "Drive to pickup 2");
        queueFollow(scorePickup2, "Return to score 2");
        queueAlignAndShoot("Align & shoot pickup 2");

        queueFollow(grabPickup3, "Drive to pickup 3");
        queueFollow(scorePickup3, "Return to score 3");
        queueAlignAndShoot("Align & shoot pickup 3");

        totalSteps = routineSteps.size();
    }

    private void startNextStep() {
        if (!routineStarted) {
            return;
        }

        if (routineSteps.isEmpty()) {
            activeStep = null;
            if (!routineFinished) {
                routineFinished = true;
                activeStepLabel = "Done";
            }
            return;
        }

        activeStep = routineSteps.pollFirst();
        activeStepLabel = activeStep.name();
        stepTimer.resetTimer();
        activeStep.start();
    }

    private void finishActiveStep() {
        if (activeStep != null) {
            activeStep.onComplete();
            completedSteps++;
            activeStep = null;
        }
    }

    private void runSequence() {
        if (!routineStarted) {
            return;
        }

        if (activeStep == null) {
            startNextStep();
        }

        if (activeStep != null && activeStep.isComplete()) {
            finishActiveStep();
            startNextStep();
        }
    }

    private AprilTagDetection getDetectionForSelectedAlliance() {
        if (aprilTagProcessor == null) {
            return null;
        }

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections == null || detections.isEmpty()) {
            return null;
        }

        Alliance alliance = allianceSelector != null ? allianceSelector.getSelectedAlliance() : DEFAULT_ALLIANCE;
        int targetTag = getTagIdForAlliance(alliance);

        AprilTagDetection best = pickBestDetection(detections, targetTag);
        if (best == null && targetTag != -1) {
            best = pickBestDetection(detections, -1);
        }
        return best;
    }

    private AprilTagDetection pickBestDetection(List<AprilTagDetection> detections, int targetId) {
        AprilTagDetection best = null;
        double bestMargin = Double.NEGATIVE_INFINITY;
        for (AprilTagDetection detection : detections) {
            if (targetId > 0 && detection.id != targetId) {
                continue;
            }
            if (detection.robotPose == null) {
                continue;
            }
            if (detection.decisionMargin > bestMargin) {
                bestMargin = detection.decisionMargin;
                best = detection;
            }
        }
        return best;
    }

    private int getTagIdForAlliance(Alliance alliance) {
        if (alliance == Alliance.BLUE) {
            return BLUE_ALLIANCE_TAG_ID;
        }
        if (alliance == Alliance.RED) {
            return RED_ALLIANCE_TAG_ID;
        }
        return -1;
    }

    private void maybeSeedPoseFromAprilTag() {
        if (follower == null || seededStartPose != null) {
            return;
        }

        AprilTagDetection detection = getDetectionForSelectedAlliance();
        Pose pose = AprilTagPoseUtil.toPedroPose(detection);
        if (pose != null) {
            seededStartPose = pose;
            lastLocalizationPose = pose;
            follower.setStartingPose(pose);
            follower.setPose(pose);
        }
    }

    private void rebuildForAllianceIfNeeded() {
        if (follower == null || routineStarted || allianceSelector == null) {
            return;
        }

        Alliance selectedAlliance = allianceSelector.getSelectedAlliance();
        Pose startOverride = seededStartPose;
        FieldLayout newLayout = FieldLayout.forAlliance(selectedAlliance, startOverride);
        Pose desiredStart = newLayout.startPose;

        boolean allianceChanged = selectedAlliance != lastAppliedAlliance;
        boolean startChanged = lastBuiltStartPose == null || !posesApproximatelyEqual(desiredStart, lastBuiltStartPose);

        if (!allianceChanged && !startChanged && activeLayout != null) {
            return;
        }

        follower.setStartingPose(desiredStart);
        buildPaths(desiredStart, newLayout);
        buildRoutine();

        activeLayout = newLayout;
        lastAppliedAlliance = selectedAlliance;
        lastBuiltStartPose = desiredStart;
    }

    private boolean posesApproximatelyEqual(Pose a, Pose b) {
        if (a == null || b == null) {
            return false;
        }
        double positionTolerance = 0.25; // inches
        double headingTolerance = Math.toRadians(1.0);
        return Math.abs(a.getX() - b.getX()) <= positionTolerance
                && Math.abs(a.getY() - b.getY()) <= positionTolerance
                && Math.abs(AngleUnit.normalizeRadians(a.getHeading() - b.getHeading())) <= headingTolerance;
    }

    private void updateAllianceFromCamera() {
        if (aprilTagProcessor == null || allianceSelector == null) {
            return;
        }
        allianceSelector.updateFromDetections(aprilTagProcessor.getDetections());
    }

    private void pushAllianceTelemetry() {
        if (allianceSelector == null) {
            telemetry.addData("alliance", "selector offline");
            return;
        }

        int detectedTagId = allianceSelector.getDetectedTagId();
        telemetry.addData("detected tag", detectedTagId == -1 ? "none" : detectedTagId);
        telemetry.addData("detected alliance", allianceSelector.getDetectedAlliance().displayName());
        telemetry.addData("manual override", allianceSelector.isManualOverrideActive());
        if (allianceSelector.isManualOverrideActive()) {
            telemetry.addData("manual choice", allianceSelector.getManualAlliance().displayName());
        }
        telemetry.addData("selected alliance", allianceSelector.getSelectedAlliance().displayName());
        telemetry.addData("selection locked", allianceSelector.isSelectionLocked());
        telemetry.addData("range (in)", formatNumber(allianceSelector.getDetectedRange()));
        double yawDegrees = Double.isNaN(allianceSelector.getDetectedYaw())
                ? Double.NaN
                : Math.toDegrees(allianceSelector.getDetectedYaw());
        telemetry.addData("bearing (deg)", formatNumber(yawDegrees));
        if (!allianceSelector.isSelectionLocked()) {
            telemetry.addLine("Init: D-pad Left=Blue  Right=Red  Down=Auto");
        } else {
            telemetry.addLine("Alliance locked for autonomous");
        }
    }

    private void pushAlignmentTelemetry() {
        telemetry.addData("aligning for shot", aligningForShot);
        telemetry.addData("alignment status", alignmentStatus);
        telemetry.addData("alignment tag", lastAlignmentTagId == -1 ? "none" : lastAlignmentTagId);
        telemetry.addData("heading error (deg)", formatNumber(lastHeadingErrorDeg));
        telemetry.addData("alignment timed out", alignmentTimedOut);
    }

    private void pushPoseTelemetry() {
        Pose followerPose = follower.getPose();
        telemetry.addData("follower x", followerPose.getX());
        telemetry.addData("follower y", followerPose.getY());
        telemetry.addData("follower heading", followerPose.getHeading());
        if (lastLocalizationPose != null) {
            telemetry.addData("last tag pose x", lastLocalizationPose.getX());
            telemetry.addData("last tag pose y", lastLocalizationPose.getY());
            telemetry.addData("last tag heading", lastLocalizationPose.getHeading());
        }
    }

    private String formatNumber(double value) {
        if (Double.isNaN(value)) {
            return "--";
        }
        return String.format("%.1f", value);
    }

    private void initVision() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setCameraPose(CAMERA_POSITION, CAMERA_ORIENTATION)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder().addProcessor(aprilTagProcessor);
        WebcamName webcamName = hardwareMap.tryGet(WebcamName.class, "Webcam 1");
        if (webcamName != null) {
            builder.setCamera(webcamName);
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        visionPortal = builder.build();
    }

    @Override
    public void init() {
        BindingManager.reset();
        follower = Constants.createFollower(hardwareMap);

        stepTimer = new Timer();
        shotTimer = new Timer();

        driverPad = new GamepadEx(() -> gamepad1);
        allianceSelector = new AllianceSelector(driverPad, DEFAULT_ALLIANCE, BLUE_ALLIANCE_TAG_ID, RED_ALLIANCE_TAG_ID);

        initVision();
        updateAllianceFromCamera();
        maybeSeedPoseFromAprilTag();
        rebuildForAllianceIfNeeded();
    }

    @Override
    public void init_loop() {
        BindingManager.update();
        updateAllianceFromCamera();
        maybeSeedPoseFromAprilTag();
        rebuildForAllianceIfNeeded();

        pushAllianceTelemetry();
        telemetry.addData("routine ready", activeLayout != null);
        telemetry.addData("active step", activeStepLabel);
        telemetry.addData("steps queued", routineSteps.size());
        telemetry.update();
    }

    @Override
    public void start() {
        if (allianceSelector != null) {
            allianceSelector.lockSelection();
        }
        routineStarted = true;
        startNextStep();
    }

    @Override
    public void loop() {
        BindingManager.update();
        updateAllianceFromCamera();
        maybeSeedPoseFromAprilTag();
        rebuildForAllianceIfNeeded();

        follower.update();
        runSequence();

        pushAllianceTelemetry();
        pushAlignmentTelemetry();
        telemetry.addData("routine started", routineStarted);
        telemetry.addData("routine finished", routineFinished);
        telemetry.addData("active step", activeStepLabel);
        telemetry.addData("next step", routineSteps.isEmpty() ? "None" : routineSteps.peekFirst().name());
        telemetry.addData("steps remaining", routineSteps.size());
        telemetry.addData("steps completed", completedSteps + "/" + totalSteps);
        telemetry.addData("step timer", stepTimer.getElapsedTimeSeconds());
        telemetry.addData("shot timer", shotTimer.getElapsedTimeSeconds());
        telemetry.addData("follower busy", follower.isBusy());
        pushPoseTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        BindingManager.reset();
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}