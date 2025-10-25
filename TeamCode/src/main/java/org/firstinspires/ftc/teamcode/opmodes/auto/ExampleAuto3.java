package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.Alliance;
import org.firstinspires.ftc.teamcode.auto.AllianceSelector;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * Extends ExampleAuto2 by layering in AprilTag awareness and an alliance override.
 * <p>
 * During the init loop the op mode inspects the AprilTag stream to determine which alliance
 * we should run (Blue vs Red). The detected alliance is surfaced on telemetry alongside
 * NextFTC-driven D-pad overrides so a driver can flip between Blue, Red, or "auto" when the
 * tag is missing or the camera fails. Whenever the selected alliance changes we rebuild the
 * path set and routine queue so the scoring cycles always match the choice shown on the Driver
 * Station.
 */
@Autonomous(name = "Example Auto 3", group = "Examples")
public class ExampleAuto3 extends OpMode {

    private static final double FAKE_SHOT_DURATION_SECONDS = 1.5;
    private static final double MIN_IDLE_TIME_FOR_ZERO_LENGTH_PATH = 0.05;

    private static final int BLUE_ALLIANCE_TAG_ID = 20;
    private static final int RED_ALLIANCE_TAG_ID = 24;

    private static final Alliance DEFAULT_ALLIANCE = Alliance.BLUE;

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
    private boolean waitingForShot;
    private boolean routineStarted;
    private boolean routineFinished;
    private String activeStepLabel = "Queued";
    private int totalSteps;
    private int completedSteps;

    private AllianceSelector allianceSelector;
    private Alliance lastAppliedAlliance;
    private GamepadEx driverPad;
    private FieldLayout activeLayout;

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

        static FieldLayout forAlliance(Alliance alliance) {
            Pose blueStart = new Pose(57, 9, Math.toRadians(90));
            Pose blueScore = new Pose(57, 18, Math.toRadians(115));
            Pose bluePickup1 = new Pose(12, 12, Math.toRadians(180));
            Pose bluePickup2 = new Pose(24, 36, Math.toRadians(90));
            Pose bluePickup3 = new Pose(24, 60, Math.toRadians(90));

            if (alliance == Alliance.RED) {
                return new FieldLayout(
                        mirrorPoseAcrossField(blueStart),
                        mirrorPoseAcrossField(blueScore),
                        mirrorPoseAcrossField(bluePickup1),
                        mirrorPoseAcrossField(bluePickup2),
                        mirrorPoseAcrossField(bluePickup3)
                );
            }

            // Use the Blue coordinates for Blue and Unknown (default) cases. Update these with
            // the real field positions when you begin defining alliance-specific splines.
            return new FieldLayout(blueStart, blueScore, bluePickup1, bluePickup2, bluePickup3);
        }

        private static Pose mirrorPoseAcrossField(Pose pose) {
            double mirroredHeading = AngleUnit.normalizeRadians(Math.PI - pose.getHeading());
            return new Pose(-pose.getX(), pose.getY(), mirroredHeading);
        }
    }

    /**
     * Represents the callback that fires when Pedro finishes a path. Once the follower reports it
     * is idle we let the routine advance to the next step (either another path or a pause).
     */
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
                // Short, zero-distance paths may never report busy; guard against that so we do not hang forever.
                // When the Pedro callbacks API is available on the robot we can swap this polling logic
                // for a real pause/resume callback hook.
                return stepTimer.getElapsedTimeSeconds() > MIN_IDLE_TIME_FOR_ZERO_LENGTH_PATH;
            }
            return true;
        }

        @Override
        public void onComplete() {
            // No-op; follow steps do not have extra teardown beyond letting the follower finish.
        }
    }

    /**
     * Simple pause step that emulates the shooter-ready callback described in the Pedro docs. It
     * pauses the follower via {@link Follower#pausePathFollowing()} so subsystems can run without
     * motion, then resumes once the predicate clears. Swap {@link #durationSeconds} with a hardware
     * predicate once the launcher exists.
     */
    private class WaitForShotStep implements AutoStep {
        private final String label;
        private final double durationSeconds;

        WaitForShotStep(String label, double durationSeconds) {
            this.label = label;
            this.durationSeconds = durationSeconds;
        }

        @Override
        public String name() {
            return label;
        }

        @Override
        public void start() {
            beginFakeShotWindow();
            shotTimer.resetTimer();
        }

        @Override
        public boolean isComplete() {
            return shotTimer.getElapsedTimeSeconds() >= durationSeconds;
        }

        @Override
        public void onComplete() {
            finishFakeShotWindow();
        }
    }

    private void beginFakeShotWindow() {
        waitingForShot = true;
        follower.pausePathFollowing();
    }

    private void finishFakeShotWindow() {
        waitingForShot = false;
        follower.resumePathFollowing();
    }

    private void buildPaths(FieldLayout layout) {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(layout.startPose, layout.scorePose))
                .setLinearHeadingInterpolation(layout.startPose.getHeading(), layout.scorePose.getHeading())
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

    private void queueShotPause(String label) {
        routineSteps.addLast(new WaitForShotStep(label, FAKE_SHOT_DURATION_SECONDS));
    }

    private void buildRoutine() {
        routineSteps.clear();
        activeStep = null;
        routineStarted = false;
        routineFinished = false;
        waitingForShot = false;
        completedSteps = 0;
        activeStepLabel = "Queued";
        stepTimer.resetTimer();
        shotTimer.resetTimer();

        queueFollow(scorePreload, "Score preload");
        queueShotPause("Wait to shoot preload");

        queueFollow(grabPickup1, "Drive to pickup 1");
        queueFollow(scorePickup1, "Return to score 1");
        queueShotPause("Wait to shoot pickup 1");

        queueFollow(grabPickup2, "Drive to pickup 2");
        queueFollow(scorePickup2, "Return to score 2");
        queueShotPause("Wait to shoot pickup 2");

        queueFollow(grabPickup3, "Drive to pickup 3");
        queueFollow(scorePickup3, "Return to score 3");
        queueShotPause("Final wait to shoot");

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
        if (!(activeStep instanceof WaitForShotStep)) {
            waitingForShot = false;
        }
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

    private double getShotTimerSeconds() {
        return waitingForShot ? shotTimer.getElapsedTimeSeconds() : 0.0;
    }

    private String getNextStepLabel() {
        return routineSteps.isEmpty() ? "None" : routineSteps.peekFirst().name();
    }

    private void initVision() {
        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder().addProcessor(aprilTagProcessor);
        WebcamName webcamName = hardwareMap.tryGet(WebcamName.class, "Webcam 1");
        if (webcamName != null) {
            builder.setCamera(webcamName);
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        visionPortal = builder.build();
    }

    private void updateAllianceFromCamera() {
        if (routineStarted || aprilTagProcessor == null || allianceSelector == null || allianceSelector.isSelectionLocked()) {
            return;
        }

        allianceSelector.updateFromDetections(aprilTagProcessor.getDetections());
    }

    private void rebuildForAllianceIfNeeded() {
        if (follower == null || routineStarted || allianceSelector == null) {
            return;
        }

        Alliance selectedAlliance = allianceSelector.getSelectedAlliance();

        if (lastAppliedAlliance == selectedAlliance && activeLayout != null) {
            return;
        }

        activeLayout = FieldLayout.forAlliance(selectedAlliance);
        follower.setStartingPose(activeLayout.startPose);
        buildPaths(activeLayout);
        buildRoutine();
        lastAppliedAlliance = selectedAlliance;
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
        telemetry.addData("yaw (deg)", formatNumber(yawDegrees));
        if (!allianceSelector.isSelectionLocked()) {
            telemetry.addLine("Init: D-pad Left=Blue  Right=Red  Down=Auto");
        } else {
            telemetry.addLine("Alliance locked for autonomous");
        }
    }

    private String formatNumber(double value) {
        if (Double.isNaN(value)) {
            return "--";
        }
        return String.format("%.1f", value);
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
        rebuildForAllianceIfNeeded();
    }

    @Override
    public void init_loop() {
        BindingManager.update();
        updateAllianceFromCamera();
        rebuildForAllianceIfNeeded();

        pushAllianceTelemetry();
        telemetry.addData("routine ready", lastAppliedAlliance != null);
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
        rebuildForAllianceIfNeeded();

        follower.update();
        runSequence();

        pushAllianceTelemetry();
        telemetry.addData("routine started", routineStarted);
        telemetry.addData("routine finished", routineFinished);
        telemetry.addData("active step", activeStepLabel);
        telemetry.addData("next step", getNextStepLabel());
        telemetry.addData("steps remaining", routineSteps.size());
        telemetry.addData("steps completed", completedSteps + "/" + totalSteps);
        telemetry.addData("step timer", stepTimer.getElapsedTimeSeconds());
        telemetry.addData("waiting for shot", waitingForShot);
        telemetry.addData("shot timer", getShotTimerSeconds());
        telemetry.addData("follower busy", follower.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
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