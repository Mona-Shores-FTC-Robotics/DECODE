package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * Example autonomous routine that demonstrates how to stage pauses between scoring cycles.
 * The routine is declarative: each path step registers a completion callback that queues the next
 * action, and wait steps represent the shooter window. Replacing the fake wait with a real
 * subsystem check later only requires swapping the wait step implementation.
 */
@Autonomous(name = "Example Auto 2", group = "Examples")
public class ExampleAuto2 extends OpMode {

    private static final double FAKE_SHOT_DURATION_SECONDS = 1.5;
    private static final double MIN_IDLE_TIME_FOR_ZERO_LENGTH_PATH = 0.05;

    private final Pose startPose = new Pose(57, 9, Math.toRadians(90));
    private final Pose scorePose = new Pose(57, 18, Math.toRadians(115));
    private final Pose pickup1Pose = new Pose(12, 12, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(24, 36, Math.toRadians(90));
    private final Pose pickup3Pose = new Pose(24, 60, Math.toRadians(90));

    private Follower follower;
    private Timer stepTimer;
    private Timer shotTimer;

    private Path scorePreload;
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

    private interface AutoStep {
        String name();
        void start();
        boolean isComplete();
        void onComplete();
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

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    private void queueFollow(PathChain chain, String label) {
        routineSteps.addLast(new FollowPathStep(label, () -> follower.followPath(chain)));
    }


    private void queueFollow(PathChain chain, boolean holdEnd, String label) {
        routineSteps.addLast(new FollowPathStep(label, () -> follower.followPath(chain, holdEnd)));
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

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        stepTimer = new Timer();
        shotTimer = new Timer();

        buildPaths();
        follower.setStartingPose(startPose);
        buildRoutine();
    }

    @Override
    public void start() {
        routineStarted = true;
        startNextStep();
    }

    @Override
    public void loop() {
        follower.update();
        runSequence();

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
}