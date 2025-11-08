package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.RobotMode;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * Copy of {@link ExampleAuto2} that runs through the {@link Robot} container so subsystem hooks
 * (shooter, intake, lighting, etc.) are available while still using the simple queued-path flow.
 */
@Autonomous(name = "Example Auto 2 (Robot)", group = "Examples")
public class ExampleAuto2Robot extends OpMode {

    private static final double FAKE_SHOT_DURATION_SECONDS = 1.5;
    private static final double MIN_IDLE_TIME_FOR_ZERO_LENGTH_PATH = 0.05;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(57, 18, Math.toRadians(109));
    private final Pose pickup1Pose = new Pose(12, 12, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(24, 36, Math.toRadians(90));
    private final Pose pickup3Pose = new Pose(24, 60, Math.toRadians(90));

    private Robot robot;
    private Follower follower;
    private Timer stepTimer;
    private Timer shotTimer;

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

    private interface AutoStep {
        String name();
        void start();
        boolean isComplete();
        void onComplete();
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
            // No-op
        }
    }

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
            if (robot != null && robot.shooter != null) {
                robot.shooter.requestSpinUp();
            }
        }

        @Override
        public boolean isComplete() {
            return shotTimer.getElapsedTimeSeconds() >= durationSeconds;
        }

        @Override
        public void onComplete() {
            finishFakeShotWindow();
            if (robot != null && robot.shooter != null) {
                robot.shooter.abort();
            }
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
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

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
        robot = new Robot(hardwareMap);
        robot.setRobotMode(RobotMode.MATCH);
        robot.initializeForAuto();

        follower = robot.drive.getFollower();

        stepTimer = new Timer();
        shotTimer = new Timer();

        buildPaths();
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        buildRoutine();
    }

    @Override
    public void start() {
        routineStarted = true;
        startNextStep();
    }

    @Override
    public void loop() {
        robot.drive.updateFollower();
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

    @Override
    public void stop() {
        routineStarted = false;
        routineFinished = true;
        if (robot != null) {
            robot.drive.stop();
            robot.vision.stop();
        }
    }
}
