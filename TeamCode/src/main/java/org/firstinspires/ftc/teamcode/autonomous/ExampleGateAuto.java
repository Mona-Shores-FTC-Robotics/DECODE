package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.autonomous.TimelineModel.PickupPlan;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.InitMenu;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalInt;

@Autonomous(name = "ExampleGateAuto", group = "Examples")
public class ExampleGateAuto extends OpMode {

    private enum State {
        AIM,
        SHOOT_PRELOADS,
        PICKUP,
        MOVE_TO_SHOOT,
        AIM_AGAIN,
        SHOOT_CYCLE,
        DONE
    }

    private static final String TAG = "ExampleGateAuto";

    private InitMenu initMenu;
    private AutoConfig.Snapshot config;
    private GatePolicy.Preview preview;

    private Follower follower;
    private Shooter shooter;
    private Gate gate;
    private Intake intake;

    private boolean isBlueAlliance;
    private List<PickupPlan> pickupPlans;
    private Pose midShootPose;
    private Pose scorePose;

    private State state = State.AIM;
    private final ElapsedTime stateTimer = new ElapsedTime();

    private int cycleIndex = 0;
    private int pickupIndex = 0;
    private boolean followerCommandIssued = false;
    private int moveStage = 0;
    private Path scorePath;
    private List<Path> pathsToPickups = new ArrayList<>();
    private Path pathBackToScore;
    private boolean scorePathIssued = false;

    private int ourShotsPlanned = 0;
    private int ourShotsFired = 0;
    private int shotsRemainingInBurst = 0;
    private int combinedShotCount = 0;
    private boolean gateOpened = false;
    private OptionalInt predictedOpenAt = OptionalInt.empty();

    @Override
    public void init() {
        initMenu = new InitMenu();
        shooter = new Shooter(hardwareMap);
        gate = new Gate(hardwareMap);
        intake = new Intake(hardwareMap);
        follower = createFollower();
        config = snapshotConfig();
        preview = GatePolicy.preview(
                config.expectedTotalShots,
                AutoConfig.mustLeaveLast,
                config.partnerPreloads
        );
        Pose startPose = TimelineModel.getStartPose(false);
        if (follower != null) {
            follower.setStartingPose(startPose);
        }
    }

    @Override
    public void init_loop() {
        config = initMenu.update(gamepad1, telemetry);
        preview = initMenu.getPreview();
       isBlueAlliance = config.alliance == AutoConfig.Alliance.BLUE;
       Pose startPose = TimelineModel.getStartPose(isBlueAlliance);
        if (follower != null) {
            follower.setStartingPose(startPose);
        }
    }

    @Override
    public void start() {
        config = snapshotConfig();
        isBlueAlliance = config.alliance == AutoConfig.Alliance.BLUE;
        pickupPlans = TimelineModel.buildDefaultPickups(isBlueAlliance);
        midShootPose = TimelineModel.getMidShootPose(isBlueAlliance);
        scorePose = TimelineModel.getScorePose(isBlueAlliance);
        Pose startPose = TimelineModel.getStartPose(isBlueAlliance);
        if (follower != null) {
            follower.setStartingPose(startPose);
        }
        buildPaths(startPose, scorePose, pickupPlans);
        scorePathIssued = false;

        gate.close();

        shooter.setSpinUp(true);
        shooter.aimToTarget(scorePose.getHeading());

        ourShotsPlanned = config.ourPreloadCount + (config.plannedCycles * 3);
        ourShotsFired = 0;
        combinedShotCount = config.partnerPreloads;
        gateOpened = gate.isOpen();
        cycleIndex = 0;
        pickupIndex = 0;
        followerCommandIssued = false;
        moveStage = pathBackToScore != null ? 0 : 1;
        shotsRemainingInBurst = 0;
        stateTimer.reset();
        changeState(State.AIM);
    }

    @Override
    public void loop() {
        if (follower != null) {
            follower.update();
        }
        shooter.update();
        intake.update();
        gate.update();

        int newShots = shooter.takeCompletedShots();
        if (newShots > 0) {
            ourShotsFired += newShots;
            shotsRemainingInBurst = Math.max(0, shotsRemainingInBurst - newShots);
            combinedShotCount = config.partnerPreloads + ourShotsFired;
        }

        evaluateGatePolicy();

        switch (state) {
            case AIM:
                if (!scorePathIssued && scorePath != null && !followerCommandIssued) {
                    if (follower != null) {
                        follower.followPath(scorePath);
                    }
                    followerCommandIssued = true;
                    scorePathIssued = true;
                }
                if (followerCommandIssued && !isFollowerBusy()) {
                    followerCommandIssued = false;
                }
                if (!isFollowerBusy() && shooter.atSpeed()) {
                    changeState(State.SHOOT_PRELOADS);
                }
                break;
            case SHOOT_PRELOADS:
                if (shotsRemainingInBurst == 0 && !shooter.isBusy()) {
                    if (config.plannedCycles > 0 && ourShotsFired < ourShotsPlanned) {
                        changeState(State.PICKUP);
                    } else {
                        changeState(State.DONE);
                    }
                }
                break;
            case PICKUP:
                handlePickupState();
                break;
            case MOVE_TO_SHOOT:
                handleMoveToShootState();
                break;
            case AIM_AGAIN:
                if (shooter.atSpeed()) {
                    changeState(State.SHOOT_CYCLE);
                }
                break;
            case SHOOT_CYCLE:
                if (shotsRemainingInBurst == 0 && !shooter.isBusy()) {
                    cycleIndex++;
                    if (cycleIndex < config.plannedCycles && ourShotsFired < ourShotsPlanned) {
                        pickupIndex = 0;
                        followerCommandIssued = false;
                        changeState(State.PICKUP);
                    } else {
                        changeState(State.DONE);
                    }
                }
                break;
            case DONE:
                intake.stop();
                shooter.stop();
                break;
        }

        sendRuntimeTelemetry();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
    }

    private void handlePickupState() {
        if (config.plannedCycles <= 0) {
            changeState(State.DONE);
            return;
        }

        if (pathsToPickups.isEmpty()) {
            intake.stop();
            changeState(State.MOVE_TO_SHOOT);
            return;
        }

        if (!followerCommandIssued && pickupIndex < pathsToPickups.size()) {
            if (follower != null) {
                follower.followPath(pathsToPickups.get(pickupIndex));
            }
            followerCommandIssued = true;
            intake.runIn();
        }

        if (followerCommandIssued && !isFollowerBusy()) {
            pickupIndex++;
            followerCommandIssued = false;
            if (pickupIndex >= pickupPlans.size()) {
                intake.stop();
                changeState(State.MOVE_TO_SHOOT);
            }
        }
    }

    private void handleMoveToShootState() {
        if (!followerCommandIssued) {
            boolean commanded = false;
            if (moveStage == 0 && pathBackToScore != null) {
                if (follower != null) {
                    follower.followPath(pathBackToScore);
                }
                commanded = true;
            } else if (moveStage == 0 && pathBackToScore == null) {
                moveStage = 1;
            } else if (moveStage == 1) {
                commanded = followLineTo(midShootPose);
            } else if (moveStage == 2) {
                commanded = followLineTo(scorePose);
            } else {
                moveStage = pathBackToScore != null ? 0 : 1;
                changeState(State.AIM_AGAIN);
                return;
            }
            if (commanded) {
                followerCommandIssued = true;
            }
        }

        if (followerCommandIssued && !isFollowerBusy()) {
            followerCommandIssued = false;
            moveStage++;
            int terminalStage = 3;
            if (moveStage >= terminalStage) {
                moveStage = pathBackToScore != null ? 0 : 1;
                changeState(State.AIM_AGAIN);
            }
        }
    }

    private void changeState(State next) {
        if (state == next) {
            return;
        }
        state = next;
        stateTimer.reset();
        switch (next) {
            case AIM:
                armShooterForScore();
                break;
            case SHOOT_PRELOADS:
                queueShots(config.ourPreloadCount);
                break;
            case PICKUP:
                followerCommandIssued = false;
                pickupIndex = 0;
                intake.start();
                break;
            case MOVE_TO_SHOOT:
                followerCommandIssued = false;
                moveStage = 0;
                break;
            case AIM_AGAIN:
                armShooterForScore();
                break;
            case SHOOT_CYCLE:
                queueShots(3);
                break;
            case DONE:
                intake.stop();
                shooter.stop();
                shotsRemainingInBurst = 0;
                break;
        }
    }

    private void armShooterForScore() {
        shooter.setSpinUp(true);
        if (scorePose != null) {
            shooter.aimToTarget(scorePose.getHeading());
        }
    }

    private void queueShots(int desiredCount) {
        int remainingCapacity = Math.max(0, ourShotsPlanned - ourShotsFired);
        int shotsToQueue = Math.min(desiredCount, remainingCapacity);
        if (shotsToQueue <= 0) {
            shotsRemainingInBurst = 0;
            return;
        }
        shotsRemainingInBurst = shotsToQueue;
        shooter.burst(shotsToQueue);
    }

    private void evaluateGatePolicy() {
        gateOpened = gateOpened || gate.isOpen();
        predictedOpenAt = GatePolicy.computeOpenShotIndex(
                config.expectedTotalShots,
                AutoConfig.mustLeaveLast,
                config.partnerPreloads,
                combinedShotCount,
                gateOpened
        );

        if (gateOpened) {
            return;
        }
        if (!config.weOwnGate) {
            return;
        }

        boolean shouldOpen = false;
        switch (config.gateMode) {
            case AUTO:
                shouldOpen = GatePolicy.shouldOpenNow(
                        combinedShotCount,
                        config.expectedTotalShots,
                        AutoConfig.mustLeaveLast,
                        gateOpened
                );
                break;
            case MANUAL_AFTER_N:
                shouldOpen = combinedShotCount >= config.manualOpenAfterShots;
                break;
            case NEVER:
                shouldOpen = false;
                break;
        }

        if (shouldOpen) {
            gate.openGate();
            gateOpened = true;
        }
    }

    private Follower createFollower() {
        try {
            return Constants.createFollower(hardwareMap);
        } catch (Throwable t) {
            RobotLog.ee(TAG, t, "Unable to create Pedro follower");
            return null;
        }
    }

    private AutoConfig.Snapshot snapshotConfig() {
        return new AutoConfig.Snapshot(
                AutoConfig.alliance,
                AutoConfig.partnerPreloads,
                AutoConfig.weOwnGate,
                AutoConfig.expectedTotalShots,
                AutoConfig.plannedCycles,
                AutoConfig.gateMode,
                AutoConfig.manualOpenAfterShots,
                AutoConfig.ourPreloadCount
        );
    }

    private void buildPaths(Pose startPose, Pose targetScorePose, List<PickupPlan> pickups) {
        scorePathIssued = false;

        if (follower == null) {
            scorePath = null;
            pathsToPickups.clear();
            pathBackToScore = null;
            return;
        }
        pathsToPickups = new ArrayList<>();
        scorePath = new Path(new BezierLine(startPose, targetScorePose));
        scorePath.setLinearHeadingInterpolation(startPose.getHeading(), targetScorePose.getHeading());

        if (!pickups.isEmpty()) {
            Pose first = pickups.get(0).targetPose;
            Pose control = TimelineModel.getLineupControlPose(isBlueAlliance);
            Path toFirst = control != null
                    ? new Path(new BezierCurve(targetScorePose, control, first))
                    : new Path(new BezierLine(targetScorePose, first));
            toFirst.setLinearHeadingInterpolation(targetScorePose.getHeading(), first.getHeading());
            pathsToPickups.add(toFirst);

            for (int i = 1; i < pickups.size(); i++) {
                Pose prev = pickups.get(i - 1).targetPose;
                Pose next = pickups.get(i).targetPose;
                Path segment = new Path(new BezierLine(prev, next));
                segment.setLinearHeadingInterpolation(prev.getHeading(), next.getHeading());
                pathsToPickups.add(segment);
            }

            Pose last = pickups.get(pickups.size() - 1).targetPose;
            pathBackToScore = new Path(new BezierLine(last, targetScorePose));
            pathBackToScore.setLinearHeadingInterpolation(last.getHeading(), targetScorePose.getHeading());
        } else {
            pathBackToScore = null;
        }
    }

    private void sendRuntimeTelemetry() {
        telemetry.addData("State", state);
        telemetry.addData("State time (s)", stateTimer.seconds());
        telemetry.addData("Cycle index", cycleIndex + "/" + config.plannedCycles);
        telemetry.addData("Pickup index", pickupIndex);
        telemetry.addData("Shots fired (ours)", ourShotsFired + "/" + ourShotsPlanned);
        telemetry.addData("Shots queued", shotsRemainingInBurst);
        telemetry.addData("Combined shots", combinedShotCount + "/" + config.expectedTotalShots);
        telemetry.addData("Predicted open at", predictedOpenAt.isPresent()
                ? predictedOpenAt.getAsInt()
                : "No gate open needed");
        telemetry.addData("Gate opened", gateOpened);
        telemetry.addData("Gate busy", gate.isBusy());
        telemetry.addData("Shooter at speed", shooter.atSpeed());
        telemetry.addData("Shooter bursting", shooter.isBursting());
        telemetry.addData("Shooter busy", shooter.isBusy());
        telemetry.addData("Follower busy", isFollowerBusy());
        Pose pose = follower != null ? follower.getPose() : null;
        telemetry.addData("Follower pose", pose != null ? pose : "n/a");
        telemetry.update();
    }
    private boolean followLineTo(Pose targetPose) {
        if (targetPose == null) {
            return true;
        }
        if (follower == null) {
            return true;
        }
        Pose start = follower.getPose();
        if (start == null) {
            start = targetPose;
        }
        Path path = new Path(new BezierLine(start, targetPose));
        path.setLinearHeadingInterpolation(start.getHeading(), targetPose.getHeading());
        follower.followPath(path);
        return true;
    }

    private boolean isFollowerBusy() {
        return follower != null && follower.isBusy();
    }
}
