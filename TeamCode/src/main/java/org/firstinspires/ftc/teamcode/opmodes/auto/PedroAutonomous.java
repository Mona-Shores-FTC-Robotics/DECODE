package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.util.AllianceLight;

import java.util.EnumMap;
import java.util.Map;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
public class PedroAutonomous extends OpMode {

    private static final double FAKE_SHOT_DURATION_SECONDS = 1.0;

    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private Timer stepTimer;
    private ShootingController shootingController;
    private AllianceLight light;


    private Alliance activeAlliance = Alliance.BLUE;
    private RoutineStep routineStep = RoutineStep.NOT_STARTED;

    private PathChain pathToScore;
    private PathChain scoreToPickup;
    private PathChain pickupToStackEnd;
    private PathChain stackToScore;

    @Override
    public void init() {
        panelsTelemetry = PanelsBridge.preparePanels();
        follower = Constants.createFollower(hardwareMap);
        shootingController = new TimedShootingController(FAKE_SHOT_DURATION_SECONDS);
        stepTimer = new Timer();
        light = AllianceLight.onServo(hardwareMap, "indicator");
        applyAlliance(activeAlliance);
    }

    @Override
    public void init_loop() {
        Alliance desired = activeAlliance;
        if (gamepad1.x) {
            desired = Alliance.BLUE;
        } else if (gamepad1.b) {
            desired = Alliance.RED;
        }

        if (desired != activeAlliance) {
            applyAlliance(desired);
        }

        if (panelsTelemetry != null) {
            panelsTelemetry.debug("Select alliance: X=Blue, B=Red");
            panelsTelemetry.debug("Active alliance: " + activeAlliance.displayName());
            panelsTelemetry.update(telemetry);
        }
        telemetry.addLine("Press X for Blue or B for Red alliance");
        telemetry.addData("Active alliance", activeAlliance.displayName());
        telemetry.update();
    }

    @Override
    public void start() {
        transitionTo(RoutineStep.DRIVE_TO_PRELOAD_SCORE);
    }

    @Override
    public void loop() {
        follower.update();
        updateShootingRoutine();
        autonomousStep();

        PanelsBridge.drawFollowerDebug(follower);
        if (panelsTelemetry != null) {
            panelsTelemetry.debug("Routine step", routineStep.getDisplayName());
            panelsTelemetry.debug("Alliance", activeAlliance.displayName());
            panelsTelemetry.debug("Pose X", follower.getPose().getX());
            panelsTelemetry.debug("Pose Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.debug("Step timer", getStepTimerSeconds());
            panelsTelemetry.debug("Waiting for shot", isWaitingForShot());
            panelsTelemetry.debug("Shot timer", getShotTimerSeconds());
            panelsTelemetry.update(telemetry);
        }

        telemetry.addData("routine step", routineStep.getDisplayName());
        telemetry.addData("alliance", activeAlliance.displayName());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("step timer", getStepTimerSeconds());
        telemetry.addData("waiting for shot", isWaitingForShot());
        telemetry.addData("shot timer", getShotTimerSeconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shootingController != null) {
            shootingController.stop();
        }
    }

    private void autonomousStep() {
        switch (routineStep) {
            case DRIVE_TO_PRELOAD_SCORE:
                startPath(pathToScore, RoutineStep.WAIT_FOR_PRELOAD_SCORE_PATH);
                break;
            case WAIT_FOR_PRELOAD_SCORE_PATH:
                if (!follower.isBusy()) {
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
                if (!follower.isBusy()) {
                    transitionTo(RoutineStep.DRIVE_THROUGH_STACK);
                }
                break;
            case DRIVE_THROUGH_STACK:
                startPath(pickupToStackEnd, RoutineStep.WAIT_FOR_STACK_PATH);
                break;
            case WAIT_FOR_STACK_PATH:
                if (!follower.isBusy()) {
                    transitionTo(RoutineStep.DRIVE_BACK_TO_SCORE);
                }
                break;
            case DRIVE_BACK_TO_SCORE:
                startPath(stackToScore, RoutineStep.WAIT_FOR_RETURN_TO_SCORE);
                break;
            case WAIT_FOR_RETURN_TO_SCORE:
                if (!follower.isBusy()) {
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

    private void applyAlliance(Alliance alliance) {
        activeAlliance = alliance;
        FieldLayout layout = FieldLayout.forAlliance(alliance);
        follower.setStartingPose(layout.pose(FieldPoint.START));
        buildPaths(layout);
        light.applyAlliance(alliance);
    }

    private void buildPaths(FieldLayout layout) {
        pathToScore = follower.pathBuilder()
                .addPath(new BezierLine(layout.pose(FieldPoint.START), layout.pose(FieldPoint.LAUNCH_FAR)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(109))
                .build();

        scoreToPickup = follower.pathBuilder()
                .addPath(new BezierLine(layout.pose(FieldPoint.LAUNCH_FAR), layout.pose(FieldPoint.SETUP_PARKING_ARTIFACTS)))
                .setLinearHeadingInterpolation(Math.toRadians(109), Math.toRadians(90))
                .build();

        pickupToStackEnd = follower.pathBuilder()
                .addPath(new BezierLine(layout.pose(FieldPoint.SETUP_PARKING_ARTIFACTS), layout.pose(FieldPoint.PARKING_ARTIFACTS)))
                .setTangentHeadingInterpolation()
                .build();

        stackToScore = follower.pathBuilder()
                .addPath(new BezierLine(layout.pose(FieldPoint.PARKING_ARTIFACTS), layout.pose(FieldPoint.LAUNCH_FAR)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(109))
                .build();
    }

    private void transitionTo(RoutineStep nextStep) {
        routineStep = nextStep;
        if (stepTimer != null) {
            stepTimer.resetTimer();
        }
    }

    private void startPath(PathChain pathChain, RoutineStep waitingStep) {
        follower.followPath(pathChain, true);
        transitionTo(waitingStep);
    }

    private void beginShootingRoutine() {
        if (shootingController != null) {
            shootingController.beginShooting();
        }
    }

    private void updateShootingRoutine() {
        if (shootingController != null) {
            shootingController.update();
        }
    }

    private boolean isWaitingForShot() {
        return shootingController != null && shootingController.isShooting();
    }

    private double getShotTimerSeconds() {
        return shootingController != null ? shootingController.getElapsedTimeSeconds() : 0.0;
    }

    private double getStepTimerSeconds() {
        return stepTimer != null ? stepTimer.getElapsedTimeSeconds() : 0.0;
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

    private interface ShootingController {
        void beginShooting();
        void update();
        boolean isShooting();
        double getElapsedTimeSeconds();
        void stop();
    }

    private static final class TimedShootingController implements ShootingController {
        private final Timer shotTimer = new Timer();
        private final double durationSeconds;
        private boolean shooting;

        TimedShootingController(double durationSeconds) {
            this.durationSeconds = durationSeconds;
        }

        @Override
        public void beginShooting() {
            shooting = true;
            shotTimer.resetTimer();
        }

        @Override
        public void update() {
            if (shooting && shotTimer.getElapsedTimeSeconds() >= durationSeconds) {
                shooting = false;
            }
        }

        @Override
        public boolean isShooting() {
            return shooting;
        }

        @Override
        public double getElapsedTimeSeconds() {
            return shooting ? shotTimer.getElapsedTimeSeconds() : 0.0;
        }

        @Override
        public void stop() {
            shooting = false;
        }
    }

    private enum FieldPoint {
        START,
        LAUNCH_FAR,
        SETUP_PARKING_ARTIFACTS,
        PARKING_ARTIFACTS
    }

    private static class FieldLayout {
        private final EnumMap<FieldPoint, Pose> poses;

        private FieldLayout(EnumMap<FieldPoint, Pose> poses) {
            this.poses = poses;
        }

        Pose pose(FieldPoint point) {
            return poses.get(point);
        }

        static FieldLayout forAlliance(Alliance alliance) {
            EnumMap<FieldPoint, Pose> blue = new EnumMap<>(FieldPoint.class);
            blue.put(FieldPoint.START, new Pose(56.000, 8.0, Math.toRadians(90)));
            blue.put(FieldPoint.LAUNCH_FAR, new Pose(56.279, 19.817, Math.toRadians(109)));
            blue.put(FieldPoint.SETUP_PARKING_ARTIFACTS, new Pose(23.780, 23.780, Math.toRadians(90)));
            blue.put(FieldPoint.PARKING_ARTIFACTS, new Pose(23.516, 39.633, Math.toRadians(90)));

            if (alliance == Alliance.RED) {
                EnumMap<FieldPoint, Pose> red = new EnumMap<>(FieldPoint.class);
                for (Map.Entry<FieldPoint, Pose> entry : blue.entrySet()) {
                    red.put(entry.getKey(), mirrorAcrossField(entry.getValue()));
                }
                return new FieldLayout(red);
            }

            return new FieldLayout(blue);
        }

        private static final double FIELD_WIDTH_IN = 144.0;

        private static Pose mirrorAcrossField(Pose pose) {
            double mirroredX = FIELD_WIDTH_IN - pose.getX();
            double mirroredHeading = AngleUnit.normalizeRadians(Math.PI - pose.getHeading());
            return new Pose(mirroredX, pose.getY(), mirroredHeading);
        }
    }
}
