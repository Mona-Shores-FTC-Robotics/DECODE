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
import com.bylazar.configurables.annotations.Configurable;

import java.util.EnumMap;
import java.util.Locale;
import java.util.Map;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
public class PedroAutonomous extends OpMode {

    private static final double FAKE_SHOT_DURATION_SECONDS = 1.0;

    @Configurable
    public static class Waypoints {
        public static double fieldWidthIn = 144.0;

        public static double startX = 56.000;
        public static double startY = 8.0;
        public static double startHeadingDeg = 90.0;

        public static double launchFarX = 56.279;
        public static double launchFarY = 19.817;
        public static double launchFarHeadingDeg = 109.0;

        public static double setupParkingX = 23.780;
        public static double setupParkingY = 23.780;
        public static double setupParkingHeadingDeg = 90.0;

        public static double parkingArtifactsX = 23.516;
        public static double parkingArtifactsY = 39.633;
        public static double parkingArtifactsHeadingDeg = 90.0;
    }

    
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
    private FieldLayout currentLayout;
        private boolean lastInitLoopY = false;
    private boolean lastInitLoopLeftBumper = false;
    private boolean lastInitLoopRightBumper = false;
    private String pathToScoreSummary;
    private String scoreToPickupSummary;
    private String pickupToStackEndSummary;
    private String stackToScoreSummary;

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

        boolean yPressed = gamepad1.y;
        if (yPressed && !lastInitLoopY) {
            applyAlliance(activeAlliance);
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

//        if (panelsTelemetry != null) {
//            pushFieldPointPanels();
//            panelsTelemetry.debug("Select alliance: X=Blue, B=Red");
//            panelsTelemetry.debug("Active alliance: " + activeAlliance.displayName());
//            panelsTelemetry.debug("Press Y to rebuild paths after tweaking points");
//            panelsTelemetry.debug("LB: preview blue, RB: preview red");
//            panelsTelemetry.update(telemetry);
//        }
        telemetry.addLine("Press X for Blue or B for Red alliance");
        telemetry.addData("Active alliance", activeAlliance.displayName());
        telemetry.addLine("Press Y to rebuild paths after tweaking waypoints");
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
            pushFieldPointPanels();
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


    private void drawPreviewForAlliance(Alliance alliance) {
        FieldLayout layout = FieldLayout.forAlliance(alliance);
        PathChain[] chains = createPreviewPathChains(layout);
        PanelsBridge.drawPreview(chains, layout.pose(FieldPoint.START), alliance == Alliance.RED);
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
        activeAlliance = alliance;
        currentLayout = FieldLayout.forAlliance(alliance);
        Pose startPose = currentLayout.pose(FieldPoint.START);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        buildPaths(currentLayout);
        cachePathSummaries(currentLayout);
        publishLayoutTelemetry(currentLayout);
        light.applyAlliance(alliance);
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

    private static String formatSegment(String label, Pose start, Pose end) {
        return label + ": " + formatPose(start) + " → " + formatPose(end);
    }

    private static String formatPose(Pose pose) {
        if (pose == null) {
            return "(null)";
        }
        return String.format(Locale.US, "(%.3f, %.3f, %.1f°)",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
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
            Pose stored = poses.get(point);
            return stored == null ? null : new Pose(stored.getX(), stored.getY(), stored.getHeading());
        }

        static FieldLayout forAlliance(Alliance alliance) {
            EnumMap<FieldPoint, Pose> layout = new EnumMap<>(FieldPoint.class);
            layout.put(FieldPoint.START, poseForAlliance(
                    Waypoints.startX,
                    Waypoints.startY,
                    Waypoints.startHeadingDeg,
                    alliance
            ));
            layout.put(FieldPoint.LAUNCH_FAR, poseForAlliance(
                    Waypoints.launchFarX,
                    Waypoints.launchFarY,
                    Waypoints.launchFarHeadingDeg,
                    alliance
            ));
            layout.put(FieldPoint.SETUP_PARKING_ARTIFACTS, poseForAlliance(
                    Waypoints.setupParkingX,
                    Waypoints.setupParkingY,
                    Waypoints.setupParkingHeadingDeg,
                    alliance
            ));
            layout.put(FieldPoint.PARKING_ARTIFACTS, poseForAlliance(
                    Waypoints.parkingArtifactsX,
                    Waypoints.parkingArtifactsY,
                    Waypoints.parkingArtifactsHeadingDeg,
                    alliance
            ));
            return new FieldLayout(layout);
        }

        private static Pose poseForAlliance(double x, double y, double headingDeg, Alliance alliance) {
            Pose base = new Pose(x, y, Math.toRadians(headingDeg));
            if (alliance == Alliance.RED) {
                return mirrorAcrossField(base);
            }
            return base;
        }

        private static Pose mirrorAcrossField(Pose pose) {
            double mirroredX = Waypoints.fieldWidthIn - pose.getX();
            double mirroredHeading = AngleUnit.normalizeRadians(Math.PI - pose.getHeading());
            return new Pose(mirroredX, pose.getY(), mirroredHeading);
        }
    }
}
