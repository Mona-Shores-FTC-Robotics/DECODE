package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.auto.Alliance;
import org.firstinspires.ftc.teamcode.auto.AprilTagPoseUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.util.AllianceLight;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.bylazar.configurables.annotations.Configurable;

import java.util.EnumMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
public class PedroAutonomous extends OpMode {

    private static final double FAKE_SHOT_DURATION_SECONDS = 1.0;
    private static final int BLUE_ALLIANCE_TAG_ID = 20;
    private static final int RED_ALLIANCE_TAG_ID = 24;
    private static final double POSE_POSITION_TOLERANCE = 1.0; // inches
    private static final double POSE_HEADING_TOLERANCE = Math.toRadians(10.0);
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
            0.0,   // yaw facing forward
            -90.0, // pitch to horizontal
            0.0,
            0
    );

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
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;


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
    private Alliance lastDetectedAlliance = Alliance.UNKNOWN;
    private boolean manualAllianceOverride;
    private Alliance manualAlliance = Alliance.UNKNOWN;
    private double lastRawFtcX = Double.NaN;
    private double lastRawFtcY = Double.NaN;
    private double lastRawFtcYaw = Double.NaN;
    private double lastRawRobotX = Double.NaN;
    private double lastRawRobotY = Double.NaN;
    private double lastRawRobotYaw = Double.NaN;
    private int lastDetectedTagId = -1;
    private boolean lastInitLoopY = false;
    private boolean lastInitLoopLeftBumper = false;
    private boolean lastInitLoopRightBumper = false;
    private boolean lastInitLoopX = false;
    private boolean lastInitLoopB = false;
    private boolean lastInitLoopA = false;
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
        initVision();
        allianceLocked = false;
        activeAlliance = DEFAULT_ALLIANCE;
        manualAllianceOverride = false;
        manualAlliance = Alliance.UNKNOWN;
        applyAlliance(activeAlliance);
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
        telemetry.addData("Detected alliance", lastDetectedAlliance.displayName());
        telemetry.addData("Detected tag", lastDetectedTagId == -1 ? "none" : lastDetectedTagId);
        telemetry.addData("Detected start", formatPose(lastDetectedStartPose));
        telemetry.addData("Raw ftc XYZ", formatRawFtc());
        telemetry.addData("Raw robot XYZ", formatRawRobot());
        telemetry.update();
    }

    @Override
    public void start() {
        allianceLocked = true;
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
        telemetry.addData("Detected tag", lastDetectedTagId == -1 ? "none" : lastDetectedTagId);
        telemetry.addData("Detected start", formatPose(lastDetectedStartPose));
        telemetry.addData("Raw ftc XYZ", formatRawFtc());
        telemetry.addData("Raw robot XYZ", formatRawRobot());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shootingController != null) {
            shootingController.stop();
        }
        shutdownVision();
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
        if (alliance == activeAlliance && lastAppliedStartPose != null) {
            layout.overrideStart(lastAppliedStartPose);
        } else if (alliance == lastDetectedAlliance && lastDetectedStartPose != null) {
            layout.overrideStart(lastDetectedStartPose);
        }
        PathChain[] chains = createPreviewPathChains(layout);
        PanelsBridge.drawPreview(chains, layout.pose(FieldPoint.START), alliance == Alliance.RED);
    }

    private void setManualAlliance(Alliance alliance) {
        manualAllianceOverride = true;
        manualAlliance = alliance;
        Pose override = null;
        if (lastDetectedAlliance == alliance && lastDetectedStartPose != null) {
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
        if (allianceLocked || aprilTagProcessor == null) {
            return;
        }

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        AprilTagDetection bestDetection = null;
        double bestDecisionMargin = Double.NEGATIVE_INFINITY;
        Alliance bestAlliance = Alliance.UNKNOWN;

        if (detections != null) {
            for (AprilTagDetection detection : detections) {
                Alliance tagAlliance = mapTagToAlliance(detection.id);
                if (tagAlliance == Alliance.UNKNOWN) {
                    continue;
                }
                if (manualAllianceOverride && tagAlliance != manualAlliance) {
                    continue;
                }
                if (detection.decisionMargin > bestDecisionMargin) {
                    bestDecisionMargin = detection.decisionMargin;
                    bestDetection = detection;
                    bestAlliance = tagAlliance;
                }
            }
        }

        if (bestDetection == null) {
            lastDetectedAlliance = Alliance.UNKNOWN;
            lastDetectedStartPose = null;
            lastDetectedTagId = -1;
            lastRawFtcX = Double.NaN;
            lastRawFtcY = Double.NaN;
            lastRawFtcYaw = Double.NaN;
            lastRawRobotX = Double.NaN;
            lastRawRobotY = Double.NaN;
            lastRawRobotYaw = Double.NaN;
            return;
        }

        lastDetectedTagId = bestDetection.id;
        if (bestDetection.ftcPose != null) {
            lastRawFtcX = bestDetection.ftcPose.x;
            lastRawFtcY = bestDetection.ftcPose.y;
            lastRawFtcYaw = AngleUnit.normalizeDegrees(bestDetection.ftcPose.yaw);
        } else {
            lastRawFtcX = Double.NaN;
            lastRawFtcY = Double.NaN;
            lastRawFtcYaw = Double.NaN;
        }
        if (bestDetection.robotPose != null) {
            lastRawRobotX = bestDetection.robotPose.getPosition().x;
            lastRawRobotY = bestDetection.robotPose.getPosition().y;
            lastRawRobotYaw = AngleUnit.normalizeDegrees(
                    bestDetection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)
            );
        } else {
            lastRawRobotX = Double.NaN;
            lastRawRobotY = Double.NaN;
            lastRawRobotYaw = Double.NaN;
        }

        Pose detectedPose = AprilTagPoseUtil.toPedroPose(bestDetection);
        if (detectedPose != null && (Double.isNaN(detectedPose.getX())
                || Double.isNaN(detectedPose.getY())
                || Double.isNaN(detectedPose.getHeading()))) {
            detectedPose = null;
        }

        lastDetectedAlliance = bestAlliance;
        lastDetectedStartPose = detectedPose == null ? null : copyPose(detectedPose);

        if (manualAllianceOverride) {
            if (bestAlliance == manualAlliance && detectedPose != null && shouldUpdateStartPose(detectedPose)) {
                applyAlliance(activeAlliance, detectedPose);
            }
            return;
        }

        if (bestAlliance != activeAlliance) {
            applyAlliance(bestAlliance, detectedPose);
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

    private Alliance mapTagToAlliance(int tagId) {
        if (tagId == BLUE_ALLIANCE_TAG_ID) {
            return Alliance.BLUE;
        }
        if (tagId == RED_ALLIANCE_TAG_ID) {
            return Alliance.RED;
        }
        return Alliance.UNKNOWN;
    }

    private void initVision() {
        if (visionPortal != null) {
            return;
        }
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(CAMERA_POSITION, CAMERA_ORIENTATION)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder().addProcessor(aprilTagProcessor);
        WebcamName webcam = hardwareMap.tryGet(WebcamName.class, "Webcam 1");
        if (webcam != null) {
            builder.setCamera(webcam);
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        visionPortal = builder.build();
    }

    private void shutdownVision() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
        aprilTagProcessor = null;
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
        currentLayout = FieldLayout.forAlliance(alliance);
        if (startOverride != null) {
            currentLayout.overrideStart(startOverride);
        }
        Pose startPose = currentLayout.pose(FieldPoint.START);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        buildPaths(currentLayout);
        cachePathSummaries(currentLayout);
        publishLayoutTelemetry(currentLayout);
        light.applyAlliance(alliance);
        lastAppliedStartPose = copyPose(startPose);
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

    private static Pose copyPose(Pose pose) {
        if (pose == null) {
            return null;
        }
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    private String formatRawFtc() {
        if (Double.isNaN(lastRawFtcX) || Double.isNaN(lastRawFtcY) || Double.isNaN(lastRawFtcYaw)) {
            return "(--, --, --)";
        }
        return String.format(Locale.US, "(%.2f, %.2f, %.1f°)", lastRawFtcX, lastRawFtcY, lastRawFtcYaw);
    }

    private String formatRawRobot() {
        if (Double.isNaN(lastRawRobotX) || Double.isNaN(lastRawRobotY) || Double.isNaN(lastRawRobotYaw)) {
            return "(--, --, --)";
        }
        return String.format(Locale.US, "(%.2f, %.2f, %.1f°)", lastRawRobotX, lastRawRobotY, lastRawRobotYaw);
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

        void overrideStart(Pose startPose) {
            if (startPose == null) {
                return;
            }
            poses.put(FieldPoint.START, new Pose(startPose.getX(), startPose.getY(), startPose.getHeading()));
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
