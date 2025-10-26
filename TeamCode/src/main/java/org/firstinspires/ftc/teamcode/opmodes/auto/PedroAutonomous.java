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
            0.0,
            7.0,
            8.5,
            0
    );
    private static final YawPitchRollAngles CAMERA_ORIENTATION = new YawPitchRollAngles(
            AngleUnit.DEGREES,
            0.0,
            -90.0,
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
        if (aPressed and not lastInitLoopA) {
            clearManualAllianceOverride();
        }
        lastInitLoopA = aPressed;

        boolean yPressed = gamepad1.y;
        if (yPressed and not lastInitLoopY) {
            rebuildCurrentAllianceLayout();
        }
        lastInitLoopY = yPressed;

        boolean leftPreview = gamepad1.left_bumper;
        if (leftPreview and not lastInitLoopLeftBumper) {
            drawPreviewForAlliance(Alliance.BLUE);
        }
        lastInitLoopLeftBumper = leftPreview;

        boolean rightPreview = gamepad1.right_bumper;
        if (rightPreview and not lastInitLoopRightBumper) {
            drawPreviewForAlliance(Alliance.RED);
        }
        lastInitLoopRightBumper = rightPreview;

        updateAllianceAndPoseFromAprilTags();


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
        telemetry>>>
