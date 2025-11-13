package org.firstinspires.ftc.teamcode.opmodes;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldLayout;
import org.firstinspires.ftc.teamcode.util.AutoField.FieldPoint;
import org.firstinspires.ftc.teamcode.util.DecodePatternController;
import org.firstinspires.ftc.teamcode.util.RobotMode;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.Arrays;
import java.util.Locale;
import java.util.Optional;

import Ori.Coval.Logging.AutoLogManager;
import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Decode Autonomous Close", group = "Autonomous")
public class DecodeAutonomousClose extends NextFTCOpMode {

    private static final Alliance DEFAULT_ALLIANCE = Alliance.BLUE;
    private static final RobotMode ACTIVE_MODE = RobotMode.MATCH;

    @Configurable
    public static class AutoMotionConfig {
        public static double maxPathPower = .6;
        public static double placeholderShotDelaySec = 1.0;
    }

    private static final double POSE_POSITION_TOLERANCE = 1.0; // inches
    private static final double POSE_HEADING_TOLERANCE = Math.toRadians(10.0);

    private Robot robot;
    private TelemetryManager panelsTelemetry;
    private AllianceSelector allianceSelector;

    private Timer stepTimer;
    private RoutineStep routineStep = RoutineStep.NOT_STARTED;

    private FieldLayout currentLayout;
    private Pose lastAppliedStartPosePedro;
    private Pose lastDetectedStartPosePedro;
    private Pose lastDetectedStartPoseFtc;
    private VisionSubsystemLimelight.TagSnapshot lastTagSnapshot;
    private Alliance activeAlliance = Alliance.BLUE;
    private Follower follower;

    private PathChain launchCloseToGateCloseArtifactPickup;
    private PathChain gateCloseArtifactsPickupToLaunchClose;
    private PathChain launchCloseGateFarArtifactsPickup;
    private PathChain gateFarArtifactsPickupToLaunchClose;
    private PathChain launchCloseToParkingArtifactsPickup;
    private PathChain parkingArtifactsPickupToLaunchClose;

    private final DecodePatternController decodeController = new DecodePatternController();
    private ArtifactColor[] activeDecodePattern = new ArtifactColor[0];
    private boolean opModeStarted;

    @Override
    public void onInit() {
        BindingManager.reset();
        robot = new Robot(hardwareMap);
        robot.setRobotMode(ACTIVE_MODE);
        robot.drive.setRobotCentric(DriveSubsystem.robotCentricConfig);
        robot.telemetry.startSession();
        panelsTelemetry = robot.telemetry.panelsTelemetry();
        stepTimer = new Timer();

        robot.launcherCoordinator.lockIntake();
        robot.launcherCoordinator.setIntakeAutomationEnabled(false);
        robot.initializeForAuto();
        follower = robot.drive.getFollower();

        // Register init-phase controls with NextFTC (selector handles alliance overrides automatically).
        GamepadEx driverPad = new GamepadEx(() -> gamepad1);
        allianceSelector = new AllianceSelector(driverPad, DEFAULT_ALLIANCE);
        driverPad.y().whenBecomesTrue(() -> applyAlliance(activeAlliance, lastAppliedStartPosePedro));
        driverPad.leftBumper().whenBecomesTrue(() -> drawPreviewForAlliance(Alliance.BLUE));
        driverPad.rightBumper().whenBecomesTrue(() -> drawPreviewForAlliance(Alliance.RED));
        driverPad.dpadUp().whenBecomesTrue(() -> applyDecodePattern(decodeController.cycleNext()));
        driverPad.dpadDown().whenBecomesTrue(() -> applyDecodePattern(decodeController.clear()));
        driverPad.a().whenBecomesTrue(this::applyLastDetectedStartPose);

        activeAlliance = allianceSelector.getSelectedAlliance();
        applyAlliance(activeAlliance, null);
        allianceSelector.applySelection(robot, robot.lighting);
        applyDecodePattern(decodeController.current()); // default to alliance colour until a pattern is chosen

        addComponents(
                BulkReadComponent.INSTANCE,
                CommandManager.INSTANCE,
                new SubsystemComponent(robot.drive),
                new SubsystemComponent(robot.launcher),
                new SubsystemComponent(robot.intake),
                new SubsystemComponent(robot.lighting),
                new SubsystemComponent(robot.vision),
                new SubsystemComponent(robot.launcherCoordinator)
        );

    }

    @Override
    public void onWaitForStart() {
        BindingManager.update();

        // Blend camera detections with any driver override and apply the combined alliance immediately.
        Optional<VisionSubsystemLimelight.TagSnapshot> snapshotOpt =
                allianceSelector.updateFromVision(robot.vision);

        allianceSelector.applySelection(robot, robot.lighting);

        lastTagSnapshot = snapshotOpt.orElse(null);
        Pose snapshotPosePedro = lastTagSnapshot == null ? null : lastTagSnapshot.getRobotPose().orElse(null);
        Pose snapshotPoseFtc = lastTagSnapshot == null ? null : lastTagSnapshot.getFtcPose().orElse(null);
        lastDetectedStartPosePedro = snapshotPosePedro == null ? null : copyPedroPose(snapshotPosePedro);
        lastDetectedStartPoseFtc = snapshotPoseFtc == null ? null : copyFtcPose(snapshotPoseFtc);

        Alliance selectedAlliance = allianceSelector.getSelectedAlliance();
        if (selectedAlliance != activeAlliance) {
            activeAlliance = selectedAlliance;
            applyAlliance(activeAlliance, null);
        }

        publishInitTelemetry(selectedAlliance);

        robot.telemetry.updateDriverStation(telemetry);
        robot.telemetry.setRoutineStepTelemetry(routineStep.name(), routineStep.ordinal());
        robot.telemetry.publishLoopTelemetry(
                robot.drive,
                robot.launcher ,
                robot.vision,
                null,
                robot.launcherCoordinator,
                activeAlliance,
                getRuntime(),
                null,
                "AutonomousInit",
                true,
                lastAppliedStartPosePedro
        );
    }

    @Override
    public void onStartButtonPressed() {
        BindingManager.reset();
        opModeStarted = true;
        allianceSelector.lockSelection();
        allianceSelector.applySelection(robot, robot.lighting);

        LightingSubsystem lighting = robot.lighting;
        if (lighting != null) {
            if (activeDecodePattern.length > 0) {
                lighting.showDecodePattern(activeDecodePattern);
            } else {
                lighting.indicateIdle();
            }
        }

        robot.launcherCoordinator.setIntakeAutomationEnabled(true);
        robot.launcherCoordinator.unlockIntake();

        LauncherSubsystem launcher = robot.launcher;
        if (launcher != null) {
            launcher.requestSpinUp();
        }

        transitionTo(RoutineStep.DRIVE_FROM_LAUNCH_CLOSE_TO_GATE_CLOSE_ARTIFACTS);
    }

    @Override
    public void onUpdate() {
        autonomousStep();

        // Periodic logging for KoalaLog (WPILOG files)
        AutoLogManager.periodic();

        robot.telemetry.updateDriverStation(telemetry);
        robot.telemetry.setRoutineStepTelemetry(routineStep.name(), routineStep.ordinal());
        robot.telemetry.publishLoopTelemetry(
                robot.drive,
                robot.launcher ,
                robot.vision,
                null,
                robot.launcherCoordinator,
                activeAlliance,
                getRuntime(),
                null,
                "Autonomous",
                false,
                null
        );
    }

    @Override
    public void onStop() {
        opModeStarted = false;
        allianceSelector.unlockSelection();
        BindingManager.reset();

        LauncherSubsystem launcher = robot.launcher;
        if (launcher != null) {
            launcher.abort();
        }
        Pose finalPose = follower == null ? null : copyPedroPose(follower.getPose());
        if (finalPose != null) {
            RobotState.setHandoffPose(finalPose);
        }
        robot.drive.stop();
        robot.vision.stop();
    }

    private void autonomousStep() {
        switch (routineStep) {
            case DRIVE_FROM_LAUNCH_CLOSE_TO_GATE_CLOSE_ARTIFACTS:
                if (!follower.isBusy()) {
                    startPath(launchCloseToGateCloseArtifactPickup, RoutineStep.DRIVE_FROM_GATE_CLOSE_ARTIFACTS_TO_LAUNCH_CLOSE);
                }
                break;
            case DRIVE_FROM_GATE_CLOSE_ARTIFACTS_TO_LAUNCH_CLOSE:
                if (!follower.isBusy()) {
                    startPath(gateCloseArtifactsPickupToLaunchClose, RoutineStep.DRIVE_FROM_LAUNCH_CLOSE_TO_GATE_FAR_ARTIFACTS);
                }
                break;
            case DRIVE_FROM_LAUNCH_CLOSE_TO_GATE_FAR_ARTIFACTS:
                if (!follower.isBusy()) {
                    startPath(launchCloseGateFarArtifactsPickup, RoutineStep.DRIVE_FROM_GATE_FAR_ARTIFACTS_TO_LAUNCH_CLOSE);
                }
                break;
            case DRIVE_FROM_GATE_FAR_ARTIFACTS_TO_LAUNCH_CLOSE:
                if (!follower.isBusy()) {
                    startPath(gateFarArtifactsPickupToLaunchClose, RoutineStep.DRIVE_FROM_LAUNCH_CLOSE_TO_PARKING_ARTIFACTS);
                }
                break;
            case DRIVE_FROM_LAUNCH_CLOSE_TO_PARKING_ARTIFACTS:
                if (!follower.isBusy()) {
                    startPath(launchCloseToParkingArtifactsPickup, RoutineStep.DRIVE_FROM_PARKING_ARTIFACTS_TO_LAUNCH_CLOSE);
                }
                break;
            case DRIVE_FROM_PARKING_ARTIFACTS_TO_LAUNCH_CLOSE:
                if (!follower.isBusy()) {
                    startPath(gateFarArtifactsPickupToLaunchClose, RoutineStep.FINISHED);
                }
                break;
            case NOT_STARTED:
            case FINISHED:
                break;
        }
    }


    private void drawPreviewForAlliance(Alliance alliance) {
        FieldLayout layout = AutoField.layoutForAlliance(alliance);
        if (alliance == activeAlliance && lastAppliedStartPosePedro != null) {
            layout.overrideStart(lastAppliedStartPosePedro);
        } else if (lastTagSnapshot != null && lastTagSnapshot.getAlliance() == alliance && lastDetectedStartPosePedro != null) {
            layout.overrideStart(lastDetectedStartPosePedro);
        }
        PathChain[] chains = createPreviewPathChains(layout, alliance);
        PanelsBridge.drawPreview(chains, layout.pose(FieldPoint.LAUNCH_CLOSE), alliance == Alliance.RED);
    }

    private boolean shouldUpdateStartPose(Pose candidate) {
        if (candidate == null) {
            return false;
        }
        if (lastAppliedStartPosePedro == null) {
            return true;
        }

        double dx = candidate.getX() - lastAppliedStartPosePedro.getX();
        double dy = candidate.getY() - lastAppliedStartPosePedro.getY();
        double distance = Math.hypot(dx, dy);
        double headingError = AngleUnit.normalizeRadians(candidate.getHeading() - lastAppliedStartPosePedro.getHeading());
        headingError = Math.abs(headingError);

        return distance > POSE_POSITION_TOLERANCE || headingError > POSE_HEADING_TOLERANCE;
    }

    private PathChain[] createPreviewPathChains(FieldLayout layout, Alliance alliance) {
        LayoutPaths previewPaths = buildPathChainsForLayout(layout, alliance);
        if (previewPaths == null) {
            return new PathChain[0];
        }
        return new PathChain[]{
                previewPaths.startFarToLaunchFar,
                previewPaths.launchCloseToGateCloseArtifactsPickup,
                previewPaths.gateCloseArtifactsPickupToLaunchClose,
                previewPaths.launchCloseToGateFarArtifactsPickup,
                previewPaths.gateFarPickupToLaunchClose,
                previewPaths.launchCloseToParkingArtifactsPickup,
                previewPaths.parkingArtifactsPickupToLaunchClose
        };
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

        currentLayout = AutoField.layoutForAlliance(activeAlliance);
        if (startOverride != null) {
            currentLayout.overrideStart(startOverride);
        }
        Pose startPose = currentLayout.pose(FieldPoint.LAUNCH_CLOSE);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        buildPaths(currentLayout);
//        cachePathSummaries(currentLayout);
//        publishLayoutTelemetry(currentLayout);
        lastAppliedStartPosePedro = copyPedroPose(startPose);
        if (lastDetectedStartPosePedro == null) {
            lastDetectedStartPosePedro = copyPedroPose(startPose);
        }
        if (lastDetectedStartPoseFtc == null) {
            lastDetectedStartPoseFtc = toFtcPose(startPose);
        }
    }

    private void applyLastDetectedStartPose() {
        Pose candidate = lastDetectedStartPosePedro;
        if (candidate == null) {
            return;
        }
        if (!shouldUpdateStartPose(candidate)) {
            return;
        }
        applyAlliance(activeAlliance, candidate);
    }

    private void publishInitTelemetry(Alliance selectedAlliance) {
        telemetry.clear();
        Alliance detectedAlliance = allianceSelector.getDetectedAlliance();
        telemetry.addLine(String.format(Locale.US,
                "Alliance sel=%s  active=%s  detected=%s%s",
                selectedAlliance.displayName(),
                activeAlliance.displayName(),
                detectedAlliance.displayName(),
                allianceSelector.isManualOverrideActive() ? " [override]" : ""));

        int tagId = allianceSelector.getDetectedTagId();
        telemetry.addLine(String.format(Locale.US,
                "AprilTag: %s  range=%s in  bearing=%s°",
                tagId == -1 ? "none" : tagId,
                formatDouble(allianceSelector.getDetectedRange()),
                formatDouble(allianceSelector.getDetectedYaw())));

        Pose layoutStartPedro = currentLayout == null ? null : currentLayout.pose(FieldPoint.LAUNCH_CLOSE);
        telemetry.addLine(String.format(Locale.US,
                "Layout start (Pedro): %s",
                formatPosePedro(layoutStartPedro)));

        Pose layoutStartFtc = layoutStartPedro == null ? null : toFtcPose(layoutStartPedro);
        telemetry.addLine(String.format(Locale.US,
                "Layout start (FTC):   %s",
                formatPoseFtc(layoutStartFtc)));

        telemetry.addLine(String.format(Locale.US,
                "AprilTag (FTC):       %s",
                formatPoseFtc(lastDetectedStartPoseFtc)));

        telemetry.addLine(String.format(Locale.US,
                "AprilTag (Pedro):     %s",
                formatPosePedro(lastDetectedStartPosePedro)));

        telemetry.addLine(String.format(Locale.US,
                "Applied start:        %s",
                formatPosePedro(lastAppliedStartPosePedro)));

        telemetry.addLine(String.format(Locale.US,
                "Applied start (FTC):  %s",
                formatPoseFtc(toFtcPose(lastAppliedStartPosePedro))));

        telemetry.addLine(String.format(Locale.US,
                "Decode pattern: %s",
                formatDecodePattern(activeDecodePattern)));

        telemetry.addLine("Controls: D-pad ←/→ override, ↓ vision, LB/RB preview, Y rebuild, A apply tag pose, D-pad ↑ cycle decode, ↓ clear");
        telemetry.update();
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

//    private void cachePathSummaries(FieldLayout layout) {
//        Pose start = layout.pose(FieldPoint.START);
//        Pose launch = layout.pose(FieldPoint.LAUNCH_FAR);
//        Pose setup = layout.pose(FieldPoint.SETUP_PARKING_ARTIFACTS);
//        Pose parking = layout.pose(FieldPoint.PARKING_ARTIFACTS);
//
//        pathToScoreSummary = formatSegment("Start→Launch", start, launch);
//        scoreToPickupSummary = formatSegment("Launch→Setup", launch, setup);
//        pickupToStackEndSummary = formatSegment("Setup→Parking", setup, parking);
//        stackToScoreSummary = formatSegment("Parking→Launch", parking, launch);
//    }

//    private void publishLayoutTelemetry(FieldLayout layout) {
//        if (panelsTelemetry == null) {
//            return;
//        }
//        Pose start = layout.pose(FieldPoint.START);
//        Pose launch = layout.pose(FieldPoint.LAUNCH_FAR);
//        Pose setup = layout.pose(FieldPoint.SETUP_PARKING_ARTIFACTS);
//        Pose parking = layout.pose(FieldPoint.PARKING_ARTIFACTS);
//
//        panelsTelemetry.debug("Start pose", formatPosePedro(start));
//        panelsTelemetry.debug("Launch pose", formatPosePedro(launch));
//        panelsTelemetry.debug("Setup pose", formatPosePedro(setup));
//        panelsTelemetry.debug("Parking pose", formatPosePedro(parking));
//        panelsTelemetry.debug("Path Start→Launch", pathToScoreSummary);
//        panelsTelemetry.debug("Path Launch→Setup", scoreToPickupSummary);
//        panelsTelemetry.debug("Path Setup→Parking", pickupToStackEndSummary);
//        panelsTelemetry.debug("Path Parking→Launch", stackToScoreSummary);
//    }

    private void buildPaths(FieldLayout layout) {
        LayoutPaths paths = buildPathChainsForLayout(layout, activeAlliance);
        if (paths == null) {
            return;
        }
        launchCloseToGateCloseArtifactPickup = paths.launchCloseToGateCloseArtifactsPickup;
        gateCloseArtifactsPickupToLaunchClose = paths.gateCloseArtifactsPickupToLaunchClose;
        launchCloseGateFarArtifactsPickup = paths.launchCloseToGateFarArtifactsPickup;
        gateFarArtifactsPickupToLaunchClose = paths.gateFarPickupToLaunchClose;
        launchCloseToParkingArtifactsPickup = paths.launchCloseToParkingArtifactsPickup;
        gateFarArtifactsPickupToLaunchClose = paths.parkingArtifactsPickupToLaunchClose;
    }

    private LayoutPaths buildPathChainsForLayout(FieldLayout layout, Alliance alliance) {
        if (layout == null || follower == null) {
            return null;
        }
        Pose launchClosePose = layout.pose(FieldPoint.LAUNCH_CLOSE);
        Pose gateCloseArtifactsPickupPose = layout.pose(FieldPoint.GATE_CLOSE_ARTIFACTS_PICKUP);
        Pose gateFar270DegArtifactsPickupPose = layout.pose(FieldPoint.GATE_FAR_ARTIFACTS_PICKUP_270_DEG);
        Pose parking270DegArtifactsPickupPose = layout.pose(FieldPoint.PARKING_ARTIFACTS_PICKUP_270_DEG);


        LayoutPaths paths = new LayoutPaths();
        paths.launchCloseToGateCloseArtifactsPickup = follower.pathBuilder()
                .addPath(new BezierLine(launchClosePose, gateCloseArtifactsPickupPose))
                .setLinearHeadingInterpolation(launchClosePose.getHeading(), gateCloseArtifactsPickupPose.getHeading(), .7)
                .build();

        paths.gateCloseArtifactsPickupToLaunchClose = follower.pathBuilder()
                .addPath(new BezierLine(gateCloseArtifactsPickupPose, launchClosePose))
                .setLinearHeadingInterpolation(gateCloseArtifactsPickupPose.getHeading(), launchClosePose.getHeading(),.7)
                .build();

        paths.launchCloseToGateFarArtifactsPickup = follower.pathBuilder()
                .addPath(new BezierCurve(launchClosePose,  gateFar270DegArtifactsPickupPose))
                .setLinearHeadingInterpolation(launchClosePose.getHeading(), gateFar270DegArtifactsPickupPose.getHeading(), .7)
                .build();

        paths.gateFarPickupToLaunchClose = follower.pathBuilder()
                .addPath(new BezierLine(gateFar270DegArtifactsPickupPose, launchClosePose))
                .setLinearHeadingInterpolation(gateFar270DegArtifactsPickupPose.getHeading(), launchClosePose.getHeading(), .7)
                .build();

        paths.launchCloseToParkingArtifactsPickup = follower.pathBuilder()
                .addPath(new BezierCurve(launchClosePose, parking270DegArtifactsPickupPose))
                .setLinearHeadingInterpolation(launchClosePose.getHeading(), parking270DegArtifactsPickupPose.getHeading(), .7)
                .build();

        paths.parkingArtifactsPickupToLaunchClose = follower.pathBuilder()
                .addPath(new BezierLine(parking270DegArtifactsPickupPose, launchClosePose))
                .setLinearHeadingInterpolation(parking270DegArtifactsPickupPose.getHeading(), launchClosePose.getHeading(), .7)
                .build();

        return paths;
    }

    private static final class LayoutPaths {
        PathChain startFarToLaunchFar;
        PathChain launchCloseToGateCloseArtifactsPickup;
        PathChain gateCloseArtifactsPickupToLaunchClose;
        PathChain launchCloseToGateFarArtifactsPickup;
        PathChain gateFarPickupToLaunchClose;
        PathChain launchCloseToParkingArtifactsPickup;
        PathChain parkingArtifactsPickupToLaunchClose;
    }

    private void transitionTo(RoutineStep nextStep) {
        routineStep = nextStep;
        if (stepTimer != null) {
            stepTimer.resetTimer();
        }
    }

    private void startPath(PathChain pathChain, RoutineStep waitingStep) {
        double maxPower = Range.clip(AutoMotionConfig.maxPathPower, 0.0, 1.0);
        follower.followPath(pathChain, maxPower, false);
        transitionTo(waitingStep);
    }

    private void beginLaunchingRoutine() {
        // Placeholder: pause for a configurable duration instead of firing the launcher.
        if (stepTimer != null) {
            stepTimer.resetTimer();
        }
    }

    private boolean isWaitingForShot() {
        // Treat the shot as complete once the delay elapses.
        return stepTimer != null && stepTimer.getElapsedTimeSeconds() < AutoMotionConfig.placeholderShotDelaySec;
    }

    private double getShotTimerSeconds() {
        LauncherSubsystem launcher = robot.launcher;
        return launcher != null ? launcher.getStateElapsedSeconds() : 0.0;
    }

    private double getStepTimerSeconds() {
        return stepTimer != null ? stepTimer.getElapsedTimeSeconds() : 0.0;
    }

    private String getFlywheelStateName() {
        LauncherSubsystem launcher = robot.launcher;
        return launcher != null ? launcher.getState().name() : "N/A";
    }

    private static String formatSegment(String label, Pose start, Pose end) {
        return label + ": " + formatPosePedro(start) + " → " + formatPosePedro(end);
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

    private static String formatPosePedro(Pose pose) {
        if (pose == null) {
            return "(null)";
        }
        return String.format(Locale.US, "(%.3f, %.3f, %.1f°)",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }

    private static String formatPoseFtc(Pose pose) {
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

    private static Pose copyPedroPose(Pose pose) {
        if (pose == null) {
            return null;
        }
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    private static Pose copyFtcPose(Pose pose) {
        if (pose == null) {
            return null;
        }
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    public static Pose toFtcPose(Pose pedroPose) {
        if (pedroPose == null) {
            return null;
        }
        double halfField = AutoField.Waypoints.fieldWidthIn / 2.0;
        double ftcX = halfField - pedroPose.getY();
        double ftcY = pedroPose.getX() - halfField;
        double heading = AngleUnit.normalizeRadians(pedroPose.getHeading() + Math.PI / 2.0);
        return new Pose(ftcX, ftcY, heading);
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

//    private void pushFieldPointPanels() {
//        if (panelsTelemetry == null || currentLayout == null) {
//            return;
//        }
//        Pose start = currentLayout.pose(FieldPoint.START);
//        Pose launch = currentLayout.pose(FieldPoint.LAUNCH_FAR);
//        Pose setup = currentLayout.pose(FieldPoint.SETUP_PARKING_ARTIFACTS);
//        Pose parking = currentLayout.pose(FieldPoint.PARKING_ARTIFACTS);
//
//        panelsTelemetry.debug("Start pose", formatPosePedro(start));
//        panelsTelemetry.debug("Launch pose", formatPosePedro(launch));
//        panelsTelemetry.debug("Setup pose", formatPosePedro(setup));
//        panelsTelemetry.debug("Parking pose", formatPosePedro(parking));
//        panelsTelemetry.debug("Path Start→Launch", pathToScoreSummary);
//        panelsTelemetry.debug("Path Launch→Setup", scoreToPickupSummary);
//        panelsTelemetry.debug("Path Setup→Parking", pickupToStackEndSummary);
//        panelsTelemetry.debug("Path Parking→Launch", stackToScoreSummary);
//    }

    private enum RoutineStep {
        NOT_STARTED("Not started"),
        DRIVE_FROM_LAUNCH_CLOSE_TO_GATE_CLOSE_ARTIFACTS("Drive to alliance wall artifacts"),
        DRIVE_FROM_GATE_CLOSE_ARTIFACTS_TO_LAUNCH_CLOSE("Drive to far score"),
        DRIVE_FROM_LAUNCH_CLOSE_TO_GATE_FAR_ARTIFACTS("Drive to park artifacts"),
        DRIVE_FROM_GATE_FAR_ARTIFACTS_TO_LAUNCH_CLOSE("Drive to far score"),
        DRIVE_FROM_LAUNCH_CLOSE_TO_PARKING_ARTIFACTS("Drive to far gate artifacts"),
        DRIVE_FROM_PARKING_ARTIFACTS_TO_LAUNCH_CLOSE("Drive to far score"),
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
