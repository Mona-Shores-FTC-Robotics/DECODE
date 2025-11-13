package org.firstinspires.ftc.teamcode.telemetry;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import android.os.SystemClock;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.PoseTransforms;

/**
 * Centralises telemetry output to FTControl Panels and the driver station.
 * OpModes create a single instance and share its {@link TelemetryPublisher} across subsystems.
 */
public class TelemetryService {

    private final boolean enableDashboardTelemetry;
    private final TelemetryPublisher publisher;
    private final FtcDashboard dashboard;
    private volatile String routineStepName = "";
    private volatile double routineStepOrdinal = Double.NaN;

    private TelemetryManager panelsTelemetry;
    private boolean sessionActive = false;
    private long lastPanelsDrawMs = 0L;
    private long lastDashboardPacketMs = 0L;

    private static final boolean ENABLE_PANELS_DRAWING = false;
    private static final long PANELS_DRAW_INTERVAL_MS = 50L;
    private static final long DASHBOARD_PACKET_INTERVAL_MS = 50L;

    public TelemetryService() {
        this.enableDashboardTelemetry = TelemetrySettings.enableDashboardTelemetry;
        this.publisher = new TelemetryPublisher(null);
        this.dashboard = enableDashboardTelemetry ? safelyGetDashboard() : null;
    }

    /**
     * Prepares Panels for telemetry. Call once when the OpMode starts.
     */
    public void startSession() {
        if (!sessionActive) {
            panelsTelemetry = PanelsBridge.preparePanels();
            publisher.setTelemetryManager(panelsTelemetry);
            sessionActive = true;
        }
    }

    /**
     * Stops the telemetry session and flushes resources.
     */
    public void stopSession() {
        if (sessionActive) {
            publisher.setTelemetryManager(null);
            panelsTelemetry = null;
            sessionActive = false;
        }
    }

    public TelemetryPublisher publisher() {
        return publisher;
    }

    public TelemetryManager panelsTelemetry() {
        return panelsTelemetry;
    }

    public void setRoutineStepTelemetry(String stepName, double stepOrdinal) {
        routineStepName = stepName == null ? "" : stepName;
        routineStepOrdinal = stepOrdinal;
    }

    public void updateDriverStation(Telemetry telemetry) {
        // Driver station output is handled directly by the caller.
    }


    public void publishLoopTelemetry(DriveSubsystem drive,
                                     LauncherSubsystem launcher,
                                     VisionSubsystemLimelight vision,
                                     DriverBindings.DriveRequest driveRequest,
                                     LauncherCoordinator launcherCoordinator,
                                     Alliance alliance,
                                     double runtimeSec,
                                     Telemetry dsTelemetry,
                                     RobotLogger logger,
                                     String modeLabel,
                                     boolean suppressDriveTelemetry,
                                     Pose poseOverride) {
        double requestX = driveRequest != null ? driveRequest.fieldX : 0.0;
        double requestY = driveRequest != null ? driveRequest.fieldY : 0.0;
        double requestRot = driveRequest != null ? driveRequest.rotation : 0.0;
        boolean slowMode = driveRequest != null && driveRequest.slowMode;
        boolean aimMode = driveRequest != null && driveRequest.aimMode;
        Pose pedroPose = poseOverride != null ? poseOverride : drive.getFollowerPose();
        Pose2D pose = drive.getPose();
        if (poseOverride != null) {
            pose = new Pose2D(
                    DistanceUnit.INCH,
                    poseOverride.getX(),
                    poseOverride.getY(),
                    AngleUnit.RADIANS,
                    poseOverride.getHeading()
            );
        }
        Pose ftcPose = PoseTransforms.toFtcPose(pedroPose);
        double ftcXIn = ftcPose != null ? ftcPose.getX() : Double.NaN;
        double ftcYIn = ftcPose != null ? ftcPose.getY() : Double.NaN;
        double ftcHeadingRad = ftcPose != null ? ftcPose.getHeading() : Double.NaN;
        double poseXIn = pose != null ? pose.getX(DistanceUnit.INCH) : 0.0;
        double poseYIn = pose != null ? pose.getY(DistanceUnit.INCH) : 0.0;
        double headingDeg = pose != null ? pose.getHeading(AngleUnit.DEGREES) : 0.0;
        double headingRad = pose != null ? pose.getHeading(AngleUnit.RADIANS) : 0.0;

        VisionSnapshot visionSnapshot = captureVisionSnapshot(vision, alliance);

        long nowMs = SystemClock.uptimeMillis();
        boolean drawPanelsThisLoop = nowMs - lastPanelsDrawMs >= PANELS_DRAW_INTERVAL_MS;
        boolean sendDashboardThisLoop = nowMs - lastDashboardPacketMs >= DASHBOARD_PACKET_INTERVAL_MS;

        if (drawPanelsThisLoop && ENABLE_PANELS_DRAWING) {
            PanelsBridge.drawFollowerDebug(follower());
            lastPanelsDrawMs = nowMs;
        }

        TelemetryManager panels = panelsTelemetry();
        if (!suppressDriveTelemetry && panels != null) {
            String label = modeLabel == null || modeLabel.isEmpty() ? "robot" : modeLabel.toLowerCase();
            panels.debug("mode/label", label);
            panels.debug("mode/drive", drive.getDriveMode());
            panels.debug("mode/aim/enabled", aimMode);
            panels.debug("vision/tag/visible", visionSnapshot.hasTag);
            panels.debug("vision/tag/id", visionSnapshot.tagId);
            panels.debug("vision/tag/rangeIn", visionSnapshot.rangeIn);
            panels.debug("vision/tag/bearingDeg", visionSnapshot.bearingDeg);
            panels.debug("vision/tag/yawDeg", visionSnapshot.yawDeg);
            panels.debug("vision/odometryPending", visionSnapshot.odometryPending);
        }

        double leftTargetRpm = launcher.getTargetRpm(LauncherLane.LEFT);
        double centerTargetRpm = launcher.getTargetRpm(LauncherLane.CENTER);
        double rightTargetRpm = launcher.getTargetRpm(LauncherLane.RIGHT);
        double leftCurrentRpm = launcher.getCurrentRpm(LauncherLane.LEFT);
        double centerCurrentRpm = launcher.getCurrentRpm(LauncherLane.CENTER);
        double rightCurrentRpm = launcher.getCurrentRpm(LauncherLane.RIGHT);
        double leftPower = launcher.getLastPower(LauncherLane.LEFT);
        double centerPower = launcher.getLastPower(LauncherLane.CENTER);
        double rightPower = launcher.getLastPower(LauncherLane.RIGHT);
        boolean leftReady = launcher.isLaneReady(LauncherLane.LEFT);
        boolean centerReady = launcher.isLaneReady(LauncherLane.CENTER);
        boolean rightReady = launcher.isLaneReady(LauncherLane.RIGHT);
        String controlMode = LauncherSubsystem.getFlywheelControlMode().name();
        String leftPhase = launcher.getPhaseName(LauncherLane.LEFT);
        String centerPhase = launcher.getPhaseName(LauncherLane.CENTER);
        String rightPhase = launcher.getPhaseName(LauncherLane.RIGHT);
        boolean leftBang = "BANG".equals(leftPhase);
        boolean leftHold = "HOLD".equals(leftPhase);
        boolean leftHybrid = "HYBRID".equals(leftPhase);
        boolean centerBang = "BANG".equals(centerPhase);
        boolean centerHold = "HOLD".equals(centerPhase);
        boolean centerHybrid = "HYBRID".equals(centerPhase);
        boolean rightBang = "BANG".equals(rightPhase);
        boolean rightHold = "HOLD".equals(rightPhase);
        boolean rightHybrid = "HYBRID".equals(rightPhase);
        int leftBangToHoldCount = launcher.getBangToHoldCount(LauncherLane.LEFT);
        int centerBangToHoldCount = launcher.getBangToHoldCount(LauncherLane.CENTER);
        int rightBangToHoldCount = launcher.getBangToHoldCount(LauncherLane.RIGHT);
        double displayTargetRpm = Math.max(Math.max(leftTargetRpm, centerTargetRpm), rightTargetRpm);
        double displayCurrentRpm = Math.max(Math.max(leftCurrentRpm, centerCurrentRpm), rightCurrentRpm);
        double displayPower = Math.max(Math.max(leftPower, centerPower), rightPower);

        boolean launcherReady;
        if (launcherCoordinator != null) {
            launcherReady = launcherCoordinator.logLauncherReadyEvent(logger);
        } else {
            launcherReady = launcher.atTarget();
        }
        String launcherState = launcher.getState().name();
        String launcherSpinMode = launcher.getEffectiveSpinMode().name();

        if (!suppressDriveTelemetry) {
            publisher.publishDrive(drive, requestX, requestY, requestRot, slowMode, aimMode);
        }
        publisher.publishLauncher(
                displayTargetRpm,
                displayCurrentRpm,
                displayPower,
                displayTargetRpm - displayCurrentRpm
        );

        if (panels != null) {
            panels.debug("launcher/control/mode", controlMode);
            panels.debug("launcher/lanes/left/targetRpm", leftTargetRpm);
            panels.debug("launcher/lanes/left/currentRpm", leftCurrentRpm);
            panels.debug("launcher/lanes/left/power", leftPower);
            panels.debug("launcher/lanes/left/ready", leftReady);
            panels.debug("launcher/lanes/left/phase", leftPhase);
            panels.debug("launcher/lanes/center/targetRpm", centerTargetRpm);
            panels.debug("launcher/lanes/center/currentRpm", centerCurrentRpm);
            panels.debug("launcher/lanes/center/power", centerPower);
            panels.debug("launcher/lanes/center/ready", centerReady);
            panels.debug("launcher/lanes/center/phase", centerPhase);
            panels.debug("launcher/lanes/right/targetRpm", rightTargetRpm);
            panels.debug("launcher/lanes/right/currentRpm", rightCurrentRpm);
            panels.debug("launcher/lanes/right/power", rightPower);
            panels.debug("launcher/lanes/right/ready", rightReady);
            panels.debug("launcher/lanes/right/phase", rightPhase);
            panels.debug("launcher/lanes/left/bangToHoldCount", leftBangToHoldCount);
            panels.debug("launcher/lanes/center/bangToHoldCount", centerBangToHoldCount);
            panels.debug("launcher/lanes/right/bangToHoldCount", rightBangToHoldCount);
        }

        if (logger != null) {
            logger.logString("Launcher", "ControlMode", controlMode);
            logger.logString("Launcher", "State", launcher.getState().name());
            logger.logString("Launcher", "SpinMode", launcher.getEffectiveSpinMode().name());

            logger.logNumber("Launcher", "LeftTargetRpm", leftTargetRpm);
            logger.logNumber("Launcher", "LeftCurrentRpm", leftCurrentRpm);
            logger.logNumber("Launcher", "LeftPower", leftPower);
            logger.logBoolean("Launcher", "LeftReady", leftReady);
            logger.logString("Launcher", "LeftPhase", leftPhase);

            logger.logNumber("Launcher", "CenterTargetRpm", centerTargetRpm);
            logger.logNumber("Launcher", "CenterCurrentRpm", centerCurrentRpm);
            logger.logNumber("Launcher", "CenterPower", centerPower);
            logger.logBoolean("Launcher", "CenterReady", centerReady);
            logger.logString("Launcher", "CenterPhase", centerPhase);

            logger.logNumber("Launcher", "RightTargetRpm", rightTargetRpm);
            logger.logNumber("Launcher", "RightCurrentRpm", rightCurrentRpm);
            logger.logNumber("Launcher", "RightPower", rightPower);
            logger.logBoolean("Launcher", "RightReady", rightReady);
            logger.logString("Launcher", "RightPhase", rightPhase);
            logger.logNumber("Launcher", "LeftBangToHoldCount", leftBangToHoldCount);
            logger.logNumber("Launcher", "CenterBangToHoldCount", centerBangToHoldCount);
            logger.logNumber("Launcher", "RightBangToHoldCount", rightBangToHoldCount);
        }

        if (logger != null) {
            logger.logNumber("Robot", "RuntimeSec", runtimeSec);
        }

        if (dsTelemetry != null && !suppressDriveTelemetry) {
            if (pose != null) {
                dsTelemetry.addData("Pose", "x=%.1f in  y=%.1f in  h=%.1f°",
                        poseXIn,
                        poseYIn,
                        headingDeg);
            } else {
                dsTelemetry.addData("Pose", "(unavailable)");
            }

            Alliance activeAlliance = alliance == null ? Alliance.UNKNOWN : alliance;
            dsTelemetry.addData("Alliance", activeAlliance.displayName());
            dsTelemetry.addData("Drive Aim Assist", aimMode ? "vision-locked" : "manual");
            dsTelemetry.addData("Drive Turn Cmd",
                    "%.2f (lock %.2f)",
                    drive.getLastCommandTurn());
            dsTelemetry.addData(
                    "launcher Mode",
                    "%s | ready=%s",
                    controlMode,
                    launcherReady
            );
            dsTelemetry.addData(
                    "launcher Left",
                    "T=%.0f  C=%.0f  P=%.2f  %s  %s",
                    leftTargetRpm,
                    leftCurrentRpm,
                    leftPower,
                    leftReady ? "READY" : "----",
                    leftPhase
            );
            dsTelemetry.addData(
                    "launcher Center",
                    "T=%.0f  C=%.0f  P=%.2f  %s  %s",
                    centerTargetRpm,
                    centerCurrentRpm,
                    centerPower,
                    centerReady ? "READY" : "----",
                    centerPhase
            );
            dsTelemetry.addData(
                    "launcher Right",
                    "T=%.0f  C=%.0f  P=%.2f  %s  %s",
                    rightTargetRpm,
                    rightCurrentRpm,
                    rightPower,
                    rightReady ? "READY" : "----",
                    rightPhase
            );
        }

        if (launcherCoordinator != null) {
            launcherCoordinator.publishLaneTelemetry(dsTelemetry, panels);
        }

        if (dsTelemetry != null) {
            dsTelemetry.update();
            updateDriverStation(dsTelemetry);
        }

        if (sendDashboardThisLoop && enableDashboardTelemetry && dashboard != null) {
            sendDashboardPacket(
                    drive,
                    requestX,
                    requestY,
                    requestRot,
                    slowMode,
                    aimMode,
                    pose != null,
                    poseXIn,
                    poseYIn,
                    headingRad,
                    headingDeg,
                    ftcXIn,
                    ftcYIn,
                    ftcHeadingRad,
                    launcherReady,
                    launcherState,
                    launcherSpinMode,
                    launcherCoordinator,
                    alliance,
                    visionSnapshot.alliance,
                    runtimeSec,
                    visionSnapshot.hasTag,
                    visionSnapshot.tagId,
                    visionSnapshot.rangeIn,
                    visionSnapshot.bearingDeg,
                    visionSnapshot.yawDeg,
                    visionSnapshot.poseXIn,
                    visionSnapshot.poseYIn,
                    visionSnapshot.headingRad,
                    visionSnapshot.txDeg,
                    visionSnapshot.tyDeg,
                    visionSnapshot.taPercent,
                    visionSnapshot.odometryPending,
                    controlMode,
                    leftTargetRpm,
                    centerTargetRpm,
                    rightTargetRpm,
                    leftCurrentRpm,
                    centerCurrentRpm,
                    rightCurrentRpm,
                    leftPower,
                    centerPower,
                    rightPower,
                    leftReady,
                    centerReady,
                    rightReady,
                    leftPhase,
                    centerPhase,
                    rightPhase,
                    leftBang,
                    leftHold,
                    leftHybrid,
                    centerBang,
                    centerHold,
                    centerHybrid,
                    rightBang,
                    rightHold,
                    rightHybrid,
                    leftBangToHoldCount,
                    centerBangToHoldCount,
                    rightBangToHoldCount
            );
            lastDashboardPacketMs = nowMs;
        }
    }

    private FtcDashboard safelyGetDashboard() {
        try {
            return FtcDashboard.getInstance();
        } catch (Throwable ignored) {
            return null;
        }
    }

    private void sendDashboardPacket(DriveSubsystem drive,
                                     double requestX,
                                     double requestY,
                                     double requestRot,
                                     boolean slowMode,
                                     boolean aimMode,
                                     boolean poseValid,
                                     double poseXIn,
                                     double poseYIn,
                                     double headingRad,
                                     double headingDeg,
                                     double ftcXIn,
                                     double ftcYIn,
                                     double ftcHeadingRad,
                                     boolean launcherReady,
                                     String launcherState,
                                     String launcherSpinMode,
                                     LauncherCoordinator launcherCoordinator,
                                     Alliance alliance,
                                     Alliance visionAlliance,
                                     double runtimeSec,
                                     boolean visionHasTag,
                                     int visionTagId,
                                     double visionRangeIn,
                                     double visionBearingDeg,
                                     double visionYawDeg,
                                     double visionPoseXIn,
                                     double visionPoseYIn,
                                     double visionHeadingRad,
                                     double visionTxDeg,
                                     double visionTyDeg,
                                     double visionTaPercent,
                                     boolean visionOdometryPending,
                                     String controlMode,
                                     double leftTargetRpm,
                                     double centerTargetRpm,
                                     double rightTargetRpm,
                                     double leftCurrentRpm,
                                     double centerCurrentRpm,
                                     double rightCurrentRpm,
                                     double leftPower,
                                     double centerPower,
                                     double rightPower,
                                     boolean leftReady,
                                     boolean centerReady,
                                     boolean rightReady,
                                     String leftPhase,
                                     String centerPhase,
                                     String rightPhase,
                                     boolean leftBang,
                                     boolean leftHold,
                                     boolean leftHybrid,
                                     boolean centerBang,
                                     boolean centerHold,
                                     boolean centerHybrid,
                                     boolean rightBang,
                                     boolean rightHold,
                                     boolean rightHybrid,
                                     int leftBangToHoldCount,
                                     int centerBangToHoldCount,
                                     int rightBangToHoldCount) {
        if (!enableDashboardTelemetry || dashboard == null) {
            return;
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("runtimeSec", runtimeSec);
        packet.put("drive/requestX", requestX);
        packet.put("drive/requestY", requestY);
        packet.put("drive/requestRot", requestRot);
        packet.put("drive/slowMode", slowMode);
        packet.put("drive/aimMode", aimMode);
        packet.put("drive/mode", drive.getDriveMode().name());
        packet.put("drive/commandTurn", drive.getLastCommandTurn());
        packet.put("drive/lfPower", drive.getLfPower());
        packet.put("drive/rfPower", drive.getRfPower());
        packet.put("drive/lbPower", drive.getLbPower());
        packet.put("drive/rbPower", drive.getRbPower());

        double lfVelIps = DistanceUnit.METER.toInches(Constants.Speed.ticksPerSecToMps(drive.getLfVelocityTicksPerSec()));
        double rfVelIps = DistanceUnit.METER.toInches(Constants.Speed.ticksPerSecToMps(drive.getRfVelocityTicksPerSec()));
        double lbVelIps = DistanceUnit.METER.toInches(Constants.Speed.ticksPerSecToMps(drive.getLbVelocityTicksPerSec()));
        double rbVelIps = DistanceUnit.METER.toInches(Constants.Speed.ticksPerSecToMps(drive.getRbVelocityTicksPerSec()));
        packet.put("drive/lfVelIps", lfVelIps);
        packet.put("drive/rfVelIps", rfVelIps);
        packet.put("drive/lbVelIps", lbVelIps);
        packet.put("drive/rbVelIps", rbVelIps);



        packet.put("launcher/ready", launcherReady);
        packet.put("launcher/controlMode", controlMode);
        packet.put("launcher/state", launcherState);
        packet.put("launcher/spinMode", launcherSpinMode);
        packet.put("launcher/left/targetRpm", leftTargetRpm);
        packet.put("launcher/left/currentRpm", leftCurrentRpm);
        packet.put("launcher/left/power", leftPower);
        packet.put("launcher/left/ready", leftReady);
        packet.put("launcher/center/targetRpm", centerTargetRpm);
        packet.put("launcher/center/currentRpm", centerCurrentRpm);
        packet.put("launcher/center/power", centerPower);
        packet.put("launcher/center/ready", centerReady);
        packet.put("launcher/right/targetRpm", rightTargetRpm);
        packet.put("launcher/right/currentRpm", rightCurrentRpm);
        packet.put("launcher/right/power", rightPower);
        packet.put("launcher/right/ready", rightReady);
        packet.put("launcher/left/phase", leftPhase);
        packet.put("launcher/center/phase", centerPhase);
        packet.put("launcher/right/phase", rightPhase);
        packet.put("launcher/left/isBang", leftBang);
        packet.put("launcher/left/isHold", leftHold);
        packet.put("launcher/left/isHybrid", leftHybrid);
        packet.put("launcher/center/isBang", centerBang);
        packet.put("launcher/center/isHold", centerHold);
        packet.put("launcher/center/isHybrid", centerHybrid);
        packet.put("launcher/right/isBang", rightBang);
        packet.put("launcher/right/isHold", rightHold);
        packet.put("launcher/right/isHybrid", rightHybrid);
        packet.put("launcher/left/bangToHoldCount", leftBangToHoldCount);
        packet.put("launcher/center/bangToHoldCount", centerBangToHoldCount);
        packet.put("launcher/right/bangToHoldCount", rightBangToHoldCount);
        Alliance activeAlliance = alliance == null ? Alliance.UNKNOWN : alliance;
        packet.put("alliance/id", activeAlliance.name());

        Alliance detectedAlliance = visionAlliance == null ? Alliance.UNKNOWN : visionAlliance;
        packet.put("vision/alliance", detectedAlliance.name());
        packet.put("vision/hasTag", visionHasTag);
        packet.put("vision/tagId", visionTagId);
        packet.put("vision/rangeIn", visionRangeIn);
        packet.put("vision/bearingDeg", visionBearingDeg);
        packet.put("vision/yawDeg", visionYawDeg);
        packet.put("vision/txDeg", visionTxDeg);
        packet.put("vision/tyDeg", visionTyDeg);
        packet.put("vision/targetAreaPercent", visionTaPercent);
        packet.put("vision/odometryPending", visionOdometryPending);

        packet.put("Pose/Pose x", poseXIn); // Inches (Pedro frame)
        packet.put("Pose/Pose y", poseYIn); // Inches (Pedro frame)
        packet.put("Pose/Pose heading", headingRad); // Radians (Pedro frame)
        if (!Double.isNaN(ftcXIn) && !Double.isNaN(ftcYIn)) {
            packet.put("Pose/FTC Pose x", ftcXIn); // Inches (FTC frame)
            packet.put("Pose/FTC Pose y", ftcYIn); // Inches (FTC frame)
        }
        if (!Double.isNaN(ftcHeadingRad)) {
            packet.put("Pose/FTC Pose heading", ftcHeadingRad); // Radians (FTC frame)
        }
        if (!Double.isNaN(routineStepOrdinal)) {
            packet.put("Autonomous/RoutineStep", routineStepOrdinal);
        }
        if (routineStepName != null && !routineStepName.isEmpty()) {
            packet.put("Autonomous/RoutineStepName", routineStepName);
        }

        boolean visionPoseValid = visionHasTag
                && !Double.isNaN(visionPoseXIn)
                && !Double.isNaN(visionPoseYIn)
                && !Double.isNaN(visionHeadingRad);

        if (visionPoseValid) {
            packet.put("Pose/Vision Pose x", visionPoseXIn);
            packet.put("Pose/Vision Pose y", visionPoseYIn);
            packet.put("Pose/Vision Pose heading", visionHeadingRad);
        }

        Canvas overlay = packet.fieldOverlay();
        if (poseValid) {
            double headingLength = 6.0;
            double endX = poseXIn + Math.cos(headingRad) * headingLength;
            double endY = poseYIn + Math.sin(headingRad) * headingLength;
            overlay.setStroke("white");
            overlay.setStrokeWidth(2);
            overlay.strokeCircle(poseXIn, poseYIn, 1.5);
            overlay.strokeLine(poseXIn, poseYIn, endX, endY);
        }

// Alternatively, headings can be published in degrees
        if (visionPoseValid) {
            double visionHeadingLength = 6.0;
            double visionEndX = visionPoseXIn + Math.cos(visionHeadingRad) * visionHeadingLength;
            double visionEndY = visionPoseYIn + Math.sin(visionHeadingRad) * visionHeadingLength;
            overlay.setStroke("lime");
            overlay.setStrokeWidth(2);
            overlay.strokeCircle(visionPoseXIn, visionPoseYIn, 2.0);
            overlay.strokeLine(visionPoseXIn, visionPoseYIn, visionEndX, visionEndY);
        }

        dashboard.sendTelemetryPacket(packet);
    }

    private static String formatValue(double value, String format) {
        return Double.isNaN(value) ? "--" : String.format(format, value);
    }

    private static VisionSnapshot captureVisionSnapshot(VisionSubsystemLimelight vision, Alliance allianceFallback) {
        VisionSnapshot snapshot = new VisionSnapshot();
        snapshot.alliance = allianceFallback == null ? Alliance.UNKNOWN : allianceFallback;
        if (vision == null) {
            return snapshot;
        }
        snapshot.alliance = vision.getAlliance();
        snapshot.odometryPending = vision.shouldUpdateOdometry();
        snapshot.hasTag = vision.hasValidTag();
        snapshot.tagId = snapshot.hasTag ? vision.getCurrentTagId() : -1;
        VisionSubsystemLimelight.TagSnapshot lastSnapshot = vision.getLastSnapshot().orElse(null);
        if (lastSnapshot != null) {
            snapshot.rangeIn = lastSnapshot.getFtcRange();
            snapshot.bearingDeg = lastSnapshot.getFtcBearing();
            snapshot.yawDeg = lastSnapshot.getFtcYaw();
            snapshot.poseXIn = lastSnapshot.getFtcX();
            snapshot.poseYIn = lastSnapshot.getFtcY();
            double yawDeg = lastSnapshot.getFtcYaw();
            snapshot.headingRad = Double.isNaN(yawDeg) ? Double.NaN : Math.toRadians(yawDeg);
            snapshot.txDeg = lastSnapshot.getTxDegrees();
            snapshot.tyDeg = lastSnapshot.getTyDegrees();
            snapshot.taPercent = lastSnapshot.getTargetAreaPercent();
            if (snapshot.alliance == Alliance.UNKNOWN) {
                snapshot.alliance = lastSnapshot.getAlliance();
            }
        }
        if (snapshot.alliance == Alliance.UNKNOWN) {
            snapshot.alliance = allianceFallback == null ? Alliance.UNKNOWN : allianceFallback;
        }
        return snapshot;
    }

    private static final class VisionSnapshot {
        boolean hasTag;
        int tagId = -1;
        double rangeIn = Double.NaN;
        double bearingDeg = Double.NaN;
        double yawDeg = Double.NaN;
        double poseXIn = Double.NaN;
        double poseYIn = Double.NaN;
        double headingRad = Double.NaN;
        double txDeg = Double.NaN;
        double tyDeg = Double.NaN;
        double taPercent = Double.NaN;
        boolean odometryPending;
        Alliance alliance = Alliance.UNKNOWN;
    }
}
