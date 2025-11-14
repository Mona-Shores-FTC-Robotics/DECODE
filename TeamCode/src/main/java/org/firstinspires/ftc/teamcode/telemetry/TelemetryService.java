package org.firstinspires.ftc.teamcode.telemetry;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import android.os.SystemClock;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

import Ori.Coval.Logging.Logger.KoalaLog;

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

import java.util.Map;

/**
 * Centralises telemetry output to FTControl Panels and the driver station.
 * OpModes create a single instance and share its {@link TelemetryPublisher} across subsystems.
 * <p>
 * Note: Logging is now handled by KoalaLog's @AutoLog annotation system.
 * </p>
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

    /**
     * Publishes telemetry for the current loop to all available outputs.
     * Telemetry verbosity is controlled by TelemetrySettings.config.level.
     */
    public void publishLoopTelemetry(DriveSubsystem drive,
                                     LauncherSubsystem launcher,
                                     VisionSubsystemLimelight vision,
                                     DriverBindings.DriveRequest driveRequest,
                                     LauncherCoordinator launcherCoordinator,
                                     Alliance alliance,
                                     double runtimeSec,
                                     Telemetry dsTelemetry,
                                     String modeLabel,
                                     boolean suppressDriveTelemetry,
                                     Pose poseOverride) {
        TelemetrySettings.TelemetryLevel level = TelemetrySettings.config.level;

        switch (level) {
            case MATCH:
                publishMatchTelemetry(drive, launcher, vision, launcherCoordinator, alliance, dsTelemetry, poseOverride);
                break;
            case PRACTICE:
                publishPracticeTelemetry(drive, launcher, vision, driveRequest, launcherCoordinator, alliance, runtimeSec, dsTelemetry, modeLabel, suppressDriveTelemetry, poseOverride);
                break;
            case DEBUG:
            default:
                publishDebugTelemetry(drive, launcher, vision, driveRequest, launcherCoordinator, alliance, runtimeSec, dsTelemetry, modeLabel, suppressDriveTelemetry, poseOverride);
                break;
        }
    }

    /**
     * MATCH mode: Minimal telemetry for competition (<10ms target).
     * - Essential driver station info only
     * - Critical pose logging preserved (done in DriveSubsystem.periodic)
     * - No FTC Dashboard packets
     * - No FullPanels telemetry
     */
    private void publishMatchTelemetry(DriveSubsystem drive,
                                       LauncherSubsystem launcher,
                                       VisionSubsystemLimelight vision,
                                       LauncherCoordinator launcherCoordinator,
                                       Alliance alliance,
                                       Telemetry dsTelemetry,
                                       Pose poseOverride) {
        if (dsTelemetry == null) {
            return;
        }

        // Pose (essential for driver awareness)
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

        if (pose != null) {
            dsTelemetry.addData("Pose", "x=%.1f in  y=%.1f in  h=%.1f°",
                    pose.getX(DistanceUnit.INCH),
                    pose.getY(DistanceUnit.INCH),
                    pose.getHeading(AngleUnit.DEGREES));
        } else {
            dsTelemetry.addData("Pose", "(unavailable)");
        }

        // Alliance
        Alliance activeAlliance = alliance == null ? Alliance.UNKNOWN : alliance;
        dsTelemetry.addData("Alliance", activeAlliance.displayName());

        // Launcher ready status (critical for driver)
        boolean launcherReady = launcher.atTarget();
        dsTelemetry.addData("Launcher", launcherReady ? "READY" : "NOT READY");

        // Vision tag detection (helpful for relocalization awareness)
        if (vision.hasValidTag()) {
            dsTelemetry.addData("Vision", "Tag %d", vision.getCurrentTagId());
        }

        // Artifact count (critical for match strategy)
        if (launcherCoordinator != null) {
            dsTelemetry.addData("Artifacts", "%d", launcherCoordinator.getArtifactCount());
        }

        dsTelemetry.update();
    }

    /**
     * PRACTICE mode: Moderate telemetry for tuning sessions (<20ms target).
     * - Basic FTC Dashboard metrics
     * - High-level subsystem state
     * - Suitable for real-time parameter adjustment
     */
    private void publishPracticeTelemetry(DriveSubsystem drive,
                                          LauncherSubsystem launcher,
                                          VisionSubsystemLimelight vision,
                                          DriverBindings.DriveRequest driveRequest,
                                          LauncherCoordinator launcherCoordinator,
                                          Alliance alliance,
                                          double runtimeSec,
                                          Telemetry dsTelemetry,
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

        double poseXIn = pose != null ? pose.getX(DistanceUnit.INCH) : 0.0;
        double poseYIn = pose != null ? pose.getY(DistanceUnit.INCH) : 0.0;
        double headingDeg = pose != null ? pose.getHeading(AngleUnit.DEGREES) : 0.0;
        double headingRad = pose != null ? pose.getHeading(AngleUnit.RADIANS) : 0.0;

        VisionSnapshot visionSnapshot = captureVisionSnapshot(vision, alliance);
        boolean launcherReady = launcher.atTarget();
        String launcherState = launcher.getState().name();

        // Driver station telemetry (moderate detail)
        if (dsTelemetry != null && !suppressDriveTelemetry) {
            if (pose != null) {
                dsTelemetry.addData("Pose", "x=%.1f in  y=%.1f in  h=%.1f°",
                        poseXIn, poseYIn, headingDeg);
            } else {
                dsTelemetry.addData("Pose", "(unavailable)");
            }

            Alliance activeAlliance = alliance == null ? Alliance.UNKNOWN : alliance;
            dsTelemetry.addData("Alliance", activeAlliance.displayName());
            dsTelemetry.addData("Drive", "%s | aim=%s",
                    drive.getDriveMode().name(),
                    aimMode ? "ON" : "OFF");
            dsTelemetry.addData("Launcher", "%s | ready=%s",
                    launcherState,
                    launcherReady ? "YES" : "NO");

            if (visionSnapshot.hasTag) {
                dsTelemetry.addData("Vision", "Tag %d @ %.1f in",
                        visionSnapshot.tagId,
                        visionSnapshot.rangeIn);
            }

            if (launcherCoordinator != null) {
                dsTelemetry.addData("Artifacts", "%d", launcherCoordinator.getArtifactCount());
            }

            dsTelemetry.update();
        }

        // FTC Dashboard packets (throttled to 100ms in PRACTICE mode)
        long nowMs = SystemClock.uptimeMillis();
        long dashboardInterval = TelemetrySettings.getDashboardInterval();
        boolean sendDashboardThisLoop = (dashboardInterval > 0) &&
                                        (nowMs - lastDashboardPacketMs >= dashboardInterval);

        if (sendDashboardThisLoop && TelemetrySettings.shouldSendDashboardPackets() && dashboard != null) {
            Pose ftcPose = PoseTransforms.toFtcPose(pedroPose);
            double ftcXIn = ftcPose != null ? ftcPose.getX() : Double.NaN;
            double ftcYIn = ftcPose != null ? ftcPose.getY() : Double.NaN;
            double ftcHeadingRad = ftcPose != null ? ftcPose.getHeading() : Double.NaN;

            TelemetryPacket packet = new TelemetryPacket();

            // Essential pose data
            packet.put("Pose/Pose x", poseXIn);
            packet.put("Pose/Pose y", poseYIn);
            packet.put("Pose/Pose heading", headingRad);
            if (!Double.isNaN(ftcXIn) && !Double.isNaN(ftcYIn)) {
                packet.put("Pose/FTC Pose x", ftcXIn);
                packet.put("Pose/FTC Pose y", ftcYIn);
            }
            if (!Double.isNaN(ftcHeadingRad)) {
                packet.put("Pose/FTC Pose heading", ftcHeadingRad);
            }

            // Drive state
            packet.put("drive/mode", drive.getDriveMode().name());
            packet.put("drive/aimMode", aimMode);
            packet.put("drive/slowMode", slowMode);

            // Launcher high-level state
            packet.put("launcher/ready", launcherReady);
            packet.put("launcher/state", launcherState);

            // Vision essentials
            packet.put("vision/hasTag", visionSnapshot.hasTag);
            packet.put("vision/tagId", visionSnapshot.tagId);
            packet.put("vision/rangeIn", visionSnapshot.rangeIn);

            // Alliance
            Alliance activeAlliance = alliance == null ? Alliance.UNKNOWN : alliance;
            packet.put("alliance/id", activeAlliance.name());

            // Runtime
            packet.put("runtimeSec", runtimeSec);

            // Field overlay
            Canvas overlay = packet.fieldOverlay();
            if (pose != null) {
                double headingLength = 6.0;
                double endX = poseXIn + Math.cos(headingRad) * headingLength;
                double endY = poseYIn + Math.sin(headingRad) * headingLength;
                overlay.setStroke("white");
                overlay.setStrokeWidth(2);
                overlay.strokeCircle(poseXIn, poseYIn, 1.5);
                overlay.strokeLine(poseXIn, poseYIn, endX, endY);
            }

            dashboard.sendTelemetryPacket(packet);
            lastDashboardPacketMs = nowMs;
        }
    }

    /**
     * DEBUG mode: Full telemetry for development and debugging (current behavior).
     * - All @AutoLogOutput methods
     * - Complete dashboard packets
     * - Full FullPanels telemetry
     * - All diagnostics
     */
    private void publishDebugTelemetry(DriveSubsystem drive,
                                       LauncherSubsystem launcher,
                                       VisionSubsystemLimelight vision,
                                       DriverBindings.DriveRequest driveRequest,
                                       LauncherCoordinator launcherCoordinator,
                                       Alliance alliance,
                                       double runtimeSec,
                                       Telemetry dsTelemetry,
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

        boolean launcherReady = launcher.atTarget();
        String launcherState = launcher.getState().name();
        String launcherSpinMode = launcher.getEffectiveSpinMode().name();

       publisher.publishDrive(drive, requestX, requestY, requestRot, slowMode, aimMode);

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

    /**
     * Helper method to log a value to both FTC Dashboard and KoalaLog.
     *
     * @param packet The telemetry packet to add the value to
     * @param key The telemetry key
     * @param value The value to log (can be any type: double, String, boolean, int, etc.)
     */
    private void log(TelemetryPacket packet, String key, Boolean value) {
        packet.put(key, value);
        KoalaLog.log(key, value, true);
    }

    private void log(TelemetryPacket packet, String key, String value) {
        packet.put(key, value);
        KoalaLog.log(key, value, true);
    }

    private void log(TelemetryPacket packet, String key, Double value) {
        packet.put(key, value);
        KoalaLog.log(key, value, true);
    }

    private void log(TelemetryPacket packet, String key, int value) {
        packet.put(key, value);
        KoalaLog.log(key, value, true);
    }

    /**
     * Sends a telemetry packet to the FTC Dashboard.
     */
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
        log(packet, "runtimeSec", runtimeSec);
        log(packet, "drive/requestX", requestX);
        log(packet, "drive/requestY", requestY);
        log(packet, "drive/requestRot", requestRot);
        log(packet, "drive/slowMode", slowMode);
        log(packet, "drive/aimMode", aimMode);
        log(packet, "drive/mode", drive.getDriveMode().name());
        log(packet, "drive/commandTurn", drive.getLastCommandTurn());
        log(packet, "drive/lfPower", drive.getLfPower());
        log(packet, "drive/rfPower", drive.getRfPower());
        log(packet, "drive/lbPower", drive.getLbPower());
        log(packet, "drive/rbPower", drive.getRbPower());

        double lfVelIps = DistanceUnit.METER.toInches(Constants.Speed.ticksPerSecToMps(drive.getLfVelocityTicksPerSec()));
        double rfVelIps = DistanceUnit.METER.toInches(Constants.Speed.ticksPerSecToMps(drive.getRfVelocityTicksPerSec()));
        double lbVelIps = DistanceUnit.METER.toInches(Constants.Speed.ticksPerSecToMps(drive.getLbVelocityTicksPerSec()));
        double rbVelIps = DistanceUnit.METER.toInches(Constants.Speed.ticksPerSecToMps(drive.getRbVelocityTicksPerSec()));
        log(packet, "drive/lfVelIps", lfVelIps);
        log(packet, "drive/rfVelIps", rfVelIps);
        log(packet, "drive/lbVelIps", lbVelIps);
        log(packet, "drive/rbVelIps", rbVelIps);



        log(packet, "launcher/ready", launcherReady);
        log(packet, "launcher/controlMode", controlMode);
        log(packet, "launcher/state", launcherState);
        log(packet, "launcher/spinMode", launcherSpinMode);
        log(packet, "launcher/left/targetRpm", leftTargetRpm);
        log(packet, "launcher/left/currentRpm", leftCurrentRpm);
        log(packet, "launcher/left/power", leftPower);
        log(packet, "launcher/left/ready", leftReady);
        log(packet, "launcher/center/targetRpm", centerTargetRpm);
        log(packet, "launcher/center/currentRpm", centerCurrentRpm);
        log(packet, "launcher/center/power", centerPower);
        log(packet, "launcher/center/ready", centerReady);
        log(packet, "launcher/right/targetRpm", rightTargetRpm);
        log(packet, "launcher/right/currentRpm", rightCurrentRpm);
        log(packet, "launcher/right/power", rightPower);
        log(packet, "launcher/right/ready", rightReady);
        log(packet, "launcher/left/phase", leftPhase);
        log(packet, "launcher/center/phase", centerPhase);
        log(packet, "launcher/right/phase", rightPhase);
        log(packet, "launcher/left/isBang", leftBang);
        log(packet, "launcher/left/isHold", leftHold);
        log(packet, "launcher/left/isHybrid", leftHybrid);
        log(packet, "launcher/center/isBang", centerBang);
        log(packet, "launcher/center/isHold", centerHold);
        log(packet, "launcher/center/isHybrid", centerHybrid);
        log(packet, "launcher/right/isBang", rightBang);
        log(packet, "launcher/right/isHold", rightHold);
        log(packet, "launcher/right/isHybrid", rightHybrid);
        log(packet, "launcher/left/bangToHoldCount", leftBangToHoldCount);
        log(packet, "launcher/center/bangToHoldCount", centerBangToHoldCount);
        log(packet, "launcher/right/bangToHoldCount", rightBangToHoldCount);
        Alliance activeAlliance = alliance == null ? Alliance.UNKNOWN : alliance;
        log(packet, "alliance/id", activeAlliance.name());

        Alliance detectedAlliance = visionAlliance == null ? Alliance.UNKNOWN : visionAlliance;
        log(packet, "vision/alliance", detectedAlliance.name());
        log(packet, "vision/hasTag", visionHasTag);
        log(packet, "vision/tagId", visionTagId);
        log(packet, "vision/rangeIn", visionRangeIn);
        log(packet, "vision/bearingDeg", visionBearingDeg);
        log(packet, "vision/yawDeg", visionYawDeg);
        log(packet, "vision/txDeg", visionTxDeg);
        log(packet, "vision/tyDeg", visionTyDeg);
        log(packet, "vision/targetAreaPercent", visionTaPercent);
        log(packet, "vision/odometryPending", visionOdometryPending);

        log(packet, "Pose/Pose x", poseXIn); // Inches (Pedro frame)
        log(packet, "Pose/Pose y", poseYIn); // Inches (Pedro frame)
        log(packet, "Pose/Pose heading", headingRad); // Radians (Pedro frame)
        if (!Double.isNaN(ftcXIn) && !Double.isNaN(ftcYIn)) {
            log(packet, "Pose/FTC Pose x", ftcXIn); // Inches (FTC frame)
            log(packet, "Pose/FTC Pose y", ftcYIn); // Inches (FTC frame)
        }
        if (!Double.isNaN(ftcHeadingRad)) {
            log(packet, "Pose/FTC Pose heading", ftcHeadingRad); // Radians (FTC frame)
        }
        if (!Double.isNaN(routineStepOrdinal)) {
            log(packet, "Autonomous/RoutineStep", routineStepOrdinal);
        }
        if (routineStepName != null && !routineStepName.isEmpty()) {
            log(packet, "Autonomous/RoutineStepName", routineStepName);
        }

        boolean visionPoseValid = visionHasTag
                && !Double.isNaN(visionPoseXIn)
                && !Double.isNaN(visionPoseYIn)
                && !Double.isNaN(visionHeadingRad);

        if (visionPoseValid) {
            log(packet, "Pose/Vision Pose x", visionPoseXIn);
            log(packet, "Pose/Vision Pose y", visionPoseYIn);
            log(packet, "Pose/Vision Pose heading", visionHeadingRad);
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
