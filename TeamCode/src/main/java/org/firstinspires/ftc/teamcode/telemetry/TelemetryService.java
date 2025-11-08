package org.firstinspires.ftc.teamcode.telemetry;

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
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.PoseTransforms;

/**
 * Centralises telemetry output to FTControl Panels, the driver station, and optional PsiKit logging.
 * OpModes create a single instance and share its {@link TelemetryPublisher} across subsystems.
 */
public class TelemetryService {

    private final boolean enablePsiKitLogging;
    private final boolean enableDashboardTelemetry;
    private final PsiKitAdapter psiKitLogger;
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

    public TelemetryService(boolean enablePsiKitLogging) {
        this.enablePsiKitLogging = enablePsiKitLogging;
        this.enableDashboardTelemetry = TelemetrySettings.enableDashboardTelemetry;
        this.psiKitLogger = enablePsiKitLogging ? new PsiKitAdapter() : null;
        this.publisher = new TelemetryPublisher(null, psiKitLogger);
        this.dashboard = enableDashboardTelemetry ? safelyGetDashboard() : null;
    }

    public TelemetryService() {
        this(false);
    }

    /**
     * Prepares Panels and (optionally) starts PsiKit logging. Call once when the OpMode starts.
     */
    public void startSession() {
        if (!sessionActive) {
            panelsTelemetry = PanelsBridge.preparePanels();
            publisher.setTelemetryManager(panelsTelemetry);
            if (enablePsiKitLogging && psiKitLogger != null) {
                psiKitLogger.startSession();
            }
            sessionActive = true;
        }
    }

    /**
     * Stops any active logging session and flushes resources.
     */
    public void stopSession() {
        if (sessionActive && psiKitLogger != null) {
            psiKitLogger.stopSession();
        }
        publisher.setTelemetryManager(null);
        panelsTelemetry = null;
        sessionActive = false;
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
     * Provides access to the optional PsiKit adapter so that background loggers can stream
     * metrics without blocking the OpMode loop. This will be {@code null} when PsiKit logging
     * is disabled via {@link TelemetrySettings#enablePsiKitLogging}.
     */
    public PsiKitAdapter psiKitLogger() {
        return psiKitLogger;
    }

    public void publishLoopTelemetry(DriveSubsystem drive,
                                     ShooterSubsystem shooter,
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
        boolean headingHold = driveRequest != null && driveRequest.headingHold;
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
            PanelsBridge.drawFollowerDebug(drive.getFollower());
            lastPanelsDrawMs = nowMs;
        }

        TelemetryManager panels = panelsTelemetry();
        if (!suppressDriveTelemetry && panels != null) {
            String label = modeLabel == null || modeLabel.isEmpty() ? "Robot" : modeLabel;
            panels.debug("Mode", label);
            panels.debug("DriveMode", drive.getDriveMode());
            panels.debug("Drive/AimMode", aimMode);
            panels.debug("Drive/HeadingHoldRequest", headingHold);
            panels.debug("Vision/HasTag", visionSnapshot.hasTag);
            panels.debug("Vision/TagId", visionSnapshot.tagId);
            panels.debug("Vision/RangeIn", visionSnapshot.rangeIn);
            panels.debug("Vision/BearingDeg", visionSnapshot.bearingDeg);
            panels.debug("Vision/YawDeg", visionSnapshot.yawDeg);
            panels.debug("Vision/OdometryPending", visionSnapshot.odometryPending);
        }

        double leftTargetRpm = shooter.getTargetRpm(LauncherLane.LEFT);
        double centerTargetRpm = shooter.getTargetRpm(LauncherLane.CENTER);
        double rightTargetRpm = shooter.getTargetRpm(LauncherLane.RIGHT);
        double leftCurrentRpm = shooter.getCurrentRpm(LauncherLane.LEFT);
        double centerCurrentRpm = shooter.getCurrentRpm(LauncherLane.CENTER);
        double rightCurrentRpm = shooter.getCurrentRpm(LauncherLane.RIGHT);
        double leftPower = shooter.getLastPower(LauncherLane.LEFT);
        double centerPower = shooter.getLastPower(LauncherLane.CENTER);
        double rightPower = shooter.getLastPower(LauncherLane.RIGHT);
        boolean leftReady = shooter.isLaneReady(LauncherLane.LEFT);
        boolean centerReady = shooter.isLaneReady(LauncherLane.CENTER);
        boolean rightReady = shooter.isLaneReady(LauncherLane.RIGHT);
        String controlMode = ShooterSubsystem.getFlywheelControlMode().name();
        String leftPhase = shooter.getPhaseName(LauncherLane.LEFT);
        String centerPhase = shooter.getPhaseName(LauncherLane.CENTER);
        String rightPhase = shooter.getPhaseName(LauncherLane.RIGHT);
        boolean leftBang = "BANG".equals(leftPhase);
        boolean leftHold = "HOLD".equals(leftPhase);
        boolean leftHybrid = "HYBRID".equals(leftPhase);
        boolean centerBang = "BANG".equals(centerPhase);
        boolean centerHold = "HOLD".equals(centerPhase);
        boolean centerHybrid = "HYBRID".equals(centerPhase);
        boolean rightBang = "BANG".equals(rightPhase);
        boolean rightHold = "HOLD".equals(rightPhase);
        boolean rightHybrid = "HYBRID".equals(rightPhase);
        double displayTargetRpm = Math.max(Math.max(leftTargetRpm, centerTargetRpm), rightTargetRpm);
        double displayCurrentRpm = Math.max(Math.max(leftCurrentRpm, centerCurrentRpm), rightCurrentRpm);
        double displayPower = Math.max(Math.max(leftPower, centerPower), rightPower);

        boolean shooterReady;
        if (launcherCoordinator != null) {
            shooterReady = launcherCoordinator.logShooterReadyEvent(logger);
        } else {
            shooterReady = shooter.atTarget();
        }

        if (!suppressDriveTelemetry) {
            publisher.publishDrive(drive, requestX, requestY, requestRot, slowMode, aimMode, headingHold);
        }
        publisher.publishShooter(
                displayTargetRpm,
                displayCurrentRpm,
                displayPower,
                displayTargetRpm - displayCurrentRpm
        );

        boolean autoSpin = launcherCoordinator != null && launcherCoordinator.isAutoSpinEnabled();

        if (panels != null) {
            panels.debug("shooter/controlMode", controlMode);
            panels.debug("shooter/autoSpin", autoSpin);
            panels.debug("shooter/leftTargetRpm", leftTargetRpm);
            panels.debug("shooter/leftCurrentRpm", leftCurrentRpm);
            panels.debug("shooter/leftPower", leftPower);
            panels.debug("shooter/leftReady", leftReady);
            panels.debug("shooter/centerTargetRpm", centerTargetRpm);
            panels.debug("shooter/centerCurrentRpm", centerCurrentRpm);
            panels.debug("shooter/centerPower", centerPower);
            panels.debug("shooter/centerReady", centerReady);
            panels.debug("shooter/rightTargetRpm", rightTargetRpm);
            panels.debug("shooter/rightCurrentRpm", rightCurrentRpm);
            panels.debug("shooter/rightPower", rightPower);
            panels.debug("shooter/rightReady", rightReady);
            panels.debug("shooter/leftPhase", leftPhase);
            panels.debug("shooter/centerPhase", centerPhase);
            panels.debug("shooter/rightPhase", rightPhase);
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
            dsTelemetry.addData("Drive Heading Hold", headingHold ? "engaged" : "free");
            dsTelemetry.addData("Drive Turn Cmd",
                    "%.2f (lock %.2f)",
                    drive.getLastCommandTurn(),
                    drive.getHeadingLockOutput());
            dsTelemetry.addData(
                    "Shooter Mode",
                    "%s | ready=%s | autoSpin=%s",
                    controlMode,
                    shooterReady,
                    autoSpin
            );
            dsTelemetry.addData(
                    "Shooter Left",
                    "T=%.0f  C=%.0f  P=%.2f  %s  %s",
                    leftTargetRpm,
                    leftCurrentRpm,
                    leftPower,
                    leftReady ? "READY" : "----",
                    leftPhase
            );
            dsTelemetry.addData(
                    "Shooter Center",
                    "T=%.0f  C=%.0f  P=%.2f  %s  %s",
                    centerTargetRpm,
                    centerCurrentRpm,
                    centerPower,
                    centerReady ? "READY" : "----",
                    centerPhase
            );
            dsTelemetry.addData(
                    "Shooter Right",
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
                    headingHold,
                    pose != null,
                    poseXIn,
                    poseYIn,
                    headingRad,
                    headingDeg,
                    ftcXIn,
                    ftcYIn,
                    ftcHeadingRad,
                    shooterReady,
                    autoSpin,
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
                    rightHybrid
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
                                     boolean headingHold,
                                     boolean poseValid,
                                     double poseXIn,
                                     double poseYIn,
                                     double headingRad,
                                     double headingDeg,
                                     double ftcXIn,
                                     double ftcYIn,
                                     double ftcHeadingRad,
                                     boolean shooterReady,
                                     boolean autoSpin,
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
                                     boolean rightHybrid) {
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
        packet.put("drive/headingHold", headingHold);
        packet.put("drive/mode", drive.getDriveMode().name());
        packet.put("drive/commandTurn", drive.getLastCommandTurn());
        packet.put("drive/headingLockOutput", drive.getHeadingLockOutput());
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

        packet.put("shooter/ready", shooterReady);
        packet.put("shooter/controlMode", controlMode);
        packet.put("shooter/left/targetRpm", leftTargetRpm);
        packet.put("shooter/left/currentRpm", leftCurrentRpm);
        packet.put("shooter/left/power", leftPower);
        packet.put("shooter/left/ready", leftReady);
        packet.put("shooter/center/targetRpm", centerTargetRpm);
        packet.put("shooter/center/currentRpm", centerCurrentRpm);
        packet.put("shooter/center/power", centerPower);
        packet.put("shooter/center/ready", centerReady);
        packet.put("shooter/right/targetRpm", rightTargetRpm);
        packet.put("shooter/right/currentRpm", rightCurrentRpm);
        packet.put("shooter/right/power", rightPower);
        packet.put("shooter/right/ready", rightReady);
        packet.put("shooter/left/phase", leftPhase);
        packet.put("shooter/center/phase", centerPhase);
        packet.put("shooter/right/phase", rightPhase);
        packet.put("shooter/left/isBang", leftBang);
        packet.put("shooter/left/isHold", leftHold);
        packet.put("shooter/left/isHybrid", leftHybrid);
        packet.put("shooter/center/isBang", centerBang);
        packet.put("shooter/center/isHold", centerHold);
        packet.put("shooter/center/isHybrid", centerHybrid);
        packet.put("shooter/right/isBang", rightBang);
        packet.put("shooter/right/isHold", rightHold);
        packet.put("shooter/right/isHybrid", rightHybrid);
        packet.put("lanes/autoSpinEnabled", autoSpin);

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
