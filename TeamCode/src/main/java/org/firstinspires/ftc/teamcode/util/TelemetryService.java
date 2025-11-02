package org.firstinspires.ftc.teamcode.util;

import android.os.SystemClock;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.TelemetryManager;

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

    public void updateDriverStation(Telemetry telemetry) {
        if (panelsTelemetry != null) {
            panelsTelemetry.update(telemetry);
        }
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
                                     String modeLabel) {
        double requestX = driveRequest != null ? driveRequest.fieldX : 0.0;
        double requestY = driveRequest != null ? driveRequest.fieldY : 0.0;
        double requestRot = driveRequest != null ? driveRequest.rotation : 0.0;
        boolean slowMode = driveRequest != null && driveRequest.slowMode;
        Pose2D pose = drive.getPose();
        double poseXIn = pose != null ? pose.getX(DistanceUnit.INCH) : 0.0;
        double poseYIn = pose != null ? pose.getY(DistanceUnit.INCH) : 0.0;
        double headingDeg = pose != null ? pose.getHeading(AngleUnit.DEGREES) : 0.0;
        double headingRad = pose != null ? pose.getHeading(AngleUnit.RADIANS) : 0.0;

        boolean visionHasTag = false;
        int visionTagId = -1;
        double visionRangeIn = Double.NaN;
        double visionBearingDeg = Double.NaN;
        double visionYawDeg = Double.NaN;
        double visionPoseXIn = Double.NaN;
        double visionPoseYIn = Double.NaN;
        double visionHeadingRad = Double.NaN;
        double visionTxDeg = Double.NaN;
        double visionTyDeg = Double.NaN;
        double visionTaPercent = Double.NaN;
        boolean visionOdometryPending = false;
        Alliance visionAlliance = Alliance.UNKNOWN;

        if (vision != null) {
            visionAlliance = vision.getAlliance();
            visionOdometryPending = vision.shouldUpdateOdometry();
            visionHasTag = vision.hasValidTag();
            visionTagId = visionHasTag ? vision.getCurrentTagId() : -1;
            VisionSubsystemLimelight.TagSnapshot snapshot = vision.getLastSnapshot().orElse(null);
            if (snapshot != null) {
                visionRangeIn = snapshot.getFtcRange();
                visionBearingDeg = snapshot.getFtcBearing();
                visionYawDeg = snapshot.getFtcYaw();
                visionPoseXIn = snapshot.getRobotX();
                visionPoseYIn = snapshot.getRobotY();
                double snapshotYawDeg = snapshot.getRobotYaw();
                visionHeadingRad = Double.isNaN(snapshotYawDeg) ? Double.NaN : Math.toRadians(snapshotYawDeg);
                visionTxDeg = snapshot.getTxDegrees();
                visionTyDeg = snapshot.getTyDegrees();
                visionTaPercent = snapshot.getTargetAreaPercent();
                if (visionAlliance == Alliance.UNKNOWN) {
                    visionAlliance = snapshot.getAlliance();
                }
            }
        }
        if (visionAlliance == Alliance.UNKNOWN) {
            visionAlliance = alliance == null ? Alliance.UNKNOWN : alliance;
        }

        long nowMs = SystemClock.uptimeMillis();
        boolean drawPanelsThisLoop = nowMs - lastPanelsDrawMs >= PANELS_DRAW_INTERVAL_MS;
        boolean sendDashboardThisLoop = nowMs - lastDashboardPacketMs >= DASHBOARD_PACKET_INTERVAL_MS;

        if (drawPanelsThisLoop && ENABLE_PANELS_DRAWING) {
            PanelsBridge.drawFollowerDebug(drive.getFollower());
            lastPanelsDrawMs = nowMs;
        }

        TelemetryManager panels = panelsTelemetry();
        if (panels != null) {
            String label = modeLabel == null || modeLabel.isEmpty() ? "Robot" : modeLabel;
            panels.debug("Mode", label);
            panels.debug("DriveMode", drive.getDriveMode());
            panels.debug("Vision/HasTag", visionHasTag);
            panels.debug("Vision/TagId", visionTagId);
            panels.debug("Vision/RangeIn", visionRangeIn);
            panels.debug("Vision/BearingDeg", visionBearingDeg);
            panels.debug("Vision/YawDeg", visionYawDeg);
            panels.debug("Vision/OdometryPending", visionOdometryPending);
        }

        double currentRpm = shooter.getCurrentRpm();
        double targetRpm = shooter.getTargetRpm();

        boolean shooterReady;
        if (launcherCoordinator != null) {
            shooterReady = launcherCoordinator.logShooterReadyEvent(logger);
        } else {
            shooterReady = shooter.atTarget();
        }

        publisher.publishDrive(drive, requestX, requestY, requestRot, slowMode);
        publisher.publishShooter(
                targetRpm,
                currentRpm,
                shooter.getLastPower(),
                targetRpm - currentRpm
        );

        if (logger != null) {
            logger.logNumber("Robot", "RuntimeSec", runtimeSec);
        }

        if (dsTelemetry != null) {
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
                    pose != null,
                    poseXIn,
                    poseYIn,
                    headingRad,
                    headingDeg,
                    currentRpm,
                    targetRpm,
                    shooterReady,
                    launcherCoordinator,
                    alliance,
                    visionAlliance,
                    runtimeSec,
                    visionHasTag,
                    visionTagId,
                    visionRangeIn,
                    visionBearingDeg,
                    visionYawDeg,
                    visionPoseXIn,
                    visionPoseYIn,
                    visionHeadingRad,
                    visionTxDeg,
                    visionTyDeg,
                    visionTaPercent,
                    visionOdometryPending
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
                                     boolean poseValid,
                                     double poseXIn,
                                     double poseYIn,
                                     double headingRad,
                                     double headingDeg,
                                     double currentRpm,
                                     double targetRpm,
                                     boolean shooterReady,
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
                                     boolean visionOdometryPending) {
        if (!enableDashboardTelemetry || dashboard == null) {
            return;
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("runtimeSec", runtimeSec);
        packet.put("drive/requestX", requestX);
        packet.put("drive/requestY", requestY);
        packet.put("drive/requestRot", requestRot);
        packet.put("drive/slowMode", slowMode);
        packet.put("drive/mode", drive.getDriveMode().name());
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

        packet.put("shooter/targetRpm", targetRpm);
        packet.put("shooter/currentRpm", currentRpm);
        packet.put("shooter/ready", shooterReady);
        if (launcherCoordinator != null) {
            packet.put("lanes/autoSpinEnabled", LauncherCoordinator.autoSpinEnabled);
        }

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

        packet.put("pose/x", poseXIn);
        packet.put("pose/y", poseYIn);
        packet.put("pose/headingDeg", headingDeg);
        packet.put("pose/headingRad", headingRad);
        packet.put("pose/heading", headingRad);
        packet.put("pose/valid", poseValid);

        double poseXMeters = DistanceUnit.INCH.toMeters(poseXIn);
        double poseYMeters = DistanceUnit.INCH.toMeters(poseYIn);
        packet.put("pose/meters/x", poseXMeters);
        packet.put("pose/meters/y", poseYMeters);
        packet.put("pose/meters/headingRad", headingRad);

        packet.put("pose3d/xMeters", poseXMeters);
        packet.put("pose3d/yMeters", poseYMeters);
        packet.put("pose3d/zMeters", 0.0);
        packet.put("pose3d/rollRad", 0.0);
        packet.put("pose3d/pitchRad", 0.0);
        packet.put("pose3d/yawRad", headingRad);

        packet.put("Pose x", poseXIn); // Inches
        packet.put("Pose y", poseYIn); // Inches
        packet.put("Pose heading", headingRad); // Radians

        boolean visionPoseValid = visionHasTag
                && !Double.isNaN(visionPoseXIn)
                && !Double.isNaN(visionPoseYIn)
                && !Double.isNaN(visionHeadingRad);
        packet.put("visionPose/valid", visionPoseValid);
        packet.put("visionPose/x", visionPoseXIn);
        packet.put("visionPose/y", visionPoseYIn);
        packet.put("visionPose/headingRad", visionHeadingRad);

        double visionPoseXMeters = DistanceUnit.INCH.toMeters(visionPoseXIn);
        double visionPoseYMeters = DistanceUnit.INCH.toMeters(visionPoseYIn);
        packet.put("visionPose/meters/x", visionPoseXMeters);
        packet.put("visionPose/meters/y", visionPoseYMeters);
        packet.put("visionPose/meters/headingRad", visionHeadingRad);

        packet.put("visionPose3d/xMeters", visionPoseXMeters);
        packet.put("visionPose3d/yMeters", visionPoseYMeters);
        packet.put("visionPose3d/zMeters", 0.0);
        packet.put("visionPose3d/rollRad", 0.0);
        packet.put("visionPose3d/pitchRad", 0.0);
        packet.put("visionPose3d/yawRad", visionHeadingRad);

        if (visionPoseValid) {
            packet.put("Vision Pose x", visionPoseXIn);
            packet.put("Vision Pose y", visionPoseYIn);
            packet.put("Vision Pose heading (deg)", Math.toDegrees(visionHeadingRad));
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
        packet.put("Pose heading (deg)", 180.0); // Degrees
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
}
