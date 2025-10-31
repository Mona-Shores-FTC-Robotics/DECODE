package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * Centralises telemetry output to FTControl Panels, the driver station, and optional PsiKit logging.
 * OpModes create a single instance and share its {@link TelemetryPublisher} across subsystems.
 */
public class TelemetryService {

    private final boolean enablePsiKitLogging;
    private final PsiKitAdapter psiKitLogger;
    private final TelemetryPublisher publisher;

    private TelemetryManager panelsTelemetry;
    private boolean sessionActive = false;

    public TelemetryService(boolean enablePsiKitLogging) {
        this.enablePsiKitLogging = enablePsiKitLogging;
        this.psiKitLogger = enablePsiKitLogging ? new PsiKitAdapter() : null;
        this.publisher = new TelemetryPublisher(null, psiKitLogger);
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

        PanelsBridge.drawFollowerDebug(drive.getFollower());

        TelemetryManager panels = panelsTelemetry();
        if (panels != null) {
            String label = modeLabel == null || modeLabel.isEmpty() ? "Robot" : modeLabel;
            panels.debug("Mode", label);
            panels.debug("DriveMode", drive.getDriveMode());
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
            Pose2D pose = drive.getPose();
            if (pose != null) {
                dsTelemetry.addData("Pose", "x=%.1f in  y=%.1f in  h=%.1f°",
                        pose.getX(DistanceUnit.INCH),
                        pose.getY(DistanceUnit.INCH),
                        pose.getHeading(AngleUnit.DEGREES));
            } else {
                dsTelemetry.addData("Pose", "(unavailable)");
            }

            Alliance activeAlliance = alliance == null ? Alliance.UNKNOWN : alliance;
            dsTelemetry.addData("Alliance", activeAlliance.displayName());
            dsTelemetry.addData("Shooter RPM", "%.0f / %.0f  %s",
                    currentRpm,
                    targetRpm,
                    shooterReady ? "ready" : "spooling");
            dsTelemetry.addData("Drive Mode", drive.getDriveMode());
            dsTelemetry.addData("Auto Spin", LauncherCoordinator.autoSpinEnabled ? "enabled" : "disabled");
            dsTelemetry.addData("Runtime (s)", runtimeSec);
        }

        if (launcherCoordinator != null) {
            launcherCoordinator.publishLaneTelemetry(dsTelemetry, panels);
        }

        if (dsTelemetry != null) {
            dsTelemetry.update();
            updateDriverStation(dsTelemetry);
        }
    }
}
