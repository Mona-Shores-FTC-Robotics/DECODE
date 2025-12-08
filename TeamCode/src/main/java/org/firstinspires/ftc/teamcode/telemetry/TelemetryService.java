package org.firstinspires.ftc.teamcode.telemetry;

import android.os.SystemClock;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.telemetry.data.RobotTelemetryData;
import org.firstinspires.ftc.teamcode.telemetry.formatters.DashboardFormatter;
import org.firstinspires.ftc.teamcode.telemetry.formatters.DriverStationFormatter;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotState;

/**
 * Centralized telemetry service for DECODE robot.
 * <p>
 * Provides telemetry output (MATCH/DEBUG) to multiple destinations:
 * - Driver Station (FTC telemetry)
 * - FTC Dashboard (web dashboard)
 * </p>
 * <p>
 * This service acts as a thin orchestrator - all formatting logic is delegated to
 * specialized formatters in the telemetry.formatters package.
 * </p>
 */
public class TelemetryService {

    private static final long DASHBOARD_INTERVAL_MS = 50L; // 20 Hz in DEBUG mode

    private FtcDashboard dashboard;
    private boolean dashboardInitialized = false;
    private final DriverStationFormatter driverStationFormatter;
    private final DashboardFormatter dashboardFormatter;

    private boolean sessionActive = false;
    private long lastDashboardPacketMs = 0L;

    /**
     * Create a new telemetry service.
     * Dashboard is lazily initialized only when needed (not in MATCH mode).
     */
    public TelemetryService() {
        // Don't initialize dashboard in constructor - do it lazily when first needed
        this.dashboard = null;
        this.driverStationFormatter = new DriverStationFormatter();
        this.dashboardFormatter = new DashboardFormatter();
    }

    /**
     * Lazily initialize FTC Dashboard only when actually needed.
     * This avoids starting the dashboard server in MATCH mode.
     */
    private FtcDashboard getDashboardIfNeeded() {
        if (!dashboardInitialized && TelemetrySettings.shouldInitializeDashboard()) {
            dashboard = safelyGetDashboard();
            dashboardInitialized = true;
        }
        return dashboard;
    }

    /**
     * Prepares FullPanels for telemetry. Call once when the OpMode starts.
     */
    public void startSession() {
        if (!sessionActive) {
            sessionActive = true;
        }
    }

    /**
     * Stops the telemetry session and flushes resources.
     */
    public void stopSession() {
        if (sessionActive) {
            sessionActive = false;
        }
    }

    /**
     * Publish telemetry for the current loop to all available outputs.
     * <p>
     * Telemetry verbosity is controlled by TelemetrySettings.LEVEL:
     * - MATCH: Minimal telemetry, no dashboard
     * - DEBUG: Full telemetry with dashboard
     * </p>
     */
    public void publishLoopTelemetry(
            DriveSubsystem drive,
            LauncherSubsystem launcher,
            IntakeSubsystem intake,
            VisionSubsystemLimelight vision,
            LightingSubsystem lighting,
            DriverBindings.DriveRequest driveRequest,
            Gamepad gamepad1,
            Gamepad gamepad2,
            Alliance alliance,
            double runtimeSec,
            double matchTimeSec,
            Telemetry dsTelemetry,
            String opMode,
            boolean isAutonomous,
            Pose poseOverride,
            double prevMainLoopMs,
            long telemetryStartNs
    ) {
        // Capture all robot telemetry data once
        RobotTelemetryData data = RobotTelemetryData.capture(
                drive,
                launcher,
                intake,
                vision,
                lighting,
                driveRequest,
                gamepad1,
                gamepad2,
                alliance,
                runtimeSec,
                matchTimeSec,
                opMode,
                isAutonomous,
                poseOverride,
                prevMainLoopMs,
                telemetryStartNs
        );

        // FTC Dashboard packets (only in DEBUG mode)
        publishDashboardTelemetry(data);

        // Driver station telemetry (skip if autonomous and driver controls hidden)
        if (dsTelemetry != null && !isAutonomous) {
            publishDriverStationTelemetry(dsTelemetry, data);
        }
    }

    /**
     * Publish driver station telemetry based on level.
     */
    private void publishDriverStationTelemetry(Telemetry telemetry, RobotTelemetryData data) {
        if (TelemetrySettings.LEVEL == TelemetrySettings.TelemetryLevel.MATCH) {
            driverStationFormatter.publishMatch(telemetry, data);
        } else {
            // DEBUG mode - handle page navigation from driver gamepad dpad
            driverStationFormatter.handlePageNavigation(
                    data.gamepad.driver.dpadUp,
                    data.gamepad.driver.dpadDown
            );
            driverStationFormatter.publishDebug(telemetry, data);
        }
    }

    /**
     * Publish FTC Dashboard telemetry (only in DEBUG mode).
     */
    private void publishDashboardTelemetry(RobotTelemetryData data) {
        if (!TelemetrySettings.shouldSendDashboardPackets()) {
            return;
        }

        // Lazily initialize dashboard only when we actually need to send
        FtcDashboard dash = getDashboardIfNeeded();
        if (dash == null) {
            return;
        }

        long nowMs = SystemClock.uptimeMillis();

        // Throttle to 20 Hz
        if (nowMs - lastDashboardPacketMs >= DASHBOARD_INTERVAL_MS) {
            TelemetryPacket packet = dashboardFormatter.createPacket(data, TelemetrySettings.LEVEL);
            if (packet != null) {
                dash.sendTelemetryPacket(packet);
                RobotState.packet = new TelemetryPacket();
                lastDashboardPacketMs = nowMs;
            }
        }
    }

    /**
     * Safely get FTC Dashboard instance (returns null if unavailable).
     */
    private FtcDashboard safelyGetDashboard() {
        try {
            return FtcDashboard.getInstance();
        } catch (Throwable ignored) {
            return null;
        }
    }

    /**
     * @deprecated Legacy method for compatibility. Does nothing.
     */
    @Deprecated
    public void updateDriverStation(Telemetry telemetry) {
        // No-op: driver station updates are now handled directly in publishLoopTelemetry
    }

    /**
     * @deprecated Legacy method for autonomous routine step tracking. Will be re-added if needed.
     */
    @Deprecated
    public void setRoutineStepTelemetry(String stepName, double stepOrdinal) {
        // TODO: Re-implement if autonomous routine step tracking is needed
    }
}
