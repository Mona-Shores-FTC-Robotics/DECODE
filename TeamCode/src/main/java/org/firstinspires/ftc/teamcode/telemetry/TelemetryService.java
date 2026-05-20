package org.firstinspires.ftc.teamcode.telemetry;

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

    private FtcDashboard dashboard;
    private boolean dashboardInitialized = false;
    private final DriverStationFormatter driverStationFormatter;
    private final DashboardFormatter dashboardFormatter;

    private boolean sessionActive = false;

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
     * Prepares telemetry outputs. Call once when the OpMode starts.
     * Sends a seed packet so all Dashboard channels appear immediately
     * without needing to run through init/play/commands first.
     */
    public void startSession() {
        if (!sessionActive) {
            sessionActive = true;
            sendSeedPacket();
        }
    }

    private void sendSeedPacket() {
        if (!TelemetrySettings.shouldSendDashboardPackets()) {
            return;
        }
        FtcDashboard dash = getDashboardIfNeeded();
        if (dash == null) {
            return;
        }
        TelemetryPacket seed = dashboardFormatter.createSeedPacket(TelemetrySettings.LEVEL);
        if (seed != null) {
            dash.sendTelemetryPacket(seed);
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

        // FTC Dashboard packets (only in PRACTICE/VERBOSE mode)
        publishDashboardTelemetry(data);

        // Driver station telemetry (skip if autonomous and driver controls hidden)
        if (dsTelemetry != null && !isAutonomous) {
            publishDriverStationTelemetry(dsTelemetry, data);
        }
    }

    /**
     * Publish driver station telemetry.
     * Always sends match-level text to the DS telemetry object. In PRACTICE/VERBOSE
     * mode the Dashboard packet carries all detailed data, so forwarding the full
     * 7-page debug text to the DS object would flood the Dashboard Telemetry panel.
     */
    private void publishDriverStationTelemetry(Telemetry telemetry, RobotTelemetryData data) {
        driverStationFormatter.publishMatch(telemetry, data);
    }

    /**
     * Publish FTC Dashboard telemetry (only in PRACTICE/VERBOSE mode).
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

        TelemetryPacket packet = dashboardFormatter.createPacket(data, TelemetrySettings.LEVEL);
        if (packet != null) {
            dash.sendTelemetryPacket(packet);
            RobotState.packet = new TelemetryPacket();
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

}
