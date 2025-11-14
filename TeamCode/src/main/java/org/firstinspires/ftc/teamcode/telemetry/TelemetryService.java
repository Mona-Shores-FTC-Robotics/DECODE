package org.firstinspires.ftc.teamcode.telemetry;

import android.os.SystemClock;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherCoordinator;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.telemetry.data.RobotTelemetryData;
import org.firstinspires.ftc.teamcode.telemetry.formatters.DashboardFormatter;
import org.firstinspires.ftc.teamcode.telemetry.formatters.DriverStationFormatter;
import org.firstinspires.ftc.teamcode.telemetry.formatters.FullPanelsFormatter;
import org.firstinspires.ftc.teamcode.util.Alliance;

/**
 * Centralized telemetry service for DECODE robot.
 * <p>
 * Provides tiered telemetry output (MATCH/PRACTICE/DEBUG) to multiple destinations:
 * - Driver Station (FTC telemetry)
 * - FTC Dashboard (web dashboard + AdvantageScope)
 * - FullPanels (FTControl panels)
 * </p>
 * <p>
 * This service acts as a thin orchestrator - all formatting logic is delegated to
 * specialized formatters in the telemetry.formatters package.
 * </p>
 */
public class TelemetryService {

    private final FtcDashboard dashboard;
    private final DriverStationFormatter driverStationFormatter;
    private final DashboardFormatter dashboardFormatter;
    private final FullPanelsFormatter fullPanelsFormatter;

    private TelemetryManager panelsTelemetry;
    private boolean sessionActive = false;
    private long lastDashboardPacketMs = 0L;

    /**
     * Create a new telemetry service.
     */
    public TelemetryService() {
        this.dashboard = TelemetrySettings.enableDashboardTelemetry ? safelyGetDashboard() : null;
        this.driverStationFormatter = new DriverStationFormatter();
        this.dashboardFormatter = new DashboardFormatter();
        this.fullPanelsFormatter = new FullPanelsFormatter();
    }

    /**
     * Prepares FullPanels for telemetry. Call once when the OpMode starts.
     */
    public void startSession() {
        if (!sessionActive) {
            panelsTelemetry = PanelsBridge.preparePanels();
            sessionActive = true;
        }
    }

    /**
     * Stops the telemetry session and flushes resources.
     */
    public void stopSession() {
        if (sessionActive) {
            panelsTelemetry = null;
            sessionActive = false;
        }
    }

    /**
     * @deprecated Use data models and formatters directly. Legacy accessor for compatibility.
     */
    @Deprecated
    public TelemetryPublisher publisher() {
        return new TelemetryPublisher(panelsTelemetry);
    }

    /**
     * @deprecated Access panelsTelemetry through formatters. Legacy accessor for compatibility.
     */
    @Deprecated
    public TelemetryManager panelsTelemetry() {
        return panelsTelemetry;
    }

    /**
     * Publish telemetry for the current loop to all available outputs.
     * <p>
     * Telemetry verbosity is controlled by TelemetrySettings.config.level:
     * - MATCH: Minimal telemetry (<10ms target)
     * - PRACTICE: Moderate telemetry (<20ms target)
     * - DEBUG: Full telemetry (all diagnostics)
     * </p>
     *
     * @param drive Drive subsystem
     * @param launcher Launcher subsystem
     * @param intake Intake subsystem
     * @param vision Vision subsystem
     * @param launcherCoordinator Launcher coordinator (for artifact tracking)
     * @param driveRequest Current drive request from bindings (may be null)
     * @param gamepad1 Driver gamepad (may be null)
     * @param gamepad2 Operator gamepad (may be null)
     * @param alliance Current alliance
     * @param runtimeSec OpMode runtime in seconds
     * @param matchTimeSec Match time remaining in seconds
     * @param dsTelemetry Driver station telemetry object (may be null)
     * @param opMode OpMode name (e.g., "TeleOp", "Autonomous")
     * @param isAutonomous Whether this is autonomous mode (hides driver controls)
     * @param poseOverride Optional pose override for autonomous routines (may be null)
     */
    public void publishLoopTelemetry(
            DriveSubsystem drive,
            LauncherSubsystem launcher,
            IntakeSubsystem intake,
            VisionSubsystemLimelight vision,
            LauncherCoordinator launcherCoordinator,
            DriverBindings.DriveRequest driveRequest,
            Gamepad gamepad1,
            Gamepad gamepad2,
            Alliance alliance,
            double runtimeSec,
            double matchTimeSec,
            Telemetry dsTelemetry,
            String opMode,
            boolean isAutonomous,
            Pose poseOverride
    ) {
        // Capture all robot telemetry data once
        RobotTelemetryData data = RobotTelemetryData.capture(
                drive,
                launcher,
                intake,
                vision,
                launcherCoordinator,
                driveRequest,
                gamepad1,
                gamepad2,
                alliance,
                runtimeSec,
                matchTimeSec,
                opMode,
                isAutonomous,
                poseOverride
        );

        TelemetrySettings.TelemetryLevel level = TelemetrySettings.config.level;

        // 1. Driver station telemetry (skip if autonomous and driver controls hidden)
        if (dsTelemetry != null && !isAutonomous) {
            publishDriverStationTelemetry(dsTelemetry, data, level);
        }

        // 2. FTC Dashboard packets (throttled based on level)
        publishDashboardTelemetry(data, level);

        // 3. FullPanels telemetry (DEBUG mode only)
        if (TelemetrySettings.shouldSendFullPanels() && panelsTelemetry != null) {
            fullPanelsFormatter.publish(panelsTelemetry, data);
        }
    }

    /**
     * Overload for backward compatibility with existing OpMode calls.
     * Adds intake parameter and calculates matchTimeSec automatically.
     */
    public void publishLoopTelemetry(
            DriveSubsystem drive,
            LauncherSubsystem launcher,
            IntakeSubsystem intake,
            VisionSubsystemLimelight vision,
            DriverBindings.DriveRequest driveRequest,
            LauncherCoordinator launcherCoordinator,
            Alliance alliance,
            double runtimeSec,
            Telemetry dsTelemetry,
            String opMode,
            boolean suppressDriveTelemetry,
            Pose poseOverride
    ) {
        // Calculate match time based on opMode
        double matchTimeSec;
        boolean isAutonomous;
        if (opMode != null && opMode.toLowerCase().contains("auto")) {
            matchTimeSec = Math.max(0.0, 30.0 - runtimeSec);
            isAutonomous = true;
        } else {
            matchTimeSec = Math.max(0.0, 150.0 - runtimeSec);
            isAutonomous = suppressDriveTelemetry;
        }

        publishLoopTelemetry(
                drive,
                launcher,
                intake,
                vision,
                launcherCoordinator,
                driveRequest,
                null,  // gamepad1 (not available in old signature)
                null,  // gamepad2 (not available in old signature)
                alliance,
                runtimeSec,
                matchTimeSec,
                dsTelemetry,
                opMode,
                isAutonomous,
                poseOverride
        );
    }

    /**
     * Publish driver station telemetry based on level.
     */
    private void publishDriverStationTelemetry(Telemetry telemetry, RobotTelemetryData data, TelemetrySettings.TelemetryLevel level) {
        switch (level) {
            case MATCH:
                driverStationFormatter.publishMatch(telemetry, data);
                break;
            case PRACTICE:
                driverStationFormatter.publishPractice(telemetry, data);
                break;
            case DEBUG:
            default:
                // Handle page navigation from driver gamepad dpad
                driverStationFormatter.handlePageNavigation(
                        data.gamepad.driver.dpadUp,
                        data.gamepad.driver.dpadDown
                );
                driverStationFormatter.publishDebug(telemetry, data);
                break;
        }
    }

    /**
     * Publish FTC Dashboard telemetry based on level (with throttling).
     */
    private void publishDashboardTelemetry(RobotTelemetryData data, TelemetrySettings.TelemetryLevel level) {
        if (!TelemetrySettings.shouldSendDashboardPackets() || dashboard == null) {
            return;
        }

        long nowMs = SystemClock.uptimeMillis();
        long dashboardInterval = TelemetrySettings.getDashboardInterval();

        // Throttle based on configured interval
        if (dashboardInterval > 0 && (nowMs - lastDashboardPacketMs >= dashboardInterval)) {
            TelemetryPacket packet = dashboardFormatter.createPacket(data, level);
            if (packet != null) {
                dashboard.sendTelemetryPacket(packet);
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
