package org.firstinspires.ftc.teamcode.telemetry;

public final class TelemetrySettings {
    private TelemetrySettings() {}

    /**
     * Telemetry level - controls whether Dashboard starts and packets are sent.
     */
    public enum TelemetryLevel {
        /**
         * MATCH: Safe for competition.
         * - FTC Dashboard does NOT start
         * - No telemetry packets sent
         * - Driver station only: essential info
         */
        MATCH,

        /**
         * DEBUG: Full telemetry for tuning and development.
         * - FTC Dashboard starts
         * - Telemetry packets sent
         * - All diagnostics available
         */
        DEBUG
    }

    /**
     * Telemetry level for this build. Change and recompile to switch modes.
     *
     * MATCH (default): No Dashboard, no packets - safe for competition
     * DEBUG: Full telemetry - for tuning and development
     */
    public static final TelemetryLevel LEVEL = TelemetryLevel.MATCH;

    /**
     * Returns true if FTC Dashboard should be initialized.
     * Dashboard only starts in DEBUG mode - never in MATCH.
     */
    public static boolean shouldInitializeDashboard() {
        return LEVEL == TelemetryLevel.DEBUG;
    }

    /**
     * Returns true if telemetry packets should be sent.
     */
    public static boolean shouldSendDashboardPackets() {
        return LEVEL == TelemetryLevel.DEBUG;
    }

    /**
     * Returns true if FullPanels telemetry should be sent.
     */
    public static boolean shouldSendFullPanels() {
        return LEVEL == TelemetryLevel.DEBUG;
    }
}
