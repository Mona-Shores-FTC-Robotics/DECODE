package org.firstinspires.ftc.teamcode.telemetry;

public final class TelemetrySettings {
    private TelemetrySettings() {}

    /**
     * Telemetry verbosity levels for different operational contexts.
     */
    public enum TelemetryLevel {
        /**
         * MATCH: Minimal telemetry for competition matches.
         * - Target: <10ms overhead
         * - Critical pose logging for AdvantageScope replay
         * - Essential driver station info only
         * - No FTC Dashboard packets
         * - No FullPanels telemetry
         */
        MATCH,

        /**
         * PRACTICE: Moderate telemetry for practice sessions and tuning.
         * - Target: <20ms overhead
         * - Basic FTC Dashboard metrics
         * - High-level subsystem state
         * - Suitable for real-time tuning
         */
        PRACTICE,

        /**
         * DEBUG: Full telemetry for development and detailed diagnostics.
         * - Current behavior (all logging enabled)
         * - All @AutoLogOutput methods
         * - Complete dashboard packets
         * - Full FullPanels telemetry
         */
        DEBUG
    }

    /**
     * Telemetry level for this build. Change and recompile to switch modes.
     *
     * MATCH (default): No Dashboard, no packets - safe for competition
     * PRACTICE: Dashboard + throttled packets - for tuning on practice field
     * DEBUG: Full telemetry - for development/pit testing
     */
    public static final TelemetryLevel LEVEL = TelemetryLevel.MATCH;

    public static class Config {
        /** Runtime level override - only works if LEVEL is not MATCH (Dashboard must be running) */
        public TelemetryLevel level = LEVEL;

        /** Enable FTC Dashboard packet sending (overridden by level) */
        public boolean enableDashboardPackets = true;

        /** Enable FullPanels telemetry (overridden by level) */
        public boolean enableFullPanels = false;

        /** AutoLogManager sampling interval in milliseconds */
        public long autoLogIntervalMs = 50L;

        /** FTC Dashboard packet sending interval in milliseconds */
        public long dashboardIntervalMs = 50L;

        /** Driver station telemetry update interval in milliseconds */
        public long driverStationIntervalMs = 200L;
    }

    public static Config config = new Config();

    /** Legacy field - use config.level instead */
    @Deprecated
    public static boolean enableDashboardTelemetry = true;

    /**
     * Returns true if FTC Dashboard should be initialized.
     * Dashboard only starts in PRACTICE or DEBUG mode - never in MATCH.
     */
    public static boolean shouldInitializeDashboard() {
        return LEVEL != TelemetryLevel.MATCH;
    }

    /**
     * Get effective dashboard packet sending state based on telemetry level.
     */
    public static boolean shouldSendDashboardPackets() {
        if (LEVEL == TelemetryLevel.MATCH) {
            return false;
        }
        return config.enableDashboardPackets;
    }

    /**
     * Get effective FullPanels telemetry state based on telemetry level.
     */
    public static boolean shouldSendFullPanels() {
        if (LEVEL == TelemetryLevel.MATCH) {
            return false;
        }
        if (config.level == TelemetryLevel.PRACTICE) {
            return config.enableFullPanels;
        }
        return true; // Always send in DEBUG
    }

    /**
     * Get effective AutoLogManager interval based on telemetry level.
     */
    public static long getAutoLogInterval() {
        switch (config.level) {
            case MATCH:
                return 100L; // 10 Hz
            case PRACTICE:
                return 50L;  // 20 Hz
            case DEBUG:
            default:
                return config.autoLogIntervalMs;
        }
    }

    /**
     * Get effective dashboard packet interval based on telemetry level.
     */
    public static long getDashboardInterval() {
        switch (LEVEL) {
            case MATCH:
                return 0L; // Disabled
            case PRACTICE:
                return 100L; // 10 Hz
            case DEBUG:
            default:
                return config.dashboardIntervalMs;
        }
    }
}
