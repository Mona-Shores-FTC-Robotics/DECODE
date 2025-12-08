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
     * Set to true for competition builds to completely disable all dashboard/telemetry overhead.
     * When true, FTC Dashboard server won't start, no packets sent, minimal CPU usage.
     * Change this to false when you need to tune on the practice field.
     */
    public static final boolean COMPETITION_MODE = false;

    public static class Config {
        /** Active telemetry level - change via FTC Dashboard (ignored if COMPETITION_MODE=true) */
        public TelemetryLevel level = TelemetryLevel.MATCH;

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
     * Dashboard is needed for changing settings at runtime, so it starts unless
     * COMPETITION_MODE is true (which requires recompile to change).
     */
    public static boolean shouldInitializeDashboard() {
        // Only COMPETITION_MODE completely disables Dashboard
        // In MATCH mode, Dashboard still starts (for config access) but no packets are sent
        return !COMPETITION_MODE;
    }

    /**
     * Get effective dashboard packet sending state based on telemetry level.
     */
    public static boolean shouldSendDashboardPackets() {
        if (COMPETITION_MODE || config.level == TelemetryLevel.MATCH) {
            return false; // Never send in MATCH mode or competition builds
        }
        return config.enableDashboardPackets;
    }

    /**
     * Get effective FullPanels telemetry state based on telemetry level.
     */
    public static boolean shouldSendFullPanels() {
        if (COMPETITION_MODE || config.level == TelemetryLevel.MATCH) {
            return false; // Never send in MATCH mode or competition builds
        }
        if (config.level == TelemetryLevel.PRACTICE) {
            return config.enableFullPanels; // Configurable in PRACTICE
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
        switch (config.level) {
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
