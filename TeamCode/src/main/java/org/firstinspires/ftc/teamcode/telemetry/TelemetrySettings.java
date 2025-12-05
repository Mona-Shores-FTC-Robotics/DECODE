package org.firstinspires.ftc.teamcode.telemetry;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
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

    @Configurable
    public static class Config {
        /** Active telemetry level - change via FTC Dashboard */
        public TelemetryLevel level = TelemetryLevel.MATCH;

        /**
         * KILL SWITCH: Set to false to completely disable all Panels debug() calls.
         * Use this if Panels is slow to connect or causing performance issues.
         * This overrides all other settings.
         */
        public boolean enablePanelsDebugCalls = true;

        /** Enable FTC Dashboard packet sending (overridden by level) */
        public boolean enableDashboardPackets = true;

        /** Enable FullPanels telemetry (overridden by level) */
        public boolean enableFullPanels = true;

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
     * Get effective dashboard packet sending state based on telemetry level.
     */
    public static boolean shouldSendDashboardPackets() {
        if (config.level == TelemetryLevel.MATCH) {
            return false; // Never send in MATCH mode
        }
        return config.enableDashboardPackets;
    }

    /**
     * Get effective FullPanels telemetry state based on telemetry level.
     * Check enablePanelsDebugCalls first as a kill switch.
     */
    public static boolean shouldSendFullPanels() {
        // Kill switch - if disabled, never send
        if (!config.enablePanelsDebugCalls) {
            return false;
        }
        if (config.level == TelemetryLevel.MATCH) {
            return false; // Never send in MATCH mode
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
