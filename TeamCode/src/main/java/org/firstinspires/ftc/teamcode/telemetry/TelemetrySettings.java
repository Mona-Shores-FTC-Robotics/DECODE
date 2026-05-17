package org.firstinspires.ftc.teamcode.telemetry;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class TelemetrySettings {
    private TelemetrySettings() {}

    public enum TelemetryLevel {
        /** Competition: no Dashboard, no packets, driver station only. */
        MATCH,
        /** Practice/tuning: lean ~25-field packet + field overlay at 20 Hz. */
        PRACTICE,
        /** Full diagnostics: every subsystem field, color sensors, gamepad axes. */
        VERBOSE
    }

    /** Change via FTC Dashboard Config tab — no recompile needed. */
    public static TelemetryLevel LEVEL = TelemetryLevel.PRACTICE;

    public static boolean shouldInitializeDashboard() {
        return LEVEL != TelemetryLevel.MATCH;
    }

    public static boolean shouldSendDashboardPackets() {
        return LEVEL != TelemetryLevel.MATCH;
    }

    /** True only in VERBOSE mode — guards expensive/high-volume packet.put calls. */
    public static boolean isVerbose() {
        return LEVEL == TelemetryLevel.VERBOSE;
    }
}
