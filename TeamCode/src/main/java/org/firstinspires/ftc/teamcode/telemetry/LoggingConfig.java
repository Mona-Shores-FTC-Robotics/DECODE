package org.firstinspires.ftc.teamcode.telemetry;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Central configuration for KoalaLog logging levels.
 * Controls which @AutoLogOutput methods are sampled to balance performance vs diagnostics.
 */
@Configurable
public class LoggingConfig {

    /**
     * Logging tiers define what data is sampled and logged.
     *
     * MINIMAL: Only critical data for AdvantageScope replay (~3ms overhead)
     *   - Robot pose, status, subsystem states
     *
     * MATCH: Essential match data for post-game analysis (~8ms overhead)
     *   - Motor commands, launcher RPM, vision tags, artifact states
     *   - Good for competition use
     *
     * DIAGNOSTIC: Full detailed logging for tuning (~40ms overhead)
     *   - Motor currents, velocities, all vision data, all launcher details
     *   - Use for testing and tuning sessions only
     */
    public enum LoggingTier {
        MINIMAL,
        MATCH,
        DIAGNOSTIC
    }

    /**
     * Active logging tier - change via FTC Dashboard Config tab during init.
     */
    @Configurable
    public static class ActiveTier {
        public static LoggingTier tier = LoggingTier.MATCH;
    }

    /**
     * Individual feature toggles for fine-grained control.
     * These override the tier defaults when set to true.
     */
    @Configurable
    public static class Features {
        /** Log motor current draws (expensive: ~20ms for 4 motors) */
        public static boolean logMotorCurrents = false;

        /** Log motor velocities (moderate: ~8ms for 4 motors) */
        public static boolean logMotorVelocities = false;

        /** Log detailed per-lane launcher data (moderate: ~5ms) */
        public static boolean logPerLaneLauncher = false;

        /** Log detailed vision targeting data (tx, ty, ta, range, bearing) */
        public static boolean logDetailedVision = false;

        /** Log per-lane intake sensor HSV/distance data (moderate: ~5ms) */
        public static boolean logPerLaneIntake = false;

        /** Log PoseFusion diagnostics (moderate: ~3ms) */
        public static boolean logPoseFusion = false;

        /** Log subsystem periodic timing breakdowns */
        public static boolean logSubsystemTimings = false;

        /** Log control loop phases and counters */
        public static boolean logControlPhases = false;
    }

    // ========================================================================
    // Tier Check Helpers
    // ========================================================================

    public static boolean isMinimal() {
        return ActiveTier.tier == LoggingTier.MINIMAL;
    }

    public static boolean isMatch() {
        return ActiveTier.tier == LoggingTier.MATCH;
    }

    public static boolean isDiagnostic() {
        return ActiveTier.tier == LoggingTier.DIAGNOSTIC;
    }

    public static boolean isMatchOrHigher() {
        return ActiveTier.tier == LoggingTier.MATCH || ActiveTier.tier == LoggingTier.DIAGNOSTIC;
    }

    public static boolean isDiagnosticOrForced(boolean featureFlag) {
        return isDiagnostic() || featureFlag;
    }

    // ========================================================================
    // Feature Check Helpers
    // ========================================================================

    public static boolean shouldLogMotorCurrents() {
        return isDiagnostic() || Features.logMotorCurrents;
    }

    public static boolean shouldLogMotorVelocities() {
        return isDiagnostic() || Features.logMotorVelocities;
    }

    public static boolean shouldLogPerLaneLauncher() {
        return isDiagnostic() || Features.logPerLaneLauncher;
    }

    public static boolean shouldLogDetailedVision() {
        return isDiagnostic() || Features.logDetailedVision;
    }

    public static boolean shouldLogPerLaneIntake() {
        return isDiagnostic() || Features.logPerLaneIntake;
    }

    public static boolean shouldLogPoseFusion() {
        return isDiagnostic() || Features.logPoseFusion;
    }

    public static boolean shouldLogSubsystemTimings() {
        return isDiagnostic() || Features.logSubsystemTimings;
    }

    public static boolean shouldLogControlPhases() {
        return isDiagnostic() || Features.logControlPhases;
    }
}
