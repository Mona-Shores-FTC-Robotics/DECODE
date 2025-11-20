package org.firstinspires.ftc.teamcode.telemetry.data;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;

/**
 * Intake subsystem telemetry data.
 * Includes mode, power, roller state, and artifact tracking.
 */
public class IntakeTelemetryData {
    // Intake state
    public final String mode;
    public final String resolvedMode;
    public final double power;

    // Roller
    public final boolean rollerPresent;
    public final boolean rollerActive;
    public final double rollerPosition;
    public final boolean gatePresent;
    public final double gatePosition;

    // Artifact tracking
    public final int artifactCount;
    public final String artifactState;

    // Lane sampling
    public final Map<LauncherLane, IntakeSubsystem.LaneSample> laneSamples;
    public final LaneTelemetrySummary laneLeftSummary;
    public final LaneTelemetrySummary laneCenterSummary;
    public final LaneTelemetrySummary laneRightSummary;

    public IntakeTelemetryData(
            String mode,
            String resolvedMode,
            double power,
            boolean rollerPresent,
            boolean rollerActive,
            double rollerPosition,
            boolean gatePresent,
            double gatePosition,
            int artifactCount,
            String artifactState,
            Map<LauncherLane, IntakeSubsystem.LaneSample> laneSamples,
            LaneTelemetrySummary laneLeftSummary,
            LaneTelemetrySummary laneCenterSummary,
            LaneTelemetrySummary laneRightSummary
    ) {
        this.mode = mode;
        this.resolvedMode = resolvedMode;
        this.power = power;
        this.rollerPresent = rollerPresent;
        this.rollerActive = rollerActive;
        this.rollerPosition = rollerPosition;
        this.gatePresent = gatePresent;
        this.gatePosition = gatePosition;
        this.artifactCount = artifactCount;
        this.artifactState = artifactState;
        this.laneSamples = laneSamples;
        this.laneLeftSummary = laneLeftSummary;
        this.laneCenterSummary = laneCenterSummary;
        this.laneRightSummary = laneRightSummary;
    }

    public static IntakeTelemetryData capture(IntakeSubsystem intake) {
        if (intake == null) {
            throw new IllegalArgumentException("Intake subsystem is required for telemetry capture");
        }

        int artifactCount = intake.getArtifactCount();
        String artifactState = formatArtifactState(artifactCount);
        Map<LauncherLane, IntakeSubsystem.LaneSample> laneSamples =
                Collections.unmodifiableMap(new EnumMap<>(intake.getLaneSampleSnapshot()));

        LaneTelemetrySummary leftSummary = summarizeLane(laneSamples.get(LauncherLane.LEFT));
        LaneTelemetrySummary centerSummary = summarizeLane(laneSamples.get(LauncherLane.CENTER));
        LaneTelemetrySummary rightSummary = summarizeLane(laneSamples.get(LauncherLane.RIGHT));

        return new IntakeTelemetryData(
                intake.getMode().name(),
                intake.getResolvedMode().name(),
                intake.getCurrentPower(),
                intake.isRollerPresent(),
                intake.isRollerActive(),
                intake.getRollerPosition(),
                intake.isPrefeedPresent(),
                intake.getGatePosition(),
                artifactCount,
                artifactState,
                laneSamples,
                leftSummary,
                centerSummary,
                rightSummary
        );
    }

    private static String formatArtifactState(int count) {
        switch (count) {
            case 0: return "EMPTY";
            case 1: return "ONE";
            case 2: return "TWO";
            case 3: return "THREE";
            default: return "UNKNOWN";
        }
    }

    private static LaneTelemetrySummary summarizeLane(IntakeSubsystem.LaneSample sample) {
        if (sample == null) {
            return LaneTelemetrySummary.absent();
        }
        return LaneTelemetrySummary.fromSample(sample);
    }

    public static final class LaneTelemetrySummary {
        public final boolean sensorPresent;
        public final boolean detected;
        public final double distanceCm;
        public final String color;
        public final String hsvColor;

        private LaneTelemetrySummary(boolean sensorPresent,
                                     boolean detected,
                                     double distanceCm,
                                     String color,
                                     String hsvColor) {
            this.sensorPresent = sensorPresent;
            this.detected = detected;
            this.distanceCm = distanceCm;
            this.color = color;
            this.hsvColor = hsvColor;
        }

        static LaneTelemetrySummary fromSample(IntakeSubsystem.LaneSample sample) {
            return new LaneTelemetrySummary(
                    sample.sensorPresent,
                    sample.withinDistance,
                    sample.distanceCm,
                    sample.color.name(),
                    sample.hsvColor.name()
            );
        }

        static LaneTelemetrySummary absent() {
            return new LaneTelemetrySummary(false, false, Double.NaN, "NONE", "NONE");
        }
    }
}
