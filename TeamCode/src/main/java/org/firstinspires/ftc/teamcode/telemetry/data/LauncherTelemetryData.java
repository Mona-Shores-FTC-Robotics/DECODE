package org.firstinspires.ftc.teamcode.telemetry.data;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Launcher subsystem telemetry data.
 * Per-lane design: Each lane is independent. Check individual lane readiness
 * rather than looking for a global "ready" state.
 */
public class LauncherTelemetryData {
    // Global launcher config
    public final String spinMode;
    public final String controlMode;

    // Per-lane data (each lane independent)
    public final LaneData left;
    public final LaneData center;
    public final LaneData right;

    public LauncherTelemetryData(
            String spinMode,
            String controlMode,
            LaneData left,
            LaneData center,
            LaneData right
    ) {
        this.spinMode = spinMode;
        this.controlMode = controlMode;
        this.left = left;
        this.center = center;
        this.right = right;
    }

    /**
     * Per-lane launcher telemetry data.
     */
    public static class LaneData {
        // Flywheel
        public final double targetRpm;
        public final double currentRpm;
        public final double power;
        public final boolean ready;

        // Control phase
        public final String phase;
        public final boolean phaseBang;
        public final boolean phaseHold;
        public final boolean phaseHybrid;
        public final int bangToHoldCount;

        // Hood position
        public final double hoodPosition;

        // Feeder position
        public final double feederPosition;

        public LaneData(
                double targetRpm,
                double currentRpm,
                double power,
                boolean ready,
                String phase,
                int bangToHoldCount,
                double hoodPosition,
                double feederPosition
        ) {
            this.targetRpm = targetRpm;
            this.currentRpm = currentRpm;
            this.power = power;
            this.ready = ready;
            this.phase = phase;
            this.phaseBang = "BANG".equals(phase);
            this.phaseHold = "HOLD".equals(phase);
            this.phaseHybrid = "HYBRID".equals(phase);
            this.bangToHoldCount = bangToHoldCount;
            this.hoodPosition = hoodPosition;
            this.feederPosition = feederPosition;
        }

        public static LaneData capture(LauncherSubsystem launcher, LauncherLane lane) {
            return new LaneData(
                    launcher.getTargetRpm(lane),
                    launcher.getCurrentRpm(lane),
                    launcher.getLastPower(lane),
                    launcher.isLaneReady(lane),
                    launcher.getPhaseName(lane),
                    launcher.getBangToHoldCount(lane),
                    launcher.getHoodPosition(lane),
                    launcher.getFeederPosition(lane)
            );
        }
    }

    public static LauncherTelemetryData capture(LauncherSubsystem launcher) {
        return new LauncherTelemetryData(
                launcher.getEffectiveSpinMode().name(),
                LauncherSubsystem.getFlywheelControlMode().name(),
                LaneData.capture(launcher, LauncherLane.LEFT),
                LaneData.capture(launcher, LauncherLane.CENTER),
                LaneData.capture(launcher, LauncherLane.RIGHT)
        );
    }
}
