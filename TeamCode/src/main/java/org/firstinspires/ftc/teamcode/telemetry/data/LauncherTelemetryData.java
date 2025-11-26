package org.firstinspires.ftc.teamcode.telemetry.data;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Launcher subsystem telemetry data.
 * Includes per-lane flywheel, hood, and feeder data.
 */
public class LauncherTelemetryData {
    // High-level launcher state
    public final boolean ready;
    public final String state;
    public final String spinMode;

    // Per-lane data
    public final LaneData left;
    public final LaneData center;
    public final LaneData right;

    public LauncherTelemetryData(
            boolean ready,
            String state,
            String spinMode,
            LaneData left,
            LaneData center,
            LaneData right
    ) {
        this.ready = ready;
        this.state = state;
        this.spinMode = spinMode;
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

        // Hood position
        public final double hoodPosition;

        // Feeder position
        public final double feederPosition;

        public LaneData(
                double targetRpm,
                double currentRpm,
                double power,
                boolean ready,
                double hoodPosition,
                double feederPosition
        ) {
            this.targetRpm = targetRpm;
            this.currentRpm = currentRpm;
            this.power = power;
            this.ready = ready;
            this.hoodPosition = hoodPosition;
            this.feederPosition = feederPosition;
        }

        public static LaneData capture(LauncherSubsystem launcher, LauncherLane lane) {
            return new LaneData(
                    launcher.getTargetRpm(lane),
                    launcher.getCurrentRpm(lane),
                    launcher.getLastPower(lane),
                    launcher.isLaneReady(lane),
                    launcher.getHoodPosition(lane),
                    launcher.getFeederPosition(lane)
            );
        }
    }

    public static LauncherTelemetryData capture(LauncherSubsystem launcher) {
        return new LauncherTelemetryData(
                launcher.atTarget(),
                launcher.getState().name(),
                launcher.getEffectiveSpinMode().name(),
                LaneData.capture(launcher, LauncherLane.LEFT),
                LaneData.capture(launcher, LauncherLane.CENTER),
                LaneData.capture(launcher, LauncherLane.RIGHT)
        );
    }
}
