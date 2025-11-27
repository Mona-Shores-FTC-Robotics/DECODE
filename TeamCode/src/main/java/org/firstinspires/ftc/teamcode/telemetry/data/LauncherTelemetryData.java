package org.firstinspires.ftc.teamcode.telemetry.data;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Launcher subsystem telemetry data.
 * Per-lane design: Each lane is independent. Check individual lane readiness
 * rather than looking for a global "ready" state.
 *
 * Simplified design: Direct RPM control via feedforward + optional proportional feedback.
 * No SpinMode or ControlMode - just target RPM per lane.
 */
public class LauncherTelemetryData {
    // Per-lane data (each lane independent)
    public final LaneData left;
    public final LaneData center;
    public final LaneData right;

    public LauncherTelemetryData(
            LaneData left,
            LaneData center,
            LaneData right
    ) {
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
                LaneData.capture(launcher, LauncherLane.LEFT),
                LaneData.capture(launcher, LauncherLane.CENTER),
                LaneData.capture(launcher, LauncherLane.RIGHT)
        );
    }
}
