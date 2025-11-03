package org.firstinspires.ftc.teamcode.bindings;

import org.firstinspires.ftc.teamcode.util.LauncherLane;

public interface OperatorControls {

    void update(double reverseTrigger, double forwardTrigger);

    void reset();

    boolean isShooterDebugMode();

    LaneDebugState getLaneDebugState(LauncherLane lane);

    final class LaneDebugState {
        public boolean enabled;
        public double targetRpm;

        public LaneDebugState(double targetRpm) {
            this.targetRpm = Math.max(0.0, targetRpm);
            this.enabled = false;
        }
    }
}
