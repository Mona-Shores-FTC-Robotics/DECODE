package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Configuration for reverse flywheel intake mode.
 * Used when loading game pieces from human player station.
 */
@Config
public class LauncherReverseIntakeConfig {
    /** Power level for reverse intake (negative runs motors backward) */
    public static double reversePower = -0.7;

    /**
     * RPM threshold at which hood retracts for human loading.
     * Each lane's hood retracts independently when its flywheel reaches this speed.
     * This allows graceful degradation - if one lane is jammed, other lanes still work.
     * Set to 0 to disable speed-gating (hoods retract immediately like before).
     */
    public static double reverseRpmThreshold = 500;
}
