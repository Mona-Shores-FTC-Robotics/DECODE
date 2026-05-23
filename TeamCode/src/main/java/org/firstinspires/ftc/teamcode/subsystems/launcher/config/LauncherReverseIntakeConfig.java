package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Configuration for reverse flywheel intake mode.
 * Used when loading game pieces from human player station.
 */
@Configurable
public class LauncherReverseIntakeConfig {
    /** Power level for reverse intake (negative runs motors backward) */
    public static double reversePower = -.75;

    /**
     * RPM threshold at which hood retracts for human loading.
     * Each lane's hood retracts independently when its flywheel reaches this speed.
     * This allows graceful degradation - if one lane is jammed, other lanes still work.
     * Set to 0 (or any value <= 0) to retract all hoods immediately when human loading starts.
     */
    public static double reverseRpmThreshold = 0;
}
