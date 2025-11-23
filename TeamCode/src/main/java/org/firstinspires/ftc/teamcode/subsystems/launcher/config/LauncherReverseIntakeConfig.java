package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Configuration for reverse flywheel intake mode.
 * Used when loading game pieces from human player station.
 */
@Configurable
public class LauncherReverseIntakeConfig {
    /** Power level for reverse intake (negative runs motors backward) */
    public double reversePower = -0.7;
}
