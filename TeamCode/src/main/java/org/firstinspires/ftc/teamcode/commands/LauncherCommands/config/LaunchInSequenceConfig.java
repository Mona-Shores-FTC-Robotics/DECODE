package org.firstinspires.ftc.teamcode.commands.LauncherCommands.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Configuration for LaunchInSequenceCommand.
 * Controls timing and RPM values for pattern-based sequential firing.
 */
@Configurable
public class LaunchInSequenceConfig {
    /** Milliseconds between shots in sequence */
    public double shotSpacingMs = 1000;

    /** RPM for left lane in sequence mode */
    public double sequenceLeftRpm = 2550;

    /** RPM for center lane in sequence mode */
    public double sequenceCenterRpm = 2550;

    /** RPM for right lane in sequence mode */
    public double sequenceRightRpm = 2550;

    /** Timeout in seconds before giving up on spin-up */
    public double timeoutSeconds = 8.0;
}
