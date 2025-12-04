package org.firstinspires.ftc.teamcode.commands.LauncherCommands.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Configuration for LaunchInSequenceCommand.
 * Controls timing for pattern-based sequential firing.
 */
@Configurable
public class LaunchInSequenceConfig {
    /** Milliseconds between shots in sequence */
    public double shotSpacingMs = 525;

    /** Timeout in seconds before giving up on spin-up */
    public double timeoutSeconds = 4.0;
}
