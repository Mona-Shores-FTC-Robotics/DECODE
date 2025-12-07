package org.firstinspires.ftc.teamcode.commands.LauncherCommands.config;

/**
 * Configuration for LaunchInSequenceCommand.
 * Controls timing for pattern-based sequential firing.
 */
public class LaunchInSequenceConfig {
    /** Milliseconds between shots in sequence */
    public double shotSpacingMs = 525;

    /** Timeout in seconds before giving up on spin-up */
    public double timeoutSeconds = 4.0;
}
