package org.firstinspires.ftc.teamcode.commands.LauncherCommands.config;

/**
 * Configuration for LaunchInSequenceCommand. Controls timing for pattern-based
 * sequential firing. Discovered by Dashboard via
 * {@code LaunchInSequenceCommand.sequenceConfig} (the public-static holder).
 */
public class LaunchInSequenceConfig {
    /** Milliseconds between groups when consecutive same-color (PPG, GPP) */
    public double shotSpacingMs = 525;

    /** Milliseconds between shots when colors alternate (PGP pattern) */
    public double alternatingSpacingMs = 265;

    /** Timeout in seconds before giving up on spin-up */
    public double timeoutSeconds = 3.0;
}
