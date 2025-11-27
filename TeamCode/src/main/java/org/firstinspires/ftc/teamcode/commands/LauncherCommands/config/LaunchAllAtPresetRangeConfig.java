package org.firstinspires.ftc.teamcode.commands.LauncherCommands.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Configuration for LaunchAllAtPresetRangeCommand.
 * Defines RPM values for each lane at different shooting ranges.
 */
@Configurable
public class LaunchAllAtPresetRangeConfig {
    /** Short range configuration */
    public double shortLeftRpm = 2000;
    public double shortCenterRpm = 2000; // Center disabled by default
    public double shortRightRpm = 2000;

    /** Mid range configuration (default/current values) */
    public double midLeftRpm = 2400;
    public double midCenterRpm = 2400; // Center disabled by default
    public double midRightRpm = 2400;

    /** Long range configuration */
    public double longLeftRpm = 2725;
    public double longCenterRpm = 2725; // Center disabled by default
    public double longRightRpm = 2725; //2850 before but we had low battery and maked a bunch

    /** Timeout in seconds before giving up on spin-up */
    public double timeoutSeconds = 3.5;
}
