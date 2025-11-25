package org.firstinspires.ftc.teamcode.commands.LauncherCommands.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Configuration for LaunchAtPositionCommand.
 * Defines RPM values for shots from specific field positions.
 */
@Configurable
public class LaunchAtPositionConfig {
    /** RPM for shots from LAUNCH_FAR position */
    public double farLaunchRpm = 2925;

    /** RPM for shots from LAUNCH_CLOSE position */
    public double closeLaunchRpm = 2150;

    /** Default RPM if position unknown */
    public double defaultLaunchRpm = 2550;

    /** Timeout in seconds before giving up */
    public double timeoutSeconds = 3.0;
}
