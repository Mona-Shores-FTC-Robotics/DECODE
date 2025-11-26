package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Shared tunable RPM and hood positions for SHORT/MID/LONG ranges.
 * Both distance-based and preset-based commands should reference this single
 * config so field tuning only needs to happen in one place.
 */
@Configurable
public class CommandRangeConfig {

    /** Shared config instance used by all launcher commands to keep tuning in sync. */
    public static final CommandRangeConfig SHARED = new CommandRangeConfig();

    /** Short range RPMs */
    public double shortLeftRpm = 2000;
    public double shortCenterRpm = 2000; // Center disabled by default
    public double shortRightRpm = 2000;

    /** Mid range RPMs */
    public double midLeftRpm = 2400;
    public double midCenterRpm = 2400; // Center disabled by default
    public double midRightRpm = 2400;

    /** Long range RPMs */
    public double longLeftRpm = 2725;
    public double longCenterRpm = 2725; // Center disabled by default
    public double longRightRpm = 2725;

    /** Per-range hood positions (applied uniformly to all lanes) */
    public double shortHoodPosition = 0.3;
    public double midHoodPosition = 0.05;
    public double longHoodPosition = 0.0;

    /** Timeout in seconds before giving up on spin-up */
    public double timeoutSeconds = 3.5;
}
