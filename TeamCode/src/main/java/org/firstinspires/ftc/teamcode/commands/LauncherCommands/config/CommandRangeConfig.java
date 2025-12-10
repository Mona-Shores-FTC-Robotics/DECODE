package org.firstinspires.ftc.teamcode.commands.LauncherCommands.config;

/**
 * Shared tunable RPM and hood positions for SHORT/MID/LONG ranges.
 * Both distance-based and preset-based commands should reference this single
 * config so field tuning only needs to happen in one place.
 *
 * Robot-specific instances are defined as static fields on DistanceBasedSpinCommand
 * (which has @Configurable), allowing Dashboard/Panels to edit the instance fields
 * of these config objects live. Access via RobotConfigs.getCommandRangeConfig().
 */
public class CommandRangeConfig {
    /** Short range RPMs */
    public double shortLeftRpm;
    public double shortCenterRpm;
    public double shortRightRpm;

    /** Mid range RPMs */
    public double midLeftRpm;
    public double midCenterRpm;
    public double midRightRpm;

    /** Long range RPMs */
    public double longLeftRpm;
    public double longCenterRpm;
    public double longRightRpm;

    /** Long range min/max RPMs for distance-based interpolation */
    public double longMinLeftRpm;
    public double longMinCenterRpm;
    public double longMinRightRpm;
    public double longMaxLeftRpm;
    public double longMaxCenterRpm;
    public double longMaxRightRpm;

    /** Short Auto range RPMs (autonomous close shots, tuned separately from SHORT) */
    public double shortAutoLeftRpm;
    public double shortAutoCenterRpm;
    public double shortAutoRightRpm;

    /** Far Auto range RPMs (autonomous far shots, tuned separately from LONG) */
    public double farAutoLeftRpm;
    public double farAutoCenterRpm;
    public double farAutoRightRpm;

    /** Per-range hood positions (applied uniformly to all lanes) */
    public double shortHoodPosition;
    public double midHoodPosition;
    public double longHoodPosition;
    public double shortAutoHoodPosition;
    public double farAutoHoodPosition;

    /** Timeout in seconds before giving up on spin-up */
    public double timeoutSeconds;
}
