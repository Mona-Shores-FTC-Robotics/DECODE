package org.firstinspires.ftc.teamcode.commands.LauncherCommands.config;

/**
 * Tunable RPM and hood positions for SHORT/MID/LONG ranges. The data SHAPE
 * lives here; per-robot values live in {@code util/RobotProfile.java} under
 * {@code rangeConfig19429()} / {@code rangeConfig20245()}. Access the live
 * config via {@code RobotProfile.forCurrent().commandRange}.
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

    /** Mid Auto range RPMs (close-side gate cycles, tuned separately from MID) */
    public double midAutoLeftRpm;
    public double midAutoCenterRpm;
    public double midAutoRightRpm;

    /** Far Auto range RPMs (autonomous far shots, tuned separately from LONG) */
    public double farAutoLeftRpm;
    public double farAutoCenterRpm;
    public double farAutoRightRpm;

    /** Per-range hood positions (applied uniformly to all lanes) */
    public double shortHoodPosition;
    public double midHoodPosition;
    public double longHoodPosition;
    public double shortAutoHoodPosition;
    public double midAutoHoodPosition;
    public double farAutoHoodPosition;

    /** Timeout in seconds before giving up on spin-up */
    public double timeoutSeconds;
}
