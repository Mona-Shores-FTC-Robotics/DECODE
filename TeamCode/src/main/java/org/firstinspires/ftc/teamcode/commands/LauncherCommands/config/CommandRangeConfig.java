package org.firstinspires.ftc.teamcode.commands.LauncherCommands.config;

/**
 * Shared tunable RPM and hood positions for SHORT/MID/LONG ranges.
 * Both distance-based and preset-based commands should reference this single
 * config so field tuning only needs to happen in one place.
 *
 * NOTE: This class is NOT annotated with @Configurable directly.
 * Instead, robot-specific instances are exposed as static fields on
 * DistanceBasedSpinCommand (which has @Configurable), allowing Dashboard
 * to edit the instance fields of these config objects.
 */
public class CommandRangeConfig {
    /** Short range RPMs */
    public double shortLeftRpm;
    public double shortCenterRpm; // Center disabled by default
    public double shortRightRpm;

    /** Mid range RPMs */
    public double midLeftRpm;
    public double midCenterRpm; // Center disabled by default
    public double midRightRpm;

    /** Long range RPMs */
    public double longLeftRpm;
    public double longCenterRpm; // Center disabled by default
    public double longRightRpm;

    public double longMinLeftRpm;
    public double longMinCenterRpm; // Center disabled by default
    public double longMinRightRpm;

    public double longMaxLeftRpm;
    public double longMaxCenterRpm; // Center disabled by default
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



    // Robot-specific instances
    public static CommandRangeConfig commandRangeConfig19429 = createCommandRangeConfig19429();
    public static CommandRangeConfig commandRangeConfig20245 = createCommandRangeConfig20245();

    /**
     * Creates feeder configuration for robot 19429.
     */
    private static CommandRangeConfig createCommandRangeConfig19429() {
        CommandRangeConfig config = new CommandRangeConfig();
        // Apply 19429-specific values
        /** Short range RPMs */
        config.shortLeftRpm = 1900;
        config.shortCenterRpm = 1900; // Center disabled by default
        config.shortRightRpm = 1900;

        /** Mid range RPMs */
        config.midLeftRpm = 2500;
        config.midCenterRpm = 2500; // Center disabled by default
        config.midRightRpm = 2500;

        /** Long range RPMs */
        config.longLeftRpm = 2900;
        config.longCenterRpm = 2900; // Center disabled by default
        config.longRightRpm = 2900;

        /** Long range min dist RPMs for DISTANCEBASEDSPINCOMMADN2 */
        config.longMinLeftRpm = 2725;
        config.longMinCenterRpm = 2725; // Center disabled by default
        config.longMinRightRpm = 2725;

        /** Long range max dist RPMs for DISTANCEBASEDSPINCOMMADN2 */
        config.longMaxLeftRpm = 2900;
        config.longMaxCenterRpm = 2900; // Center disabled by default
        config.longMaxRightRpm = 2900;

        /** Short Auto range RPMs (starts matching SHORT, tune independently for auto) */
        config.shortAutoLeftRpm = 2025;
        config.shortAutoCenterRpm = 2025;
        config.shortAutoRightRpm = 2025;

        /** Far Auto range RPMs (starts matching LONG, tune independently for auto) */
        config.farAutoLeftRpm = 2725;
        config.farAutoCenterRpm = 2725;
        config.farAutoRightRpm = 2725;

        /** Per-range hood positions (applied uniformly to all lanes) */
        config.shortHoodPosition = 1;
        config.midHoodPosition = 0.05;
        config.longHoodPosition = 0.0;

        config.shortAutoHoodPosition = .62; // Starts matching SHORT
        config.farAutoHoodPosition = 0.0; // Starts matching LONG

        /** Timeout in seconds before giving up on spin-up */
        config.timeoutSeconds = 3.5;

        return config;
    }

    private static CommandRangeConfig createCommandRangeConfig20245() {
        CommandRangeConfig config = new CommandRangeConfig();
        // Apply 20245-specific values
        /** Short range RPMs */
        config.shortLeftRpm = 1900;
        config.shortCenterRpm = 1900; // Center disabled by default
        config.shortRightRpm = 1900;

        /** Mid range RPMs */
        config.midLeftRpm = 2300;
        config.midCenterRpm = 2300; // Center disabled by default
        config.midRightRpm = 2300;

        /** Long range RPMs */
        config.longLeftRpm = 2900;
        config.longCenterRpm = 2900; // Center disabled by default
        config.longRightRpm = 2900;

        /** Long range min dist RPMs for DISTANCEBASEDSPINCOMMADN2 */
        config.longMinLeftRpm = 2725;
        config.longMinCenterRpm = 2725; // Center disabled by default
        config.longMinRightRpm = 2725;

        /** Long range max dist RPMs for DISTANCEBASEDSPINCOMMADN2 */
        config.longMaxLeftRpm = 2900;
        config.longMaxCenterRpm = 2900; // Center disabled by default
        config.longMaxRightRpm = 2900;

        /** Short Auto range RPMs (starts matching SHORT, tune independently for auto) */
        config.shortAutoLeftRpm = 2025;
        config.shortAutoCenterRpm = 2025;
        config.shortAutoRightRpm = 2025;

        /** Far Auto range RPMs (starts matching LONG, tune independently for auto) */
        config.farAutoLeftRpm = 2725;
        config.farAutoCenterRpm = 2725;
        config.farAutoRightRpm = 2725;

        /** Per-range hood positions (applied uniformly to all lanes) */
        config.shortHoodPosition = 1;
        config.midHoodPosition = 0;
        config.longHoodPosition = 0.0;

        config.shortAutoHoodPosition = .62; // Starts matching SHORT
        config.farAutoHoodPosition = 0.0; // Starts matching LONG

        /** Timeout in seconds before giving up on spin-up */
        config.timeoutSeconds = 3.5;
        return config;
    }
}
