package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherFeederConfig;

/**
 * Shared tunable RPM and hood positions for SHORT/MID/LONG ranges.
 * Both distance-based and preset-based commands should reference this single
 * config so field tuning only needs to happen in one place.
 */
@Configurable
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

    /** Shortstop range RPMs (autonomous close shots, tuned separately from SHORT) */
    public double shortstopLeftRpm;
    public double shortstopCenterRpm;
    public double shortstopRightRpm;

    /** Per-range hood positions (applied uniformly to all lanes) */
    public double shortHoodPosition;
    public double midHoodPosition;
    public double longHoodPosition;
    public double shortstopHoodPosition;

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
        config.shortLeftRpm = 2000;
        config.shortCenterRpm = 2000; // Center disabled by default
        config.shortRightRpm = 2000;

        /** Mid range RPMs */
        config.midLeftRpm = 2400;
        config.midCenterRpm = 2400; // Center disabled by default
        config.midRightRpm = 2400;

        /** Long range RPMs */
        config.longLeftRpm = 2725;
        config.longCenterRpm = 2725; // Center disabled by default
        config.longRightRpm = 2725;

        /** Shortstop range RPMs (starts matching SHORT, tune independently for auto) */
        config.shortstopLeftRpm = 2000;
        config.shortstopCenterRpm = 2000;
        config.shortstopRightRpm = 2000;

        /** Per-range hood positions (applied uniformly to all lanes) */
        config.shortHoodPosition = 0.3;
        config.midHoodPosition = 0.05;
        config.longHoodPosition = 0.0;
        config.shortstopHoodPosition = 0.3; // Starts matching SHORT

        /** Timeout in seconds before giving up on spin-up */
        config.timeoutSeconds = 3.5;

        return config;
    }

    private static CommandRangeConfig createCommandRangeConfig20245() {
        CommandRangeConfig config = new CommandRangeConfig();
        // Apply 20245-specific values
        /** Short range RPMs */
        config.shortLeftRpm = 2000;
        config.shortCenterRpm = 1875; // Center disabled by default
        config.shortRightRpm = 1875;

        /** Mid range RPMs */
        config.midLeftRpm = 2400;
        config.midCenterRpm = 2300; // Center disabled by default
        config.midRightRpm = 2300;

        /** Long range RPMs */
        config.longLeftRpm = 2828;
        config.longCenterRpm = 2828; // Center disabled by default
        config.longRightRpm = 2828;

        /** Shortstop range RPMs (starts matching SHORT, tune independently for auto) */
        config.shortstopLeftRpm = 2000;
        config.shortstopCenterRpm = 1875;
        config.shortstopRightRpm = 1875;

        /** Per-range hood positions (applied uniformly to all lanes) */
        config.shortHoodPosition = .42;
        config.midHoodPosition = 0.05;
        config.longHoodPosition = .05;
        config.shortstopHoodPosition = .42; // Starts matching SHORT

        /** Timeout in seconds before giving up on spin-up */
        config.timeoutSeconds = 3.5;
        return config;
    }
}
