package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

/**
 * Centralized robot-specific configuration selector.
 * Automatically selects the correct configuration based on RobotState.getRobotName().
 *
 * This class uses lazy initialization - configs are selected on first access after
 * the robot name has been set by ControlHubIdentifierUtil.
 */
public class RobotConfigs {

    // Cached configs - initialized once on first access
    private static volatile LauncherSubsystem.FeederConfig cachedFeederConfig = null;
    private static volatile LauncherSubsystem.HoodConfig cachedHoodConfig = null;

    /**
     * Gets the active FeederConfig for the current robot.
     * @return FeederConfig19429 or FeederConfig20245 based on robot name
     */
    public static LauncherSubsystem.FeederConfig getFeederConfig() {
        if (cachedFeederConfig == null) {
            synchronized (RobotConfigs.class) {
                if (cachedFeederConfig == null) {
                    String robotName = RobotState.getRobotName();
                    if ("DECODE_20245".equals(robotName)) {
                        cachedFeederConfig = new LauncherSubsystem.FeederConfig20245();
                    } else {
                        // Default to 19429 config if robot name is unknown or is 19429
                        cachedFeederConfig = new LauncherSubsystem.FeederConfig19429();
                    }
                }
            }
        }
        return cachedFeederConfig;
    }

    /**
     * Gets the active HoodConfig for the current robot.
     * @return HoodConfig19429 or HoodConfig20245 based on robot name
     */
    public static LauncherSubsystem.HoodConfig getHoodConfig() {
        if (cachedHoodConfig == null) {
            synchronized (RobotConfigs.class) {
                if (cachedHoodConfig == null) {
                    String robotName = RobotState.getRobotName();
                    if ("DECODE_20245".equals(robotName)) {
                        cachedHoodConfig = new LauncherSubsystem.HoodConfig20245();
                    } else {
                        // Default to 19429 config if robot name is unknown or is 19429
                        cachedHoodConfig = new LauncherSubsystem.HoodConfig19429();
                    }
                }
            }
        }
        return cachedHoodConfig;
    }

    /**
     * Gets a human-readable description of which config set is active.
     * Useful for telemetry diagnostics.
     */
    public static String getActiveConfigName() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_20245".equals(robotName)) {
            return "20245";
        } else if ("DECODE_19429".equals(robotName)) {
            return "19429";
        } else {
            return "19429 (default)";
        }
    }

    /**
     * For testing: clears cached configs to force re-selection.
     * Should not be used during normal operation.
     */
    public static void clearCache() {
        synchronized (RobotConfigs.class) {
            cachedFeederConfig = null;
            cachedHoodConfig = null;
        }
    }
}
