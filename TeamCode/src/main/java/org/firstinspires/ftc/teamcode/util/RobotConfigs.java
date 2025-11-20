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

    /**
     * Gets the active FeederConfig for the current robot.
     * Returns the static instance from LauncherSubsystem so Dashboard edits are applied.
     * @return FeederConfig19429 or FeederConfig20245 based on robot name
     */
    public static LauncherSubsystem.FeederConfig getFeederConfig() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_20245".equals(robotName)) {
            return LauncherSubsystem.feederConfig20245;
        } else {
            // Default to 19429 config if robot name is unknown or is 19429
            return LauncherSubsystem.feederConfig19429;
        }
    }

    /**
     * Gets the active HoodConfig for the current robot.
     * Returns the static instance from LauncherSubsystem so Dashboard edits are applied.
     * @return HoodConfig19429 or HoodConfig20245 based on robot name
     */
    public static LauncherSubsystem.HoodConfig getHoodConfig() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_20245".equals(robotName)) {
            return LauncherSubsystem.hoodConfig20245;
        } else {
            // Default to 19429 config if robot name is unknown or is 19429
            return LauncherSubsystem.hoodConfig19429;
        }
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
}
