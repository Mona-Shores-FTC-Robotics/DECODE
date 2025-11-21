package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ftc.localization.constants.PinpointConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
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
     * Gets the active PinpointConstants for the current robot.
     * Returns the static instance from Constants so Dashboard edits are applied.
     * @return PinpointConstants19429 or PinpointConstants20245 based on robot name
     */
    public static PinpointConstants getPinpointConstants() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_20245".equals(robotName)) {
            return Constants.localizerConstants20245;
        } else {
            // Default to 19429 config if robot name is unknown or is 19429
            return Constants.localizerConstants19429;
        }
    }

    /**
     * Gets the active FixedAngleAimConfig for the current robot.
     * Returns the static instance from DriveSubsystem so Dashboard edits are applied.
     * @return fixedAngleAimConfig19429 or fixedAngleAimConfig20245 based on robot name
     */
    public static DriveSubsystem.FixedAngleAimConfig getFixedAngleAimConfig() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_20245".equals(robotName)) {
            return DriveSubsystem.fixedAngleAimConfig20245;
        } else {
            return DriveSubsystem.fixedAngleAimConfig19429;
        }
    }

    /**
     * Gets the active InitialPoseConfig for the current robot.
     * Returns the static instance from DriveSubsystem so Dashboard edits are applied.
     * @return initialPoseConfig19429 or initialPoseConfig20245 based on robot name
     */
    public static DriveSubsystem.InitialPoseConfig getInitialPoseConfig() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_20245".equals(robotName)) {
            return DriveSubsystem.initialPoseConfig20245;
        } else {
            return DriveSubsystem.initialPoseConfig19429;
        }
    }

    /**
     * Gets the active Timing for the current robot.
     * Returns the static instance from LauncherSubsystem so Dashboard edits are applied.
     * @return timing19429 or timing20245 based on robot name
     */
    public static LauncherSubsystem.Timing getTiming() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_20245".equals(robotName)) {
            return LauncherSubsystem.timing20245;
        } else {
            return LauncherSubsystem.timing19429;
        }
    }

    /**
     * Gets the active FlywheelConfig for the current robot.
     * Returns the static instance from LauncherSubsystem so Dashboard edits are applied.
     * @return flywheelConfig19429 or flywheelConfig20245 based on robot name
     */
    public static LauncherSubsystem.FlywheelConfig getFlywheelConfig() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_20245".equals(robotName)) {
            return LauncherSubsystem.flywheelConfig20245;
        } else {
            return LauncherSubsystem.flywheelConfig19429;
        }
    }

    /**
     * Gets the active GateConfig for the current robot.
     * Returns the static instance from IntakeSubsystem so Dashboard edits are applied.
     * @return gateConfig19429 or gateConfig20245 based on robot name
     */
    public static IntakeSubsystem.GateConfig getGateConfig() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_20245".equals(robotName)) {
            return IntakeSubsystem.gateConfig20245;
        } else {
            return IntakeSubsystem.gateConfig19429;
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
