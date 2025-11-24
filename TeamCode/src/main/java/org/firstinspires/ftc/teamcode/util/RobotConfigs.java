package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.config.IntakeGateConfig;
import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherFeederConfig;
import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherFlywheelConfig;
import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherHoodConfig;
import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherTimingConfig;

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
     * Returns the static instance from LauncherFeederConfig so Dashboard edits are applied.
     * @return FeederConfig19429 or FeederConfig20245 based on robot name
     */
    public static LauncherFeederConfig getFeederConfig() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_19429".equals(robotName)) {
            return LauncherFeederConfig.feederConfig19429;
        } else {
            // Default to 20245 config if robot name is unknown or is 20245
            return LauncherFeederConfig.feederConfig20245;
        }
    }

    /**
     * Gets the active HoodConfig for the current robot.
     * Returns the static instance from LauncherHoodConfig so Dashboard edits are applied.
     * @return HoodConfig19429 or HoodConfig20245 based on robot name
     */
    public static LauncherHoodConfig getHoodConfig() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_19429".equals(robotName)) {
            return LauncherHoodConfig.hoodConfig19429;
        } else {
            // Default to 20245 config if robot name is unknown or is 20245
            return LauncherHoodConfig.hoodConfig20245;
        }
    }

    /**
     * Gets the active PinpointConstants for the current robot.
     * Returns the static instance from Constants so Dashboard edits are applied.
     * @return PinpointConstants19429 or PinpointConstants20245 based on robot name
     */
    public static PinpointConstants getPinpointConstants() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_19429".equals(robotName)) {
            return Constants.pinpointConstants19429;
        } else {
            // Default to 19429 config if robot name is unknown or is 19429
            return Constants.pinpointConstants20245;
        }
    }

    public static FollowerConstants getFollowerConstants() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_19429".equals(robotName)) {
            return Constants.followerConstants19429;
        } else {
            // Default to 19429 config if robot name is unknown or is 19429
            return Constants.followerConstants20245;
        }
    }

    public static MecanumConstants getDriveConstants() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_19429".equals(robotName)) {
            return Constants.driveConstants19429;
        } else {
            // Default to 19429 config if robot name is unknown or is 19429
            return Constants.driveConstants20245;
        }
    }

    /**
     * Gets the active FixedAngleAimConfig for the current robot.
     * Returns the static instance from DriveSubsystem so Dashboard edits are applied.
     * @return fixedAngleAimConfig19429 or fixedAngleAimConfig20245 based on robot name
     */
    public static DriveSubsystem.FixedAngleAimConfig getFixedAngleAimConfig() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_19429".equals(robotName)) {
            return DriveSubsystem.fixedAngleAimConfig19429;
        } else {
            return DriveSubsystem.fixedAngleAimConfig20245;
        }
    }

    /**
     * Gets the active InitialPoseConfig for the current robot.
     * Returns the static instance from DriveSubsystem so Dashboard edits are applied.
     * @return initialPoseConfig19429 or initialPoseConfig20245 based on robot name
     */
    public static DriveSubsystem.InitialPoseConfig getInitialPoseConfig() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_19429".equals(robotName)) {
            return DriveSubsystem.initialPoseConfig19429;
        } else {
            return DriveSubsystem.initialPoseConfig20245;
        }
    }

    /**
     * Gets the active Timing for the current robot.
     * Returns the static instance from LauncherTimingConfig so Dashboard edits are applied.
     * @return timing19429 or timing20245 based on robot name
     */
    public static LauncherTimingConfig getTiming() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_19429".equals(robotName)) {
            return LauncherTimingConfig.timing19429;
        } else {
            return LauncherTimingConfig.timing20245;
        }
    }

    /**
     * Gets the active FlywheelConfig for the current robot.
     * Returns the static instance from LauncherFlywheelConfig so Dashboard edits are applied.
     * @return flywheelConfig19429 or flywheelConfig20245 based on robot name
     */
    public static LauncherFlywheelConfig getFlywheelConfig() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_19429".equals(robotName)) {
            return LauncherFlywheelConfig.flywheelConfig19429;
        } else {
            return LauncherFlywheelConfig.flywheelConfig20245;
        }
    }

    /**
     * Gets the active GateConfig for the current robot.
     * Returns the static instance from IntakeGateConfig so Dashboard edits are applied.
     * @return gateConfig19429 or gateConfig20245 based on robot name
     */
    public static IntakeGateConfig getGateConfig() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_19429".equals(robotName)) {
            return IntakeGateConfig.gateConfig19429;
        } else {
            return IntakeGateConfig.gateConfig20245;
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
            return "20245 (default)";
        }
    }
}
