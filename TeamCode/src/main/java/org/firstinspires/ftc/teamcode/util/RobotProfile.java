package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;

import org.firstinspires.ftc.teamcode.commands.LauncherCommands.DistanceBasedSpinCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.config.CommandRangeConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveAimAssistConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveFixedAngleAimConfig;
import org.firstinspires.ftc.teamcode.subsystems.drive.config.DriveRightTriggerFixedAngleConfig;
import org.firstinspires.ftc.teamcode.subsystems.intake.config.IntakeGateConfig;
import org.firstinspires.ftc.teamcode.subsystems.intake.config.IntakeLaneSensorConfig;
import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherFeederConfig;
import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherFlywheelConfig;
import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherHoodConfig;
import org.firstinspires.ftc.teamcode.subsystems.launcher.config.LauncherTimingConfig;

/**
 * Immutable per-robot configuration bundle. Selected at runtime by
 * {@link RobotState#getRobotName()}. Adding a third robot is one new constant
 * + one branch in {@link #forCurrent()} — no new dispatcher methods.
 *
 * Fields reference the shared {@code @Configurable} singletons declared on
 * the various config classes, so Panels/Dashboard edits to those instances
 * propagate through this profile.
 */
public final class RobotProfile {
    public final String name;
    public final LauncherFeederConfig feeder;
    public final LauncherHoodConfig hood;
    public final LauncherFlywheelConfig flywheel;
    public final LauncherTimingConfig timing;
    public final PinpointConstants pinpoint;
    public final FollowerConstants follower;
    public final MecanumConstants drive;
    public final DriveAimAssistConfig aimAssist;
    public final DriveFixedAngleAimConfig fixedAngleAim;
    public final DriveRightTriggerFixedAngleConfig rightTriggerFixedAngle;
    public final IntakeGateConfig gate;
    public final IntakeLaneSensorConfig.LanePresenceConfig lanePresence;
    public final CommandRangeConfig commandRange;

    private RobotProfile(
            String name,
            LauncherFeederConfig feeder,
            LauncherHoodConfig hood,
            LauncherFlywheelConfig flywheel,
            LauncherTimingConfig timing,
            PinpointConstants pinpoint,
            FollowerConstants follower,
            MecanumConstants drive,
            DriveAimAssistConfig aimAssist,
            DriveFixedAngleAimConfig fixedAngleAim,
            DriveRightTriggerFixedAngleConfig rightTriggerFixedAngle,
            IntakeGateConfig gate,
            IntakeLaneSensorConfig.LanePresenceConfig lanePresence,
            CommandRangeConfig commandRange) {
        this.name = name;
        this.feeder = feeder;
        this.hood = hood;
        this.flywheel = flywheel;
        this.timing = timing;
        this.pinpoint = pinpoint;
        this.follower = follower;
        this.drive = drive;
        this.aimAssist = aimAssist;
        this.fixedAngleAim = fixedAngleAim;
        this.rightTriggerFixedAngle = rightTriggerFixedAngle;
        this.gate = gate;
        this.lanePresence = lanePresence;
        this.commandRange = commandRange;
    }

    public static final RobotProfile ROBOT_19429 = new RobotProfile(
            "19429",
            LauncherFeederConfig.feederConfig19429,
            LauncherHoodConfig.hoodConfig19429,
            LauncherFlywheelConfig.flywheelConfig19429,
            LauncherTimingConfig.timing19429,
            Constants.pinpointConstants19429,
            Constants.followerConstants19429,
            Constants.driveConstants19429,
            DriveSubsystem.aimAssistConfig_Robot19429,
            DriveSubsystem.fixedAngleAimConfig_Robot19429,
            DriveSubsystem.rightTriggerFixedAngleConfig_Robot19429,
            IntakeGateConfig.gateConfig19429,
            IntakeLaneSensorConfig.lanePresenceConfig19429,
            DistanceBasedSpinCommand.rangeConfig_Robot19429);

    public static final RobotProfile ROBOT_20245 = new RobotProfile(
            "20245",
            LauncherFeederConfig.feederConfig20245,
            LauncherHoodConfig.hoodConfig20245,
            LauncherFlywheelConfig.flywheelConfig20245,
            LauncherTimingConfig.timing20245,
            Constants.pinpointConstants20245,
            Constants.followerConstants20245,
            Constants.driveConstants20245,
            DriveSubsystem.aimAssistConfig_Robot20245,
            DriveSubsystem.fixedAngleAimConfig_Robot20245,
            DriveSubsystem.rightTriggerFixedAngleConfig_Robot20245,
            IntakeGateConfig.gateConfig20245,
            IntakeLaneSensorConfig.lanePresenceConfig20245,
            DistanceBasedSpinCommand.rangeConfig_Robot20245);

    /**
     * Returns the profile for the robot currently identified by
     * {@link RobotState#getRobotName()}. Falls back to 20245 if unknown.
     */
    public static RobotProfile forCurrent() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_19429".equals(robotName)) return ROBOT_19429;
        return ROBOT_20245;
    }

    /** Human-readable identifier for telemetry. Marks "(default)" if unknown. */
    public static String activeName() {
        String robotName = RobotState.getRobotName();
        if ("DECODE_19429".equals(robotName)) return "19429";
        if ("DECODE_20245".equals(robotName)) return "20245";
        return "20245 (default)";
    }
}
