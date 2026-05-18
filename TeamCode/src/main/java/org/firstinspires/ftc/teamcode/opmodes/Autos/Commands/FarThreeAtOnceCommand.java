package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AimAtGoalCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.AutoSmartIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.SetIntakeModeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.TimedEjectCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FollowPathBuilder;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

/**
 * Generated autonomous command from Pedro Pathing .pp file
 * Usage in OpMode:
 *   Command auto = LocalizeCommand.create(robot, activeAlliance);
 *   CommandManager.INSTANCE.scheduleCommand(auto);
 * 
 */
public class FarThreeAtOnceCommand {

    public static class Config {
        public double maxPathPower = .85;
        public double endTimeForLinearHeadingInterpolation = .7;
        public double autoDurationSeconds = 30.0;
        public double minTimeForFinalLaunchSeconds = 5.0;
        public double ejectTime = 1000;
    }

    public static class Waypoints {
        public double startX = 56;
        public double startY = 5;
        public double startHeading = 90.0;

        // LaunchFar
        public double launchFarX = 58;
        public double launchFarY = 11;
        public double launchFarHeadingDeg = 110;

        // Artifacts at Alliance Wall)
        public double artifactsWallX = 9;
        public double artifactsWallY = 6.5;
        public double artifactsWallHeading = 185;

        // Control point for segment: leavewall
        public double leaveWallControlPointX = 27.5;
        public double leaveWallControlPointY = 18;

        // ArtifactsSet3
        public double artifactsSet3X = 23.75;
        public double artifactsSet3Y = 37.5;
        public double artifactsSet3Heading = 90;

        // Control point for segment: ArtifactsSet2
        public double artifactSet3ControlPointX = 20.5;
        public double artifactSet3ControlPointY = 7;

        // ArtifactsSet3
        public double artifactsSet2X = 23.75;
        public double artifactsSet2Y = 62;
        public double artifactsSet2Heading = 90;

        // Control point for segment: ArtifactsSet2
        public double artifactSet2ControlPointX = 19;
        public double artifactSet2ControlPointY = 23;

        // NearGate
        public double readyForTeleopX = 33;
        public double readyForTeleopY = 12;
        public double readyForTeleopHeading = 90;

    }

    public static Config config = new Config();
    public static Waypoints waypoints = new Waypoints();

    private FarThreeAtOnceCommand() {}

    /**
     * Gets the default start pose from waypoints (before alliance mirroring).
     * This is the fallback when vision initialization is not available.
     * @return Default start pose
     */
    public static Pose getDefaultStartPose() {
        return start();
    }

    /**
     * Creates the autonomous command sequence.
     * @param robot Robot instance with all subsystems
     * @param alliance Current alliance (BLUE or RED)
     * @return Complete autonomous command
     */
    public static Command create(Robot robot, Alliance alliance) {
        return create(robot, alliance, null);
    }

    /**
     * Creates the autonomous command sequence with optional start pose override.
     * @param robot Robot instance with all subsystems
     * @param alliance Current alliance (BLUE or RED)
     * @param startOverride Vision-detected start pose (or null to use waypoints)
     * @return Complete autonomous command
     */
    public static Command create(Robot robot, Alliance alliance, Pose startOverride) {
        LauncherCommands launcherCommands = new LauncherCommands(robot.launcher, robot.intake, robot.drive, robot.lighting);
        Command autoSmartIntake = robot.intake.autoSmartIntakeCmd();

        // Build first path: start -> launch position
        FollowPathBuilder firstPathBuilder = new FollowPathBuilder(robot, alliance);
        if (startOverride != null) {
            // Vision detected: use follower's current pose (world coordinates, no mirroring)
            firstPathBuilder.fromWorldCoordinates(robot.drive.getFollower().getPose());
        } else {
            // No vision: use waypoint start pose (will be mirrored for red alliance)
            firstPathBuilder.from(start());
        }

        Command mainSequence = Groups.sequential(
                // Reset timer when auto actually starts (not when command is created)
                ConditionalFinalLaunchCommand.createTimerReset(),

                // Launch Preloads
                Groups.deadline(
                firstPathBuilder
                        .to(launchFar())
                        .withConstantHeading(waypoints.launchFarHeadingDeg)
                        .build(config.maxPathPower),
                    robot.intake.setIntakeModeCmd(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE),
                    launcherCommands.presetRangeSpinUp(LauncherRange.FAR_AUTO, true) // Spin up to FAR_AUTO speed and stay their the whole auto
                ),
            launcherCommands.launchAccordingToMode(false),

            // Pickup Alliance Wall Artifacts
            Groups.deadline(
                new FollowPathBuilder(robot, alliance)
                        .from(launchFar())
                        .to(artifactsAllianceWall())
                        .withControl(leaveControl0())
                        .withConstantHeading(waypoints.artifactsWallHeading)
                        .build(config.maxPathPower),
                        Groups.sequential(
                                robot.intake.autoSmartIntakeCmd()
                        )
            ),

            // Return and launch alliance wall artifacts
            new FollowPathBuilder(robot, alliance)
                    .from(artifactsAllianceWall())
                    .to( launchFar())
                    .withControl(leaveControl0())
                    .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                    .build(config.maxPathPower),

            launcherCommands.launchAccordingToMode(false),

            // Pickup Artifact Set 3
            Groups.deadline(
                    new FollowPathBuilder(robot, alliance)
                            .from(launchFar())
                            .to(artifactsSet3())
                            .withControl(artifactsSet3Control0())
                            .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                            .build(config.maxPathPower),
                    Groups.sequential(
                            robot.intake.autoSmartIntakeCmd()
                    )
            ),

            // Return and launch set 3
            new FollowPathBuilder(robot, alliance)
                    .from(artifactsSet3())
                    .to(launchFar())
                    .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                    .build(config.maxPathPower),

            launcherCommands.launchAccordingToMode(false),

            // Pickup Artifact Set 2
            Groups.deadline(
                    new FollowPathBuilder(robot, alliance)
                            .from(launchFar())
                            .withControl(artifactsSet2Control0())
                            .to(artifactsSet2())
                            .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                            .build(config.maxPathPower),
                    Groups.sequential(
                            robot.intake.autoSmartIntakeCmd()
                    )
            ),

            // Conditionally return and launch if time permits, otherwise go straight to park
            ConditionalFinalLaunchCommand.create(
                    config.autoDurationSeconds,
                    config.minTimeForFinalLaunchSeconds,
                    // If enough time: return to launch, shoot, then park
                    Groups.sequential(
                            new FollowPathBuilder(robot, alliance)
                                    .from(artifactsSet2())
                                    .to(launchFar())
                                    .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                    .build(config.maxPathPower),

                            launcherCommands.launchAccordingToMode(false),

                            Groups.deadline(
                                    new FollowPathBuilder(robot, alliance)
                                        .from(launchFar())
                                        .to(readyForTeleop())
                                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                        .build(config.maxPathPower),
                                    Groups.sequential(
                                            robot.intake.autoSmartIntakeCmd()
                                    )
                            )
                    ),
                    // If not enough time: go straight to park
                    Groups.sequential(
                            new FollowPathBuilder(robot, alliance)
                                    .from(artifactsSet2())
                                    .to(readyForTeleop())
                                    .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                    .build(config.maxPathPower)
                    )
            )
        );

        return
                mainSequence;
//todo CONSIDER CHANGING IF ROBOT NOT INTAKING DURING AUTO
//                new InstantCommand(()-> robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))
//                autoSmartIntake

    }

    private static Pose start() {
        return new Pose(waypoints.startX, waypoints.startY, Math.toRadians(waypoints.startHeading));
    }

    private static Pose launchFar() {
        return new Pose(waypoints.launchFarX, waypoints.launchFarY, Math.toRadians(waypoints.launchFarHeadingDeg));
    }

    private static Pose artifactsAllianceWall() {
        return new Pose(waypoints.artifactsWallX, waypoints.artifactsWallY, Math.toRadians(waypoints.artifactsWallHeading));
    }


    private static Pose leaveControl0() {
        return new Pose(waypoints.leaveWallControlPointX, waypoints.leaveWallControlPointY, 0);
    }

    private static Pose artifactsSet3() {
        return new Pose(waypoints.artifactsSet3X, waypoints.artifactsSet3Y, Math.toRadians(waypoints.artifactsSet3Heading));
    }

    private static Pose artifactsSet3Control0() {
        return new Pose(waypoints.artifactSet3ControlPointX, waypoints.artifactSet3ControlPointY, 0);
    }

    private static Pose artifactsSet2() {
        return new Pose(waypoints.artifactsSet2X, waypoints.artifactsSet2Y, Math.toRadians(waypoints.artifactsSet2Heading));
    }

    private static Pose artifactsSet2Control0() {
        return new Pose(waypoints.artifactSet2ControlPointX, waypoints.artifactSet2ControlPointY, 0);
    }

    private static Pose readyForTeleop() {
        return new Pose(waypoints.readyForTeleopX , waypoints.readyForTeleopY , Math.toRadians(waypoints.readyForTeleopHeading));
    }

}
