package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.TryRelocalizeForShotCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.AutoSmartIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FollowPathBuilder;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;

/**
 * Generated autonomous command from Pedro Pathing .pp file
 * Usage in OpMode:
 *   Command auto = LocalizeCommand.create(robot, activeAlliance);
 *   CommandManager.INSTANCE.scheduleCommand(auto);
 * 
 */
@Configurable
public class FarTogetherCommand {

    @Configurable
    public static class Config {
        public double maxPathPower = .85;
        public double endTimeForLinearHeadingInterpolation = .7;
        public double delayForGateToOpen = 1.0;
        public double autoDurationSeconds = 30.0;
        public double minTimeForFinalLaunchSeconds = 5.0;
    }

    @Configurable
    public static class Waypoints {
        public double startX = 56;
        public double startY = 5;
        public double startHeading = 90.0;

        // LaunchFar
        public double launchFarX = 58;
        public double launchFarY = 11;
        public double launchFarHeadingDeg = 108;

        // Artifacts at Alliance Wall)
        public double artifactsWallX = 13;
        public double artifactsWallY = 6.5;
        public double artifactsWallHeading = 180;

        // Control point for segment: leavewall
        public double wallControlPointX = 27.5;
        public double wallControlPointY = 10;


        // Chute Released Artifacts Try 1
        public double releasedTry1X = 13;
        public double releasedTry1Y = 23;
        public double releasedTry1Heading = 180;

        // Chute Released Artifacts Try 1
        public double releasedTry2X = 13;
        public double releasedTry2Y = 37;
        public double releasedTry2Heading = 180;

        public double releasedTry2controlX = 35.3;
        public double releasedTry2controlY = 13.6;

        // Off Line
        public double readyForTeleopX = 58;
        public double readyForTeleopY = 36;
        public double readyForTeleopHeading = 0;

    }

    public static Config config = new Config();
    public static Waypoints waypoints = new Waypoints();

    private FarTogetherCommand() {}

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
        AutoSmartIntakeCommand autoSmartIntake = new AutoSmartIntakeCommand(robot.intake);

        // Build first path: start -> launch position
        FollowPathBuilder firstPathBuilder = new FollowPathBuilder(robot, alliance);
        if (startOverride != null) {
            // Vision detected: use follower's current pose (world coordinates, no mirroring)
            firstPathBuilder.fromWorldCoordinates(robot.drive.getFollower().getPose());
        } else {
            // No vision: use waypoint start pose (will be mirrored for red alliance)
            firstPathBuilder.from(start());
        }

        Command mainSequence = new SequentialGroup(
                // Reset timer when auto actually starts (not when command is created)
                ConditionalFinalLaunchCommand.createTimerReset(),

                // Launch Preloads
                new ParallelDeadlineGroup(
                        firstPathBuilder
                                .to(launchFar())
                                .withConstantHeading(waypoints.launchFarHeadingDeg)                                .build(config.maxPathPower),
                        launcherCommands.presetRangeSpinUp(LauncherRange.FAR_AUTO, true) // Spin up to FAR_AUTO speed and stay their the whole auto
                ),

                new TryRelocalizeForShotCommand(robot.drive, robot.vision),
//                new AimAtGoalCommand(robot.drive, robot.vision),
                launcherCommands.launchAccordingToMode(false),

                // Pickup Alliance Wall Artifacts
                new FollowPathBuilder(robot, alliance)
                        .from(launchFar())
                        .to(artifactsAllianceWall())
                        .withControl(wallControl())
                        .withConstantHeading(waypoints.artifactsWallHeading)
                        .build(config.maxPathPower),

                // Return and launch alliance wall artifacts
                new FollowPathBuilder(robot, alliance)
                        .from(artifactsAllianceWall())
                        .to(launchFar())
                        .withControl(wallControl())
                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                        .build(config.maxPathPower),

                new TryRelocalizeForShotCommand(robot.drive, robot.vision),
//                new AimAtGoalCommand(robot.drive, robot.vision),
                new Delay(config.delayForGateToOpen),
                launcherCommands.launchAccordingToMode(false),

                // Pickup Released Artifacts Try 1
                new FollowPathBuilder(robot, alliance)
                        .from(launchFar())
                        .to(artifactsAllianceWall())
                        .withControl(wallControl())
                        .withConstantHeading(waypoints.artifactsWallHeading)
                        .build(config.maxPathPower),

                // Return and Launch
                new FollowPathBuilder(robot, alliance)
                    .from(artifactsAllianceWall())
                    .to(launchFar())
                        .withControl(wallControl())
                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                    .build(config.maxPathPower),

                new TryRelocalizeForShotCommand(robot.drive, robot.vision),
//                new AimAtGoalCommand(robot.drive, robot.vision),
                launcherCommands.launchAccordingToMode(false),

                // Pickup Released Artifacts Try 2
                new FollowPathBuilder(robot, alliance)
                        .from(launchFar())
                        .to(artifactsAllianceWall())
                        .withControl(wallControl())
                        .withConstantHeading(waypoints.artifactsWallHeading)
                        .build(config.maxPathPower),

                // Conditionally return and launch if time permits, otherwise go straight to park
                new ConditionalFinalLaunchCommand(
                        config.autoDurationSeconds,
                        config.minTimeForFinalLaunchSeconds,
                        // If enough time: return to launch, shoot, then park
                        new SequentialGroup(
                                new FollowPathBuilder(robot, alliance)
                                        .from(artifactsAllianceWall())
                                        .to(launchFar())
                                        .withControl(wallControl())
                                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                        .build(config.maxPathPower),

                                new TryRelocalizeForShotCommand(robot.drive, robot.vision),
                                launcherCommands.launchAccordingToMode(false),

                                new FollowPathBuilder(robot, alliance)
                                        .from(launchFar())
                                        .to(readyForTeleop())
                                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                        .build(config.maxPathPower)
                        ),
                        // If not enough time: go straight to park
                        new SequentialGroup(
                                new FollowPathBuilder(robot, alliance)
                                        .from(artifactsAllianceWall())
                                        .to(readyForTeleop())
                                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                        .build(config.maxPathPower)
                        )
                )
        );

        return new ParallelDeadlineGroup(
                mainSequence,
                autoSmartIntake // Run the smart intake the whole time
        );
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

    private static Pose wallControl() {
        return new Pose(waypoints.wallControlPointX , waypoints.wallControlPointY , 0);
    }

    private static Pose releasedArtifacts1() {
        return new Pose(waypoints.releasedTry1X , waypoints.releasedTry1Y , Math.toRadians(waypoints.releasedTry1Heading));
    }

    private static Pose releasedArtifacts2() {
        return new Pose(waypoints.releasedTry2X , waypoints.releasedTry2Y , Math.toRadians(waypoints.releasedTry2Heading));
    }

    private static Pose releasedArtifacts2Control0() {
        return new Pose(waypoints.releasedTry2controlX, waypoints.releasedTry2controlY, 0);
    }

    private static Pose readyForTeleop() {
        return new Pose(waypoints.readyForTeleopX, waypoints.readyForTeleopY, Math.toRadians(waypoints.readyForTeleopHeading));
    }
}
