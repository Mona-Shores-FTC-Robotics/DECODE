package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AimAtGoalCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.TryRelocalizeForShotCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.AutoSmartIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FollowPathBuilder;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
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
        public double maxPathPower = 0.65;
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
        public double launchFarHeadingDeg = 115;

        // Artifacts at Alliance Wall)
        public double artifactsWallX = 13;
        public double artifactsWallY = 8.5;
        public double artifactsWallHeading = 180;

        // Control point for segment: leavewall
        public double wallControlPointX = 27.5;
        public double wallControlPointY = 18;

        // ArtifactsSet3
        public double artifactsSet3X = 24;
        public double artifactsSet3Y = 38.5;
        public double artifactsSet3Heading = 90;

        // Control point for segment: ArtifactsSet3
        public double artifactSet3ControlPointX = 19;
        public double artifactSet3ControlPointY = 3;

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
        return create(robot, alliance, null, null);
    }

    /**
     * Creates the autonomous command sequence with optional start pose override.
     * @param robot Robot instance with all subsystems
     * @param alliance Current alliance (BLUE or RED)
     * @param startOverride Vision-detected start pose (or null to use waypoints)
     * @return Complete autonomous command
     */
    public static Command create(Robot robot, Alliance alliance, Pose startOverride) {
        return create(robot, alliance, startOverride, null);
    }

    /**
     * Creates the autonomous command sequence with optional start pose override and auto timer.
     * @param robot Robot instance with all subsystems
     * @param alliance Current alliance (BLUE or RED)
     * @param startOverride Vision-detected start pose (or null to use waypoints)
     * @param autoTimer Timer tracking elapsed auto time (will start if null)
     * @return Complete autonomous command
     */
    public static Command create(Robot robot, Alliance alliance, Pose startOverride, ElapsedTime autoTimer) {
        LauncherCommands launcherCommands = new LauncherCommands(robot.launcher, robot.intake, robot.drive, robot.lighting);
        AutoSmartIntakeCommand autoSmartIntake = new AutoSmartIntakeCommand(robot.intake);
        ElapsedTime timer = autoTimer != null ? autoTimer : new ElapsedTime();
        if (autoTimer == null) {
            timer.reset();
        }

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
                // Launch Preloads
                new ParallelDeadlineGroup(
                        firstPathBuilder
                                .to(launchFar())
                                .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                .build(config.maxPathPower),
                        launcherCommands.presetRangeSpinUp(LauncherRange.FAR_AUTO, true) // Spin up to FAR_AUTO speed and stay their the whole auto
                ),

                new TryRelocalizeForShotCommand(robot.drive, robot.vision),
                new AimAtGoalCommand(robot.drive, robot.vision),
                launcherCommands.launchAccordingToMode(false),

                // Pickup Alliance Wall Artifacts
                new FollowPathBuilder(robot, alliance)
                        .from(launchFar())
                        .to(artifactsAllianceWall())
                        .withControl(wallControl())
                        .withConstantHeading(artifactsAllianceWall().getHeading())
                        .build(config.maxPathPower),

                // Return and launch alliance wall artifacts
                new FollowPathBuilder(robot, alliance)
                        .from(artifactsAllianceWall())
                        .to(launchFar())
                        .withControl(wallControl())
                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                        .build(config.maxPathPower),

                new TryRelocalizeForShotCommand(robot.drive, robot.vision),
                new AimAtGoalCommand(robot.drive, robot.vision),
                new Delay(config.delayForGateToOpen),
                launcherCommands.launchAccordingToMode(false),

                // Pickup Artifact Set 3
                new FollowPathBuilder(robot, alliance)
                        .from(launchFar())
                        .withControl(artifactsSet3Control0())
                        .to(artifactsSet3())
                        .withConstantHeading(artifactsSet3().getHeading())
                        .build(config.maxPathPower),

                // Return and Launch Set 3
                new FollowPathBuilder(robot, alliance)
                        .from(artifactsSet3())
                        .to(launchFar())
                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                        .build(config.maxPathPower),

                new TryRelocalizeForShotCommand(robot.drive, robot.vision),
                new AimAtGoalCommand(robot.drive, robot.vision),
                launcherCommands.launchAccordingToMode(false),

                // Pickup Released Artifacts Try 1
                new FollowPathBuilder(robot, alliance)
                        .from(launchFar())
                        .to(releasedArtifacts1())
                        .withConstantHeading(releasedArtifacts1().getHeading())
                        .build(config.maxPathPower),

                // Return and Launch
                new FollowPathBuilder(robot, alliance)
                    .from(releasedArtifacts1())
                    .to(launchFar())
                    .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                    .build(config.maxPathPower),

                new TryRelocalizeForShotCommand(robot.drive, robot.vision),
                new AimAtGoalCommand(robot.drive, robot.vision),
                launcherCommands.launchAccordingToMode(false),

                // Pickup Released Artifacts Try 1
                new FollowPathBuilder(robot, alliance)
                        .from(launchFar())
                        .to(releasedArtifacts2())
                        .withControl(releasedArtifacts2Control0())
                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                        .build(config.maxPathPower),

                new IfElseCommand(
                        () -> hasTimeForFinalLaunch(timer),
                        new SequentialGroup(
                                // Return and launch
                                new FollowPathBuilder(robot, alliance)
                                        .from(releasedArtifacts2())
                                        .to(launchFar())
                                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                        .build(config.maxPathPower),

                                new TryRelocalizeForShotCommand(robot.drive, robot.vision),
                                new AimAtGoalCommand(robot.drive, robot.vision),
                                launcherCommands.launchAccordingToMode(false),

                                //Get Off Launch Line and Ready for Teleop
                                new FollowPathBuilder(robot, alliance)
                                        .from(launchFar())
                                        .to(readyForTeleop())
                                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                        .build(config.maxPathPower)
                        ),
                        new SequentialGroup(
                                new FollowPathBuilder(robot, alliance)
                                        .from(releasedArtifacts2())
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

    private static Pose artifactsSet3() {
        return new Pose(waypoints.artifactsSet3X, waypoints.artifactsSet3Y, Math.toRadians(waypoints.artifactsSet3Heading));
    }

    private static Pose artifactsSet3Control0() {
        return new Pose(waypoints.artifactSet3ControlPointX, waypoints.artifactSet3ControlPointY, 0);
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

    private static boolean hasTimeForFinalLaunch(ElapsedTime timer) {
        double timeRemaining = config.autoDurationSeconds - timer.seconds();
        return timeRemaining >= config.minTimeForFinalLaunchSeconds;
    }
}
