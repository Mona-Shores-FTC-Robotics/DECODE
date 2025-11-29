package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import static org.firstinspires.ftc.teamcode.util.AutoField.poseForAlliance;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FollowPathBuilder;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;

/**
 * Generated autonomous command from Pedro Pathing .pp file
 * 
 * Usage in OpMode:
 *   Command auto = tryCommand.create(robot, activeAlliance);
 *   CommandManager.INSTANCE.scheduleCommand(auto);
 * 
 * TODO: Fill in robot actions at each segment (search for TODO comments)
 */
@Configurable
public class CloseHeadOnCommand {

    @Configurable
    public static class Config {
        public double maxPathPower = 0.79;
        public double intakeDelaySeconds = 2.5;
        public double endTimeForLinearHeadingInterpolation = .7;
    }

    @Configurable
    public static class Waypoints {
        public double startX = 27.5;
        public double startY = 128.5;
        public double startHeading = 0.0;

        // Preload Shoot
        public double preloadShootX = 29.0;
        public double preloadShootY = 114.0;
        public double preloadShootHeading = 134.0;

        // LineUpArtifactSet1
        public double lineUpArtifactSet1X = 48.2;
        public double lineUpArtifactSet1Y = 83.4;
        public double lineUpArtifactSet1Heading = 180.0;

        // Control point for segment: LineUpArtifactSet1
        public double lineUpArtifactSet1Control0X = 50.0;
        public double lineUpArtifactSet1Control0Y = 94.3;

        // ArtifactSet1
        public double artifactSet1X = 18.4;
        public double artifactSet1Y = 83.7;
        public double artifactSet1Heading = 180;

        // Shoot First Balls
        public double shootFirstBallsX = 29.0;
        public double shootFirstBallsY = 114.0;
        public double shootFirstBallsHeading = 134.0;

        // Lineup Second Balls
        public double lineupSecondBallsX = 47.3;
        public double lineupSecondBallsY = 66.2;
        public double lineupSecondBallsHeading = 196.0;

        // Control point for segment: Lineup Second Balls
        public double lineupSecondBallsControl0X = 70.5;
        public double lineupSecondBallsControl0Y = 74.3;

        // Pickup Second Balls
        public double pickupSecondBallsX = 9.7;
        public double pickupSecondBallsY = 56.5;
        public double pickupSecondBallsHeading = 196;

        // Shoot Second Balls
        public double shootSecondBallsX = 29.0;
        public double shootSecondBallsY = 114.0;
        public double shootSecondBallsHeading = 134.0;

        // Control point for segment: Shoot Second Balls
        public double shootSecondBallsControl0X = 63.3;
        public double shootSecondBallsControl0Y = 71.3;

        // Lineup Third Balls
        public double lineupThirdBallsX = 50.8;
        public double lineupThirdBallsY = 35.4;
        public double lineupThirdBallsHeading = 180.0;

        // Control point for segment: Lineup Third Balls
        public double lineupThirdBallsControl0X = 71.0;
        public double lineupThirdBallsControl0Y = 61.0;

        // Pickup Third Balls
        public double pickupThirdBallsX = 10.5;
        public double pickupThirdBallsY = 35.7;
        public double pickupThirdBallsHeading = 180;

        // Shoot Third Balls
        public double shootThirdBallsX = 29.0;
        public double shootThirdBallsY = 114.0;
        public double shootThirdBallsHeading = 144.0;

        // Control point for segment: Shoot Third Balls
        public double shootThirdBallsControl0X = 47.6;
        public double shootThirdBallsControl0Y = 49.0;

    }

    public static Config config = new Config();
    public static Waypoints waypoints = new Waypoints();

    private CloseHeadOnCommand() {}

    public static Command create(Robot robot, Alliance alliance, Pose startOverride) {
        LauncherCommands launcherCommands = new LauncherCommands(robot.launcher, robot.intake);

        // Build first path: start -> launch position
        FollowPathBuilder firstPathBuilder = new FollowPathBuilder(robot, alliance);
        if (startOverride != null) {
            // Vision detected: use follower's current pose (world coordinates, no mirroring)
            firstPathBuilder.fromWorldCoordinates(robot.drive.getFollower().getPose());
        } else {
            // No vision: use waypoint start pose (will be mirrored for red alliance)
            firstPathBuilder.from(start());
        }

        return new SequentialGroup(

                // Launch Preloads
                new ParallelDeadlineGroup(
                        firstPathBuilder
                                .to(preloadShoot())
                                .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                .build(config.maxPathPower),

                        launcherCommands.presetRangeSpinUp(LauncherRange.SHORT, true)
                ),
                launcherCommands.launchAll(false),

                // LineUpArtifactSet1
                new ParallelGroup(
                        new FollowPathBuilder(robot, alliance)
                                .from(preloadShoot())
                                .to(lineUpArtifactSet1())
                                .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                .build(config.maxPathPower),

                    new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))
                ),

                // ArtifactSet1
                followPath(robot, alliance, lineUpArtifactSet1(), artifactSet1()),

                new ParallelGroup(
                    followPath(robot, alliance, artifactSet1(), shootFirstBalls()),
                    new SequentialGroup(
                            new Delay(config.intakeDelaySeconds),
                            new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE))
                    )
                ),

                launcherCommands.launchAll(false),

                // Lineup Second Balls
                followPath(robot, alliance, shootFirstBalls(), lineupSecondBalls(), lineupSecondBallsControl0()),

                // Pickup Second Balls
                new ParallelGroup(
                    followPath(robot, alliance, lineupSecondBalls(), pickupSecondBalls()),
                    new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))
                ),

                new ParallelGroup(
                        followPath(robot, alliance, pickupSecondBalls(), shootSecondBalls(),shootSecondBallsControl0()),
                        new SequentialGroup(
                                new Delay(config.intakeDelaySeconds),
                                new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE))
                        )
                ),
                launcherCommands.launchAll(false),

                followPath(robot, alliance, shootSecondBalls(), lineupThirdBalls(),lineupThirdBallsControl0()),

                new ParallelGroup(
                        followPath(robot, alliance, lineupThirdBalls(), pickupThirdBalls()),
                        new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))
                ),

                new ParallelGroup(
                        followPath(robot, alliance, pickupThirdBalls(), shootThirdBalls(),shootThirdBallsControl0()),
                        new SequentialGroup(
                                new Delay(config.intakeDelaySeconds),
                                new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE))
                        )
                ),
                launcherCommands.launchAll(false)

        );
    }

    private static Pose start() {
        return new Pose(waypoints.startX, waypoints.startY, Math.toRadians(waypoints.startHeading));
    }

    private static Pose preloadShoot() {
        return new Pose(waypoints.preloadShootX, waypoints.preloadShootY, Math.toRadians(waypoints.preloadShootHeading));
    }

    private static Pose lineUpArtifactSet1() {
        return new Pose(waypoints.lineUpArtifactSet1X, waypoints.lineUpArtifactSet1Y, Math.toRadians(waypoints.lineUpArtifactSet1Heading));
    }

    private static Pose lineUpArtifactSet1Control0() {
        return new Pose(waypoints.lineUpArtifactSet1Control0X, waypoints.lineUpArtifactSet1Control0Y, 0);
    }

    private static Pose artifactSet1() {
        return new Pose(waypoints.artifactSet1X, waypoints.artifactSet1Y, Math.toRadians(waypoints.artifactSet1Heading));
    }

    private static Pose shootFirstBalls() {
        return new Pose(waypoints.shootFirstBallsX, waypoints.shootFirstBallsY, Math.toRadians(waypoints.shootFirstBallsHeading));
    }

    private static Pose lineupSecondBalls() {
        return new Pose(waypoints.lineupSecondBallsX, waypoints.lineupSecondBallsY, Math.toRadians(waypoints.lineupSecondBallsHeading));
    }

    private static Pose lineupSecondBallsControl0() {
        return new Pose(waypoints.lineupSecondBallsControl0X, waypoints.lineupSecondBallsControl0Y, 0);
    }

    private static Pose pickupSecondBalls() {
        return new Pose(waypoints.pickupSecondBallsX, waypoints.pickupSecondBallsY, Math.toRadians(waypoints.pickupSecondBallsHeading));
    }

    private static Pose shootSecondBalls() {
        return new Pose(waypoints.shootSecondBallsX, waypoints.shootSecondBallsY, Math.toRadians(waypoints.shootSecondBallsHeading));
    }

    private static Pose shootSecondBallsControl0() {
        return new Pose(waypoints.shootSecondBallsControl0X, waypoints.shootSecondBallsControl0Y, 0);
    }

    private static Pose lineupThirdBalls() {
        return new Pose(waypoints.lineupThirdBallsX, waypoints.lineupThirdBallsY, Math.toRadians(waypoints.lineupThirdBallsHeading));
    }

    private static Pose lineupThirdBallsControl0() {
        return new Pose(waypoints.lineupThirdBallsControl0X, waypoints.lineupThirdBallsControl0Y, 0);
    }

    private static Pose pickupThirdBalls() {
        return new Pose(waypoints.pickupThirdBallsX, waypoints.pickupThirdBallsY, Math.toRadians(waypoints.pickupThirdBallsHeading));
    }

    private static Pose shootThirdBalls() {
        return new Pose(waypoints.shootThirdBallsX, waypoints.shootThirdBallsY, Math.toRadians(waypoints.shootThirdBallsHeading));
    }

    private static Pose shootThirdBallsControl0() {
        return new Pose(waypoints.shootThirdBallsControl0X, waypoints.shootThirdBallsControl0Y, 0);
    }

    private static Command followPath(Robot robot, Alliance alliance, Pose startPose, Pose endPose, Pose... controlPoses) {
        // Mirror for red alliance
        Pose start = poseForAlliance(startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()), alliance);
        Pose end = poseForAlliance(endPose.getX(), endPose.getY(), Math.toDegrees(endPose.getHeading()), alliance);

        PathChain path;
        if (controlPoses.length > 0) {
            // Use BezierCurve with control points
            Pose control = poseForAlliance(controlPoses[0].getX(), controlPoses[0].getY(), 0, alliance);
            path = robot.drive.getFollower().pathBuilder()
                    .addPath(new com.pedropathing.geometry.BezierCurve(start, control, end))
                    .setLinearHeadingInterpolation(start.getHeading(), end.getHeading(), 0.7)
                    .build();
        } else {
            // Use straight line
            path = robot.drive.getFollower().pathBuilder()
                    .addPath(new BezierLine(start, end))
                    .setLinearHeadingInterpolation(start.getHeading(), end.getHeading(), 0.7)
                    .build();
        }
        double maxPower = Range.clip(config.maxPathPower, 0.0, 1.0);
        return new FollowPath(path, false, maxPower);
    }
    /**
     * Gets the default start pose from waypoints (before alliance mirroring).
     * This is the fallback when vision initialization is not available.
     * @return Default start pose
     */
    public static Pose getDefaultStartPose() {
        return start();
    }


}
