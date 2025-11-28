package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

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

/**
 * Generated autonomous command from Pedro Pathing .pp file
 * Usage in OpMode:
 *   Command auto = LocalizeCommand.create(robot, activeAlliance);
 *   CommandManager.INSTANCE.scheduleCommand(auto);
 * 
 */
@Configurable
public class FarThreeAtOnceCommand {

    @Configurable
    public static class Config {
        public double maxPathPower = 0.7;
        public double endTimeForLinearHeadingInterpolation = .7;
        public double intakeDelaySeconds = 3;
    }

    @Configurable
    public static class Waypoints {
        public double startX = 56;
        public double startY = 6;
        public double startHeading = 90.0;

        // LaunchFar1
        public double launchFarX = 55;
        public double launchFarY = 17.3;
        public double launchFarHeadingDeg = 111;

        // ArtifactsSet1
        public double artifactsSet4X = 9;
        public double artifactsSet4Y = 8.2;
        public double artifactsSet4Heading = 180;

        // LaunchFar2
        public double launchFarX2 = 55;
        public double launchFarY2 = 17.3;
        public double launchFarHeadingDeg2 = 111;

        // ArtifactsSet2
        public double artifactsSet2X = 27;
        public double artifactsSet2Y = 33;
        public double artifactsSet2Heading = 90;

        // Control point for segment: ArtifactsSet2
        public double artifactSet2ControlPointX = 28;
        public double artifactSet2ControlPointY = 2;

        // LaunchFar3
        public double launchFarX3 = 55;
        public double launchFarY3 = 17.3;
        public double launchFarHeadingDeg3 = 111;

        // Control point for segment: LaunchFar3
  

        // ArtifactsSet3
        public double artifactsSet3X = 28;
        public double artifactsSet3Y = 56;
        public double artifactsSet3Heading = 90;

        // Control point for segment: ArtifactsSet3
 

        // LaunchOffLine
        public double launchOffLineX = 28;
        public double launchOffLineY = 116.0;
        public double launchOffLineHeading = 134.0;

        // Control point for segment: LaunchOffLine
        public double launchOffLineControl0X = 44.5;
        public double launchOffLineControl0Y = 73.5;

        // NearGate
        public double nearGateX = 35;
        public double nearGateY = 70.4;
        public double nearGateHeading = 180.0;

        // Control point for segment: NearGate
        public double nearGateControl0X = 41;
        public double nearGateControl0Y = 85;

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
                                .to(launchFar1())
                                .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                .build(config.maxPathPower),

                        launcherCommands.presetRangeSpinUp(LauncherRange.LONG, true)
                ),
                launcherCommands.launchAll(false),

                // Pickup Artifact Set 1
                new ParallelGroup(
                        new FollowPathBuilder(robot, alliance)
                                .from(launchFar1())
                                .to(artifactsSet4())
                                .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                .build(config.maxPathPower),
                        new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))
                ),


                new ParallelGroup(
                        new FollowPathBuilder(robot, alliance)
                                .from(artifactsSet4())
                                .to( launchFar2())
                                .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                .build(config.maxPathPower),
                        new SequentialGroup(
                                new Delay(config.intakeDelaySeconds),
                                new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE))
                                )
                ),
                // Launch Artifact Set 1 - We should not need to spin up again since we never should have gone to idle

                launcherCommands.launchAll(false),

                // Pickup Artifact Set 2
                new ParallelGroup(
                        new FollowPathBuilder(robot, alliance)
                                .from(launchFar2())
                                .withControl(artifactsSet2Control0())
                                .to(artifactsSet2())
                                .withConstantHeading(artifactsSet2().getHeading())
                                .build(config.maxPathPower),
                        new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))
                ),
                new ParallelGroup(
                        new FollowPathBuilder(robot, alliance)
                            .from(artifactsSet2())
                            .to(launchFar3())
                            .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                            .build(config.maxPathPower),
                        new SequentialGroup(
                                new Delay(config.intakeDelaySeconds),
                                new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE))
                        )
                ),

                launcherCommands.launchAll(false),


                // Pickup Artifact Set 3
                new ParallelGroup(
                        new FollowPathBuilder(robot, alliance)
                                .from(launchFar3())
                                .to(artifactsSet3())
                                .withConstantHeading(artifactsSet3().getHeading())
                                .build(config.maxPathPower),
                        new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))
                ),

                // LaunchOffLine
                new ParallelGroup(
                    new FollowPathBuilder(robot, alliance)
                        .from(artifactsSet3())
                        .to(launchOffLine())
                        .withControl(launchOffLineControl0())
                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                        .build(config.maxPathPower),
                       new SequentialGroup(
                                 new Delay(config.intakeDelaySeconds),
                                new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE))
                        )
                ),
                launcherCommands.launchAll(true),

                // Get Ready to Open Gate
                new FollowPathBuilder(robot, alliance)
                        .from(launchOffLine())
                        .to(nearGate())
                        .withControl(nearGateControl0())
                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                        .build(config.maxPathPower)
        );
    }

    private static Pose start() {
        return new Pose(waypoints.startX, waypoints.startY, Math.toRadians(waypoints.startHeading));
    }

    private static Pose launchFar1() {
        return new Pose(waypoints.launchFarX, waypoints.launchFarY, Math.toRadians(waypoints.launchFarHeadingDeg));
    }

    private static Pose artifactsSet4() {
        return new Pose(waypoints.artifactsSet4X, waypoints.artifactsSet4Y, Math.toRadians(waypoints.artifactsSet4Heading));
    }

    private static Pose launchFar2() {
        return new Pose(waypoints.launchFarX2, waypoints.launchFarY2, Math.toRadians(waypoints.launchFarHeadingDeg));
    }

    private static Pose artifactsSet2() {
        return new Pose(waypoints.artifactsSet2X, waypoints.artifactsSet2Y, Math.toRadians(waypoints.artifactsSet2Heading));
    }

    private static Pose artifactsSet2Control0() {
        return new Pose(waypoints.artifactSet2ControlPointX, waypoints.artifactSet2ControlPointY, 0);
    }

    private static Pose launchFar3() {
        return new Pose(waypoints.launchFarX3, waypoints.launchFarY3, Math.toRadians(waypoints.launchFarHeadingDeg));
    }



    private static Pose artifactsSet3() {
        return new Pose(waypoints.artifactsSet3X, waypoints.artifactsSet3Y, Math.toRadians(waypoints.artifactsSet3Heading));
    }

  

    private static Pose launchOffLine() {
        return new Pose(waypoints.launchOffLineX, waypoints.launchOffLineY, Math.toRadians(waypoints.launchOffLineHeading));
    }

    private static Pose launchOffLineControl0() {
        return new Pose(waypoints.launchOffLineControl0X, waypoints.launchOffLineControl0Y, 0);
    }

    private static Pose nearGate() {
        return new Pose(waypoints.nearGateX, waypoints.nearGateY, Math.toRadians(waypoints.nearGateHeading));
    }

    private static Pose nearGateControl0() {
        return new Pose(waypoints.nearGateControl0X, waypoints.nearGateControl0Y, 0);
    }
}
