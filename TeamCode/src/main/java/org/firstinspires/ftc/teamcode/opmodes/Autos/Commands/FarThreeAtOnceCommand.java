package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AimAtBasketCommand;
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
        public double maxPathPower = 0.65;
        public double endTimeForLinearHeadingInterpolation = .7;
        public double intakeDelaySeconds = 3;
    }

    @Configurable
    public static class Waypoints {
        public double startX = 56;
        public double startY = 5;
        public double startHeading = 90.0;

        // LaunchFar
        public double launchFarX = 55;
        public double launchFarY = 12.3;
        public double launchFarHeadingDeg = 111;

        // Artifacts at Alliance Wall)
        public double artifactsWallX = 8;
        public double artifactsWallY = 7.0;
        public double artifactsWallHeading = 180;

        // Control point for segment: ArtifactsSet2
        public double leaveWallControlPointX = 27.5;
        public double leaveWallControlPointY = 18;

        // ArtifactsSet3
        public double artifactsSet3X = 23.75;
        public double artifactsSet3Y = 37.5;
        public double artifactsSet3Heading = 90;

        // Control point for segment: ArtifactsSet2
        public double artifactSet3ControlPointX = 23.75;
        public double artifactSet3ControlPointY = 10;

        // ArtifactsSet3
        public double artifactsSet2X = 23.75;
        public double artifactsSet2Y = 62;
        public double artifactsSet2Heading = 90;

        // Control point for segment: ArtifactsSet2
        public double artifactSet2ControlPointX = 23.75;
        public double artifactSet2ControlPointY = 25;

        // NearGate
        public double nearGateX = 35;
        public double nearGateY = 70.4;
        public double nearGateHeading = 180.0;

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
                                .to(launchFar())
                                .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                .build(config.maxPathPower),
                        launcherCommands.presetRangeSpinUp(LauncherRange.FAR_AUTO, true)
                ),
                new AimAtBasketCommand(robot.drive, robot.vision),  // <-- Add this

                launcherCommands.launchAll(false),

                // Pickup Alliance Wall Artifacts
                new ParallelGroup(
                        new FollowPathBuilder(robot, alliance)
                                .from(launchFar())
                                .to(artifactsAllianceWall())
                                .withConstantHeading(artifactsAllianceWall().getHeading())
                                .build(config.maxPathPower),
                        new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))
                ),

                // Launch Alliance Wall Artifacts
                new ParallelGroup(
                        new FollowPathBuilder(robot, alliance)
                                .from(artifactsAllianceWall())
                                .to( launchFar())
                                .withControl(leaveControl0())
                                .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                .build(config.maxPathPower),
                        new SequentialGroup(
                                new Delay(config.intakeDelaySeconds),
                                new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE))
                                )
                ),
                new AimAtBasketCommand(robot.drive, robot.vision),  // <-- Add this

                launcherCommands.launchAll(false),

                // Pickup Artifact Set 3
                new ParallelGroup(
                        new FollowPathBuilder(robot, alliance)
                                .from(launchFar())
                                .withControl(artifactsSet3Control0())
                                .to(artifactsSet3())
                                .withConstantHeading(artifactsSet3().getHeading())
                                .build(config.maxPathPower),
                        new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))
                ),
                new ParallelGroup(
                        new FollowPathBuilder(robot, alliance)
                            .from(artifactsSet3())
                            .to(launchFar())
                            .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                            .build(config.maxPathPower),
                        new SequentialGroup(
                                new Delay(config.intakeDelaySeconds),
                                new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE))
                        )
                ),
                new AimAtBasketCommand(robot.drive, robot.vision),  // <-- Add this

                launcherCommands.launchAll(false),

                // Pickup Artifact Set 2
                new ParallelGroup(
                        new FollowPathBuilder(robot, alliance)
                                .from(launchFar())
                                .withControl(artifactsSet2Control0())
                                .to(artifactsSet2())
                                .withConstantHeading(artifactsSet2().getHeading())
                                .build(config.maxPathPower),
                        new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))
                ),

                // LaunchOffLine
                new ParallelGroup(
                    new FollowPathBuilder(robot, alliance)
                        .from(artifactsSet2())
                        .to(launchFar())
                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                        .build(config.maxPathPower),
                       new SequentialGroup(
                                 new Delay(config.intakeDelaySeconds),
                                new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE))
                        )
                ),
                new AimAtBasketCommand(robot.drive, robot.vision),  // <-- Add this

                launcherCommands.launchAll(true),

                // Get Ready to Open Gate
                new FollowPathBuilder(robot, alliance)
                        .from(launchFar())
                        .to(nearGate())
                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                        .build(config.maxPathPower)
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



    private static Pose nearGate() {
        return new Pose(waypoints.nearGateX, waypoints.nearGateY, Math.toRadians(waypoints.nearGateHeading));
    }

}
