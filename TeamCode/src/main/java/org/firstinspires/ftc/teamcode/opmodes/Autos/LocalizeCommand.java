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
public class LocalizeCommand {

    @Configurable
    public static class Config {
        public double maxPathPower = 0.7;
        public double endTimeForLinearHeadingInterpolation = .7;
    }

    @Configurable
    public static class Waypoints {
        public double startX = 23.5;
        public double startY = 121.5;
        public double startHeading = 15;

        // LaunchClose1
        public double launchClose1X = 27.0;
        public double launchClose1Y = 116.0;
        public double launchClose1Heading = 134.0;

        // ArtifactsSet1
        public double artifactsSet1X = 24.0;
        public double artifactsSet1Y = 83.8;
        public double artifactsSet1Heading = 270.0;

        // LaunchClose2
        public double launchClose2X = 27.0;
        public double launchClose2Y = 116.0;
        public double launchClose2Heading = 134.0;

        // ArtifactsSet2
        public double artifactsSet2X = 24.0;
        public double artifactsSet2Y = 60.0;
        public double artifactsSet2Heading = 270;

        // Control point for segment: ArtifactsSet2
        public double artifactsSet2Control0X = 25.0;
        public double artifactsSet2Control0Y = 96.0;

        // LaunchClose3
        public double launchClose3X = 27.0;
        public double launchClose3Y = 116.0;
        public double launchClose3Heading = 134.0;

        // Control point for segment: LaunchClose3
        public double launchClose3Control0X = 33;
        public double launchClose3Control0Y = 84;

        // ArtifactsSet3
        public double artifactsSet3X = 24.0;
        public double artifactsSet3Y = 35.5;
        public double artifactsSet3Heading = 270.0;

        // Control point for segment: ArtifactsSet3
        public double artifactsSet3Control0X = 24;
        public double artifactsSet3Control0Y = 96.6;

        // LaunchOffLine
        public double launchOffLineX = 27.0;
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

    private LocalizeCommand() {}

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

        // Use vision-detected pose if available, otherwise use configured waypoints
        Pose effectiveStart = (startOverride != null) ? startOverride : start();

        return new SequentialGroup(
                // Launch Preloads
                new ParallelDeadlineGroup(
                        new FollowPathBuilder(robot, alliance)
                                .from(effectiveStart)
                                .to(launchClose1())
                                .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                .build(config.maxPathPower),

                        launcherCommands.presetRangeSpinUp(LauncherRange.SHORT, true)
                ),
                launcherCommands.launchAll(false),

                // Pickup Artifact Set 1
                new ParallelGroup(
                        new FollowPathBuilder(robot, alliance)
                                .from(launchClose1())
                                .to(artifactsSet1())
                                .withConstantHeading(artifactsSet1().getHeading())
                                .build(config.maxPathPower),
                        new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))
                ),

                new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE)),

                // Launch Artifact Set 1 - We should not need to spin up again since we never should have gone to idle
                new FollowPathBuilder(robot, alliance)
                        .from(artifactsSet1())
                        .to( launchClose2())
                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                        .build(config.maxPathPower),
                launcherCommands.launchAll(false),

                // Pickup Artifact Set 2
                new ParallelGroup(
                        new FollowPathBuilder(robot, alliance)
                                .from(launchClose2())
                                .withControl(artifactsSet2Control0())
                                .to(artifactsSet2())
                                .withConstantHeading(artifactsSet2().getHeading())
                                .build(config.maxPathPower),
                        new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))
                ),

                new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE)),

                // Launch Artifact Set 2
                new FollowPathBuilder(robot, alliance)
                        .from(artifactsSet2())
                        .to(launchClose3())
                        .withControl(launchClose3Control0())
                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                        .build(config.maxPathPower),
                launcherCommands.launchAll(false),


                // Pickup Artifact Set 3
                new ParallelGroup(
                        new FollowPathBuilder(robot, alliance)
                                .from(launchClose3())
                                .withControl(artifactsSet3Control0())
                                .to(artifactsSet3())
                                .withConstantHeading(artifactsSet3().getHeading())
                                .build(config.maxPathPower),
                        new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))
                ),
                new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE)),

                // LaunchOffLine
                new FollowPathBuilder(robot, alliance)
                        .from(artifactsSet3())
                        .to(launchOffLine())
                        .withControl(launchOffLineControl0())
                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                        .build(config.maxPathPower),
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

    private static Pose launchClose1() {
        return new Pose(waypoints.launchClose1X, waypoints.launchClose1Y, Math.toRadians(waypoints.launchClose1Heading));
    }

    private static Pose artifactsSet1() {
        return new Pose(waypoints.artifactsSet1X, waypoints.artifactsSet1Y, Math.toRadians(waypoints.artifactsSet1Heading));
    }

    private static Pose launchClose2() {
        return new Pose(waypoints.launchClose2X, waypoints.launchClose2Y, Math.toRadians(waypoints.launchClose2Heading));
    }

    private static Pose artifactsSet2() {
        return new Pose(waypoints.artifactsSet2X, waypoints.artifactsSet2Y, Math.toRadians(waypoints.artifactsSet2Heading));
    }

    private static Pose artifactsSet2Control0() {
        return new Pose(waypoints.artifactsSet2Control0X, waypoints.artifactsSet2Control0Y, 0);
    }

    private static Pose launchClose3() {
        return new Pose(waypoints.launchClose3X, waypoints.launchClose3Y, Math.toRadians(waypoints.launchClose3Heading));
    }

    private static Pose launchClose3Control0() {
        return new Pose(waypoints.launchClose3Control0X, waypoints.launchClose3Control0Y, 0);
    }

    private static Pose artifactsSet3() {
        return new Pose(waypoints.artifactsSet3X, waypoints.artifactsSet3Y, Math.toRadians(waypoints.artifactsSet3Heading));
    }

    private static Pose artifactsSet3Control0() {
        return new Pose(waypoints.artifactsSet3Control0X, waypoints.artifactsSet3Control0Y, 0);
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
