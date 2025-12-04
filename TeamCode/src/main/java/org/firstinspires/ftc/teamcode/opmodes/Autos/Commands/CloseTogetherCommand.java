package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;
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
public class CloseTogetherCommand {

    @Configurable
    public static class Config {
        public double maxPathPower = .8;
        public double endTimeForLinearHeadingInterpolation = .7;
        public double secondsOpeningGate = .5;
        public double autoDurationSeconds = 30.0;
        public double minTimeForFinalLaunchSeconds = 5.0;
    }

    @Configurable
    public static class Waypoints {
        public double startX = 26.5;
        public double startY = 130;
        public double startHeading = 0;

        // LaunchClose1
        public double launchClose1X = 30.0;
        public double launchClose1Y = 113.0;
        public double launchClose1Heading = 128;

        // ArtifactsSet1
        public double artifactsSet1X = 23.75;
        public double artifactsSet1Y = 83.8;
        public double artifactsSet1Heading = 270.0;

        // Control point for segment: ArtifactsSet3
        public double artifactsSet1Control0X = 23.75;
        public double artifactsSet1Control0Y = 113;

        // OpenGate
        public double openGateX = 17;
        public double openGateY = 82;
        public double openGateHeading = 270;

        public double openGateControlX = 27;
        public double openGateControlY = 82;

        // OpenGate
        public double openGateStrafeX = 30;
        public double openGateStrafeY = 81;
        public double openGateStrafeHeading = 270;

        // LaunchClose2
        public double launchClose2X = 30.0;
        public double launchClose2Y = 113.0;
        public double launchClose2Heading = 134.0;

        public double launchClose2ControlX = 47;
        public double launchClose2ControlY = 80;

        // ArtifactsSet2
        public double artifactsSet2X = 23.25;
        public double artifactsSet2Y = 61.0;
        public double artifactsSet2Heading = 270;

        // Control point for segment: ArtifactsSet2
        public double artifactsSet2Control0X = 23.25;
        public double artifactsSet2Control0Y = 70;

        // LaunchClose3
        public double launchClose3X = 30.0;
        public double launchClose3Y = 113.0;
        public double launchClose3Heading = 134.0;

        // Control point for segment: LaunchClose3
        public double launchClose3Control0X = 50.5;
        public double launchClose3Control0Y = 72;

//        // ReleasedArtifacts
//        public double releasedArtifactsX = 13;
//        public double releasedArtifactsY = 54.25;
//        public double releasedArtifactsHeading = 180;
//
//        // Control point for segment: ArtifactsSet3
//        public double releasedArtifactsControlX = 51;
//        public double releasedArtifactsControlY = 59;

        // ArtifactsSet3
        public double artifactsSet3X = 23.75;
        public double artifactsSet3Y = 35.5;
        public double artifactsSet3Heading = 270.0;

        // Control point for segment: ArtifactsSet3
        public double artifactsSet3Control0X = 23.75;
        public double artifactsSet3Control0Y = 72;

        // LaunchReleased
        public double launchClose4X = 30;
        public double launchClose4Y = 113.0;
        public double launchClose4Heading = 134.0;

        // Control point for segment: LaunchOffLine
        public double launchClose4ControlX = 44;
        public double launchClose4ControlY = 55;

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

    private CloseTogetherCommand() {}

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

                new ParallelDeadlineGroup(
                        // Launch Preloads
                        firstPathBuilder
                                .to(launchClose1())
                                .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                .build(config.maxPathPower),
                        launcherCommands.presetRangeSpinUp(LauncherRange.SHORT_AUTO, true) // Spin up to SHORT RPM for the whole auto
                ),

//                new AimAtGoalCommand(robot.drive, robot.vision),
                launcherCommands.launchAccordingToMode(false),

                // Pickup Artifact Set 1
                new FollowPathBuilder(robot, alliance)
                        .from(launchClose1())
                        .to(artifactsSet1())
                        .withControl(artifactsSet1Control0())
                        .withConstantHeading(270)
                        .build(config.maxPathPower),

                // Open Gate
                new FollowPathBuilder(robot, alliance)
                        .from(artifactsSet1())
                        .to(openGate())
                        .withControl(openGateControl())
                        .withConstantHeading(270)
                        .build(config.maxPathPower),

                new Delay(config.secondsOpeningGate), //Delay to open the gate

                // Return and Launch Set 1
                new FollowPathBuilder(robot, alliance)
                        .from(openGate())
                        .to(openGateStrafePoint())
                        .withConstantHeading(270)
                        .build(config.maxPathPower),

                // Return and Launch Set 1
                new FollowPathBuilder(robot, alliance)
                        .from(openGateStrafePoint())
                        .to(launchClose2())
                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                        .build(config.maxPathPower),

//                new AimAtGoalCommand(robot.drive, robot.vision),
                launcherCommands.launchAccordingToMode(false),

                // Pickup Artifact Set 2
                new FollowPathBuilder(robot, alliance)
                        .from(launchClose2())
                        .to(artifactsSet2())
                        .withControl(artifactsSet2Control0())
                        .withConstantHeading(270)
                        .build(config.maxPathPower),

                // Return and Launch Set 2
                new FollowPathBuilder(robot, alliance)
                        .from(artifactsSet2())
                        .to(launchClose3())
                        .withControl(launchClose3Control0())
                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                        .build(config.maxPathPower),

//                new AimAtGoalCommand(robot.drive, robot.vision),
                launcherCommands.launchAccordingToMode(false),

                // Pickup Artifact Set 3
                new FollowPathBuilder(robot, alliance)
                        .from(launchClose3())
                        .to(artifactsSet3())
                        .withControl(artifactsSet3Control0())
                        .withConstantHeading(270)
                        .build(config.maxPathPower),

                // Conditionally return and launch if time permits, otherwise go straight to park
                new ConditionalFinalLaunchCommand(
                        config.autoDurationSeconds,
                        config.minTimeForFinalLaunchSeconds,
                        // If enough time: return to launch, shoot, then park
                        new SequentialGroup(
                                new FollowPathBuilder(robot, alliance)
                                        .from(artifactsSet3())
                                        .to(launchClose4())
                                        .withControl(launchClose4Control())
                                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                        .build(config.maxPathPower),

                                launcherCommands.launchAccordingToMode(false),

                                new FollowPathBuilder(robot, alliance)
                                        .from(launchClose4())
                                        .to(nearGate())
                                        .withControl(nearGateControl0())
                                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                        .build(config.maxPathPower)
                        ),
                        // If not enough time: go straight to park
                        new SequentialGroup(
                                new FollowPathBuilder(robot, alliance)
                                        .from(artifactsSet3())
                                        .to(nearGate())
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

    private static Pose launchClose1() {
        return new Pose(waypoints.launchClose1X, waypoints.launchClose1Y, Math.toRadians(waypoints.launchClose1Heading));
    }

    private static Pose artifactsSet1() {
        return new Pose(waypoints.artifactsSet1X, waypoints.artifactsSet1Y, Math.toRadians(waypoints.artifactsSet1Heading));
    }

    private static Pose artifactsSet1Control0() {
        return new Pose(waypoints.artifactsSet1Control0X, waypoints.artifactsSet1Control0Y, 0);
    }

    private static Pose openGate() {
        return new Pose(waypoints.openGateX, waypoints.openGateY, Math.toRadians(waypoints.openGateHeading));
    }

    private static Pose openGateControl() {
        return new Pose(waypoints.openGateControlX, waypoints.openGateControlY, 0);
    }

    private static Pose openGateStrafePoint() {
        return new Pose(waypoints.openGateStrafeX, waypoints.openGateStrafeY, Math.toRadians(waypoints.openGateStrafeHeading));
    }

    private static Pose launchClose2() {
        return new Pose(waypoints.launchClose2X, waypoints.launchClose2Y, Math.toRadians(waypoints.launchClose2Heading));
    }

    private static Pose launchClose2Control() {
        return new Pose(waypoints.launchClose2ControlX, waypoints.launchClose2ControlY, 0);
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

//    private static Pose releasedArtifacts() {
//        return new Pose(waypoints.releasedArtifactsX , waypoints.releasedArtifactsY , Math.toRadians(waypoints.releasedArtifactsHeading));
//    }
//
//    private static Pose releasedArtifactsControl() {
//        return new Pose(waypoints.releasedArtifactsControlX , waypoints.releasedArtifactsControlY , 0);
//    }

    private static Pose artifactsSet3() {
        return new Pose(waypoints.artifactsSet3X, waypoints.artifactsSet3Y, Math.toRadians(waypoints.artifactsSet3Heading));
    }

    private static Pose artifactsSet3Control0() {
        return new Pose(waypoints.artifactsSet3Control0X, waypoints.artifactsSet3Control0Y, 0);
    }


    private static Pose launchClose4() {
        return new Pose(waypoints.launchClose4X, waypoints.launchClose4Y, Math.toRadians(waypoints.launchClose4Heading));
    }

    private static Pose launchClose4Control() {
        return new Pose(waypoints.launchClose4ControlX, waypoints.launchClose4ControlY, 0);
    }

    private static Pose nearGate() {
        return new Pose(waypoints.nearGateX, waypoints.nearGateY, Math.toRadians(waypoints.nearGateHeading));
    }

    private static Pose nearGateControl0() {
        return new Pose(waypoints.nearGateControl0X, waypoints.nearGateControl0Y, 0);
    }

}
