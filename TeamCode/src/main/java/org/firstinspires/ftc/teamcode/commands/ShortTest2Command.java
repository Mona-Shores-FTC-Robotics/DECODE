package org.firstinspires.ftc.teamcode.commands;

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
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;

/**
 * Generated autonomous command from Pedro Pathing .pp file
 * 
 * Usage in OpMode:
 *   Command auto = ShortTest2Command.create(robot, activeAlliance);
 *   CommandManager.INSTANCE.scheduleCommand(auto);
 * 
 * TODO: Fill in robot actions at each segment (search for TODO comments)
 */
@Configurable
public class ShortTest2Command {

    @Configurable
    public static class Config {
        public double maxPathPower = 0.79;
        public double intakeDelaySeconds = 2.5;
    }

    @Configurable
    public static class Waypoints {
        public double startX = 24.56872037914692;
        public double startY = 130.18009478672985;
        public double startHeading = 90.0;

        // LaunchClose
        public double launchCloseX = 30.19905213270142;
        public double launchCloseY = 112.9478672985782;
        public double launchCloseHeading = 134.0;

        // PreGateArtifacts
        public double preGateArtifactsX = 23.0;
        public double preGateArtifactsY = 108.0;
        public double preGateArtifactsHeading = 270.0;

        // GateArtifacts
        public double gateArtifactsX = 23.5;
        public double gateArtifactsY = 87.0;
        public double gateArtifactsHeading = 270.0;

        // PreSet2
        public double preSet2X = 26.0;
        public double preSet2Y = 82.0;
        public double preSet2Heading = 270.0;

        // Set2
        public double set2X = 24.0;
        public double set2Y = 58.0;
        public double set2Heading = 270.0;

        // Launch
        public double launchX = 30.19905213270142;
        public double launchY = 112.9478672985782;
        public double launchHeading = 134.0;

        // Pre Set 3
        public double preSet3X = 26.0;
        public double preSet3Y = 62.0;
        public double preSet3Heading = 270.0;

        // set3
        public double set3X = 23.5;
        public double set3Y = 32.5;
        public double set3Heading = 270.0;

        // Control point for segment: Launch
        public double launchControl0X = 39.0;
        public double launchControl0Y = 84.5;

        // Move to gate
        public double moveToGateX = 30.0;
        public double moveToGateY = 70.0;
        public double moveToGateHeading = 180.0;

    }

    public static Config config = new Config();
    public static Waypoints waypoints = new Waypoints();

    private ShortTest2Command() {}

    public static Command create(Robot robot, Alliance alliance) {
        return new SequentialGroup(
                // LaunchClose
                new ParallelGroup(
                    followPath(robot, alliance, start(), launchClose()),
                    // TODO: Customize launcher for LaunchClose
                    spinUpLauncher(robot)
                ),
                // TODO: Customize scoring for LaunchClose
                scoreSequence(robot),
                // PreGateArtifacts
                new ParallelGroup(
                    followPath(robot, alliance, launchClose(), preGateArtifacts()),
                    // TODO: Customize intake for PreGateArtifacts
                    new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))
                ),
                new Delay(config.intakeDelaySeconds),
                // GateArtifacts
                new ParallelGroup(
                    followPath(robot, alliance, preGateArtifacts(), gateArtifacts()),
                    // TODO: Customize intake for GateArtifacts
                    new InstantCommand(() -> robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))
                ),
                new Delay(config.intakeDelaySeconds),
                // LaunchClose
                new ParallelGroup(
                    followPath(robot, alliance, gateArtifacts(), launchClose()),
                    // TODO: Customize launcher for LaunchClose
                    spinUpLauncher(robot)
                ),
                // TODO: Customize scoring for LaunchClose
                scoreSequence(robot),
                // PreSet2
                followPath(robot, alliance, launchClose(), preSet2()),
                // TODO: Add commands for PreSet2
                // Set2
                followPath(robot, alliance, preSet2(), set2()),
                // TODO: Add commands for Set2
                // Launch
                new ParallelGroup(
                    followPath(robot, alliance, set2(), launch()),
                    // TODO: Customize launcher for Launch
                    spinUpLauncher(robot)
                ),
                // TODO: Customize scoring for Launch
                scoreSequence(robot),
                // Pre Set 3
                followPath(robot, alliance, launch(), preSet3()),
                // TODO: Add commands for Pre Set 3
                // set3
                followPath(robot, alliance, preSet3(), set3()),
                // TODO: Add commands for set3
                // Launch
                new ParallelGroup(
                    followPath(robot, alliance, set3(), launch(), launchControl0()),
                    // TODO: Customize launcher for Launch
                    spinUpLauncher(robot)
                ),
                // TODO: Customize scoring for Launch
                scoreSequence(robot),
                // Move to gate
                followPath(robot, alliance, launch(), moveToGate())
                // TODO: Add commands for Move to gate
        );
    }

    private static Pose start() {
        return new Pose(waypoints.startX, waypoints.startY, Math.toRadians(waypoints.startHeading));
    }

    private static Pose launchClose() {
        return new Pose(waypoints.launchCloseX, waypoints.launchCloseY, Math.toRadians(waypoints.launchCloseHeading));
    }

    private static Pose preGateArtifacts() {
        return new Pose(waypoints.preGateArtifactsX, waypoints.preGateArtifactsY, Math.toRadians(waypoints.preGateArtifactsHeading));
    }

    private static Pose gateArtifacts() {
        return new Pose(waypoints.gateArtifactsX, waypoints.gateArtifactsY, Math.toRadians(waypoints.gateArtifactsHeading));
    }

    private static Pose preSet2() {
        return new Pose(waypoints.preSet2X, waypoints.preSet2Y, Math.toRadians(waypoints.preSet2Heading));
    }

    private static Pose set2() {
        return new Pose(waypoints.set2X, waypoints.set2Y, Math.toRadians(waypoints.set2Heading));
    }

    private static Pose launch() {
        return new Pose(waypoints.launchX, waypoints.launchY, Math.toRadians(waypoints.launchHeading));
    }

    private static Pose preSet3() {
        return new Pose(waypoints.preSet3X, waypoints.preSet3Y, Math.toRadians(waypoints.preSet3Heading));
    }

    private static Pose set3() {
        return new Pose(waypoints.set3X, waypoints.set3Y, Math.toRadians(waypoints.set3Heading));
    }

    private static Pose launchControl0() {
        return new Pose(waypoints.launchControl0X, waypoints.launchControl0Y, 0);
    }

    private static Pose moveToGate() {
        return new Pose(waypoints.moveToGateX, waypoints.moveToGateY, Math.toRadians(waypoints.moveToGateHeading));
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

    private static Command spinUpLauncher(Robot robot) {
        // TODO: Customize launcher spin-up
        LauncherCommands launcherCommands = new LauncherCommands(robot.launcher, robot.intake);
        return launcherCommands.presetRangeSpinUp(LauncherRange.SHORT, true);
    }

    private static Command scoreSequence(Robot robot) {
        // TODO: Customize scoring sequence
        LauncherCommands launcherCommands = new LauncherCommands(robot.launcher, robot.intake);
        return launcherCommands.launchAll(true);
    }
}
