package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.FireAtPathTCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.AutoSmartIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.SetIntakeModeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.TimedEjectCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FollowPathBuilder;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;

/**
 * MichianaShort autonomous: fires all 3 lanes mid-path (at fireAtT) rather than stopping to shoot.
 *
 * Key difference from CloseThreeAtOnce:
 *   - Return paths use FireAtPathTCommand so shots queue when T >= fireAtT (heading already settled)
 *   - Flywheels stay at launch speed between shots (spinDownAfterShot=false handled by presetRangeSpinUp)
 *   - No sequential launch step between return path and next artifact pickup
 *
 * Waypoints mirror CloseThreeAtOnceCommand — adjust Config and Waypoints as needed.
 */
public class MichianaShortCommand {

    @Configurable
    public static class Config {
        /** Max drive power for normal paths */
        public static double maxPathPower = 0.8;
        /** Max drive power for the last paths (park etc.) */
        public static double lastPathsMaxPower = 1.0;
        /** linearInterpWeight: heading interpolation finishes at this T fraction */
        public static double headingInterpEnd = 0.7;
        /** T value at which shots fire on return paths — must be > headingInterpEnd */
        public static double fireAtT = 0.85;
        /** Full auto duration in seconds (for ConditionalFinalLaunch) */
        public static double autoDurationSeconds = 30.0;
        /** Min remaining seconds to attempt the 4th launch + park sequence */
        public static double minTimeForFinalLaunchSeconds = 6.8;
        /** Duration of eject pulse before each intake sweep */
        public static double ejectTime = 1200;
    }

    @Configurable
    public static class Waypoints {
        // Start
        public static double startX = 26.5;
        public static double startY = 130;
        public static double startHeading = 0;

        // Launch positions (same heading — robot faces goal)
        public static double launchX = 36;
        public static double launchY = 107.0;
        public static double launchHeading = 134.0;

        // Artifact Set 1
        public static double artifactsSet1X = 24;
        public static double artifactsSet1Y = 83.8;
        public static double artifactsSet1Heading = 270.0;
        public static double artifactsSet1CtrlX = 24;
        public static double artifactsSet1CtrlY = 113;

        // Artifact Set 2
        public static double artifactsSet2X = 24;
        public static double artifactsSet2Y = 61.0;
        public static double artifactsSet2Heading = 260;
        public static double artifactsSet2CtrlX = 24;
        public static double artifactsSet2CtrlY = 100;

        // Return from set 2 — control point to avoid obstacles
        public static double returnSet2CtrlX = 50.5;
        public static double returnSet2CtrlY = 72;

        // Artifact Set 3
        public static double artifactsSet3X = 24;
        public static double artifactsSet3Y = 35.5;
        public static double artifactsSet3Heading = 260;
        public static double artifactsSet3CtrlX = 24;
        public static double artifactsSet3CtrlY = 100;

        // Return from set 3 — control point
        public static double returnSet3CtrlX = 44.5;
        public static double returnSet3CtrlY = 73.5;

        // Park / near gate
        public static double nearGateX = 35;
        public static double nearGateY = 70.4;
        public static double nearGateHeading = 180.0;
        public static double nearGateCtrlX = 41;
        public static double nearGateCtrlY = 85;
    }

    private MichianaShortCommand() {}

    public static Pose getDefaultStartPose() {
        return start();
    }

    /**
     * Creates the command. Optionally pass a vision-detected start pose.
     */
    public static Command create(Robot robot, Alliance alliance) {
        return create(robot, alliance, null);
    }

    public static Command create(Robot robot, Alliance alliance, Pose startOverride) {
        LauncherCommands launcherCommands = new LauncherCommands(
                robot.launcher, robot.intake, robot.drive, robot.lighting);

        // --- First path: start → launch position, spin up during travel ---
        FollowPathBuilder firstPathBuilder = new FollowPathBuilder(robot, alliance);
        if (startOverride != null) {
            firstPathBuilder.fromWorldCoordinates(robot.drive.getFollower().getPose());
        } else {
            firstPathBuilder.from(start());
        }

        Command driveToFirstLaunch = new ParallelDeadlineGroup(
                firstPathBuilder
                        .to(launch())
                        .withLinearHeadingCompletion(Config.headingInterpEnd)
                        .build(Config.maxPathPower),
                new SetIntakeModeCommand(robot.intake, IntakeSubsystem.IntakeMode.PASSIVE_REVERSE),
                launcherCommands.presetRangeSpinUp(LauncherRange.SHORT_AUTO, true),
                // Fire preloads when heading is settled and we're approaching launch spot
                new FireAtPathTCommand(robot.drive.getFollower(), Config.fireAtT,
                        robot.launcher, robot.intake)
        );

        // --- Artifact Set 1 pickup ---
        Command pickupSet1 = new ParallelDeadlineGroup(
                new FollowPathBuilder(robot, alliance)
                        .from(launch())
                        .to(artifactsSet1())
                        .withControl(artifactsSet1Ctrl())
                        .withConstantHeading(270)
                        .build(Config.maxPathPower),
                new SequentialGroup(
                        new TimedEjectCommand(robot.intake, Config.ejectTime),
                        new AutoSmartIntakeCommand(robot.intake)
                )
        );

        // --- Return from set 1, fire at T=fireAtT ---
        Command returnAndFireSet1 = new ParallelDeadlineGroup(
                new FollowPathBuilder(robot, alliance)
                        .from(artifactsSet1())
                        .to(launch())
                        .withLinearHeadingCompletion(Config.headingInterpEnd)
                        .build(Config.maxPathPower),
                new FireAtPathTCommand(robot.drive.getFollower(), Config.fireAtT,
                        robot.launcher, robot.intake)
        );

        // --- Artifact Set 2 pickup ---
        Command pickupSet2 = new ParallelDeadlineGroup(
                new FollowPathBuilder(robot, alliance)
                        .from(launch())
                        .to(artifactsSet2())
                        .withControl(artifactsSet2Ctrl())
                        .withConstantHeading(270)
                        .build(Config.maxPathPower),
                new SequentialGroup(
                        new TimedEjectCommand(robot.intake, Config.ejectTime),
                        new AutoSmartIntakeCommand(robot.intake)
                )
        );

        // --- Return from set 2, fire at T=fireAtT (with control point) ---
        Command returnAndFireSet2 = new ParallelDeadlineGroup(
                new FollowPathBuilder(robot, alliance)
                        .from(artifactsSet2())
                        .to(launch())
                        .withControl(returnSet2Ctrl())
                        .withLinearHeadingCompletion(Config.headingInterpEnd)
                        .build(Config.maxPathPower),
                new FireAtPathTCommand(robot.drive.getFollower(), Config.fireAtT,
                        robot.launcher, robot.intake)
        );

        // --- Artifact Set 3 pickup ---
        Command pickupSet3 = new ParallelDeadlineGroup(
                new FollowPathBuilder(robot, alliance)
                        .from(launch())
                        .to(artifactsSet3())
                        .withControl(artifactsSet3Ctrl())
                        .withConstantHeading(270)
                        .build(Config.maxPathPower),
                new SequentialGroup(
                        new TimedEjectCommand(robot.intake, Config.ejectTime),
                        new AutoSmartIntakeCommand(robot.intake)
                )
        );

        // --- Conditional final sequence: return + fire + park if time permits ---
        Command finalSequence = new ConditionalFinalLaunchCommand(
                Config.autoDurationSeconds,
                Config.minTimeForFinalLaunchSeconds,
                // Enough time: return + fire mid-path + park
                new SequentialGroup(
                        new ParallelDeadlineGroup(
                                new FollowPathBuilder(robot, alliance)
                                        .from(artifactsSet3())
                                        .to(launch())
                                        .withControl(returnSet3Ctrl())
                                        .withLinearHeadingCompletion(Config.headingInterpEnd)
                                        .build(Config.lastPathsMaxPower),
                                new FireAtPathTCommand(robot.drive.getFollower(), Config.fireAtT,
                                        robot.launcher, robot.intake)
                        ),
                        new ParallelDeadlineGroup(
                                new FollowPathBuilder(robot, alliance)
                                        .from(launch())
                                        .to(nearGate())
                                        .withControl(nearGateCtrl())
                                        .withLinearHeadingCompletion(Config.headingInterpEnd)
                                        .build(Config.lastPathsMaxPower),
                                new SequentialGroup(
                                        new TimedEjectCommand(robot.intake, Config.ejectTime),
                                        new AutoSmartIntakeCommand(robot.intake)
                                )
                        )
                ),
                // Not enough time: go straight to park
                new FollowPathBuilder(robot, alliance)
                        .from(artifactsSet3())
                        .to(nearGate())
                        .withLinearHeadingCompletion(Config.headingInterpEnd)
                        .build(Config.maxPathPower)
        );

        return new SequentialGroup(
                ConditionalFinalLaunchCommand.createTimerReset(),
                driveToFirstLaunch,
                pickupSet1,
                returnAndFireSet1,
                pickupSet2,
                returnAndFireSet2,
                pickupSet3,
                finalSequence
        );
    }

    // ---- Pose helpers ----

    private static Pose start() {
        return new Pose(Waypoints.startX, Waypoints.startY,
                Math.toRadians(Waypoints.startHeading));
    }

    private static Pose launch() {
        return new Pose(Waypoints.launchX, Waypoints.launchY,
                Math.toRadians(Waypoints.launchHeading));
    }

    private static Pose artifactsSet1() {
        return new Pose(Waypoints.artifactsSet1X, Waypoints.artifactsSet1Y,
                Math.toRadians(Waypoints.artifactsSet1Heading));
    }

    private static Pose artifactsSet1Ctrl() {
        return new Pose(Waypoints.artifactsSet1CtrlX, Waypoints.artifactsSet1CtrlY, 0);
    }

    private static Pose artifactsSet2() {
        return new Pose(Waypoints.artifactsSet2X, Waypoints.artifactsSet2Y,
                Math.toRadians(Waypoints.artifactsSet2Heading));
    }

    private static Pose artifactsSet2Ctrl() {
        return new Pose(Waypoints.artifactsSet2CtrlX, Waypoints.artifactsSet2CtrlY, 0);
    }

    private static Pose returnSet2Ctrl() {
        return new Pose(Waypoints.returnSet2CtrlX, Waypoints.returnSet2CtrlY, 0);
    }

    private static Pose artifactsSet3() {
        return new Pose(Waypoints.artifactsSet3X, Waypoints.artifactsSet3Y,
                Math.toRadians(Waypoints.artifactsSet3Heading));
    }

    private static Pose artifactsSet3Ctrl() {
        return new Pose(Waypoints.artifactsSet3CtrlX, Waypoints.artifactsSet3CtrlY, 0);
    }

    private static Pose returnSet3Ctrl() {
        return new Pose(Waypoints.returnSet3CtrlX, Waypoints.returnSet3CtrlY, 0);
    }

    private static Pose nearGate() {
        return new Pose(Waypoints.nearGateX, Waypoints.nearGateY,
                Math.toRadians(Waypoints.nearGateHeading));
    }

    private static Pose nearGateCtrl() {
        return new Pose(Waypoints.nearGateCtrlX, Waypoints.nearGateCtrlY, 0);
    }
}
