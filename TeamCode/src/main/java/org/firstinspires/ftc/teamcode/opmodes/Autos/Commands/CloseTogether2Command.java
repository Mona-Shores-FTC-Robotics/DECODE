package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.ModeAwareLaunchCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.PresetRangeSpinCommand;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FollowPathBuilder;
import org.firstinspires.ftc.teamcode.util.IntakeMode;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

/**
 * Close-side "together" auto, variant 2: two gate pushes, two pickup-and-shoot cycles, park.
 *
 * Sequence:
 *   1. Shoot preload          (drive to launchClose1, launch)
 *   2. Pick up artifact set 1
 *   3. Open gate #1           (push openGate, bounded by timeout)
 *   4. Shoot set 1            (return to launchClose2, launch)
 *   5. Pick up artifact set 2
 *   6. Open gate #2           (push openGate2, bounded by timeout)   <-- the added push
 *   7. Shoot set 2            (return to launchClose3, launch)
 *   8. Park                   (drive to nearGate)
 *
 * This is a copy of {@link CloseTogetherCommand} that adds the second gate push and
 * ends after the second set (no set-3 pickup / conditional 4th launch). openGate2 is
 * a separate, independently-tunable waypoint since it's approached from set 2.
 */
@Configurable
public class CloseTogether2Command {

    public static class Config {
        public double maxPathPower = .80;
        public double endTimeForLinearHeadingInterpolation = .80;
        public double secondsOpeningGate = .4;
        /** Hard cap (ms) on each gate-push move. The robot drives into the gate and may
         *  never reach the path endpoint, so Follow's !isBusy() completion never fires —
         *  this timeout ends the push so the auto can't hang there. Tune alongside the
         *  openGate waypoints: enough time to reach the gate and shove it open. */
        public double gatePushTimeoutMs = 1500;
    }

    public static class Waypoints {
        public double startX = 26.5;
        public double startY = 130;
        public double startHeading = 0;

        // LaunchClose1
        public double launchClose1X = 36;
        public double launchClose1Y = 107;
        public double launchClose1Heading = 140;

        // ArtifactsSet1 — pulled toward the wall (lower X) so the intake reaches the
        // wall-side artifact instead of clipping the row. Control shares the endpoint X
        // so the robot translates over to the wall line at the TOP, then comes straight
        // down in Y (rather than cutting the row diagonally and missing the wall one).
        public double artifactsSet1X = 21;
        public double artifactsSet1Y = 82;
        public double artifactsSet1Heading = 270.0;

        public double artifactsSet1Control0X = 20;
        public double artifactsSet1Control0Y = 120;

        // OpenGate #1 — control sits midway between artifactsSet1 and the gate, on the line.
        public double openGateX = 11;
        public double openGateY = 80;
        public double openGateHeading = 270;
        public double openGateControlX = 20;
        public double openGateControlY = 80;

        // LaunchClose2
        public double launchClose2X = 36;
        public double launchClose2Y = 107;
        public double launchClose2Heading = 140;

        // ArtifactsSet2
        public double artifactsSet2X = 21;
        public double artifactsSet2Y = 61.0;
        public double artifactsSet2Heading = 270;

        public double artifactsSet2Control0X = 21;
        public double artifactsSet2Control0Y = 100;

        // OpenGate #2 — same physical gate, approached from set 2 (lower Y). Control sits
        // midway between artifactsSet2 (24, 61) and the gate, on the line. Tune independently.
        public double openGate2X = 11;
        public double openGate2Y = 78;
        public double openGate2Heading = 270;
        public double openGate2ControlX = 17;
        public double openGate2ControlY = 78;

        // Gate #2 staging — sits ~5 inches off the gate toward the field (gate is at x=11,
        // this is x=16). The robot routes through here BOTH ways: on the way in it drives
        // here first, THEN pushes straight left into the gate (so it stops backing into it);
        // on the way out it backs off to here off the wall, THEN rotates to launch (so it
        // stops turning into it). Move this point to set how far / which way it goes
        // before and after the push.
        public double gate2StageX = 16;
        public double gate2StageY = 78;
        public double gate2StageHeading = 270;

        // LaunchClose3
        public double launchClose3X = 36;
        public double launchClose3Y = 107;
        public double launchClose3Heading = 140;

        // NearGate (park)
        public double nearGateX = 35;
        public double nearGateY = 70.4;
        public double nearGateHeading = 180.0;

        public double nearGateControl0X = 41;
        public double nearGateControl0Y = 85;
    }

    public static Config config = new Config();
    public static Waypoints waypoints = new Waypoints();

    private CloseTogether2Command() {}

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
        // Build first path: start -> launch position
        FollowPathBuilder firstPathBuilder = new FollowPathBuilder(robot, alliance);
        if (startOverride != null) {
            // Vision detected: use follower's current pose (world coordinates, no mirroring)
            firstPathBuilder.fromWorldCoordinates(robot.drive.getFollower().getPose());
        } else {
            // No vision: use waypoint start pose (will be mirrored for red alliance)
            firstPathBuilder.from(start());
        }

        return Groups.sequential(
                // Reset timer when auto actually starts (not when command is created)
                ConditionalFinalLaunchCommand.createTimerReset(),

                // Pre-spin the flywheels to speed while stationary BEFORE the first drive.
                // Three flywheels can't pull their startup inrush while the drive also pulls
                // hard off the line, so spinning up during the first path browns out the
                // launcher and it never starts. Get them up first, then move (the launcher
                // periodic holds the RPM target for the rest of the routine).
                PresetRangeSpinCommand.create(
                        robot.launcher, LauncherRange.SHORT_AUTO, true,
                        robot.drive, robot.lighting, null),

                // 1. Shoot preload
                Groups.deadline(
                        firstPathBuilder
                                .to(launchClose1())
                                .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                                .build(config.maxPathPower),
                        robot.intake.setIntakeModeCmd(IntakeMode.PASSIVE_REVERSE),
                        PresetRangeSpinCommand.create(
                                robot.launcher, LauncherRange.SHORT_AUTO, true,
                                robot.drive, robot.lighting, null) // Hold SHORT RPM for the whole auto
                ),
                ModeAwareLaunchCommand.create(robot.launcher, robot.intake, false),

                // 2. Pick up artifact set 1
                Groups.deadline(
                        new FollowPathBuilder(robot, alliance)
                                .from(launchClose1())
                                .to(artifactsSet1())
                                .withControl(artifactsSet1Control0())
                                .withConstantHeading(270)
                                .build(config.maxPathPower),
                        robot.intake.autoSmartIntakeCmd()
                ),

                // 3. Open gate #1 — bounded by a timeout so the gate-push can't hang the auto.
                Groups.race(
                        new FollowPathBuilder(robot, alliance)
                                .from(artifactsSet1())
                                .to(openGate())
                                .withControl(openGateControl())
                                .withConstantHeading(270)
                                .build(config.maxPathPower),
                        Commands.waitMs(config.gatePushTimeoutMs)
                ),
                Commands.waitMs(config.secondsOpeningGate * 1000.0), // dwell so the gate opens

                // 4. Shoot set 1 (return to launch; constant 270 then turn to launch heading)
                new FollowPathBuilder(robot, alliance)
                        .from(openGate())
                        .to(launchClose2())
                        .withPiecewiseConstantThenLinear(270, 0.2, waypoints.launchClose2Heading)
                        .build(config.maxPathPower),
                ModeAwareLaunchCommand.create(robot.launcher, robot.intake, false),

                // 5. Pick up artifact set 2
                Groups.deadline(
                        new FollowPathBuilder(robot, alliance)
                                .from(launchClose2())
                                .to(artifactsSet2())
                                .withControl(artifactsSet2Control0())
                                .withConstantHeading(270)
                                .build(config.maxPathPower),
                        robot.intake.autoSmartIntakeCmd()
                ),

                // 6. Open gate #2 — stage off the gate first, THEN push straight left into it.
                //    Drive to the staging point (forward, off the wall), then push left into
                //    the gate. The push is bounded by the timeout so it can't hang. Routing
                //    through the stage stops the robot backing straight into the gate.
                new FollowPathBuilder(robot, alliance)
                        .from(artifactsSet2())
                        .to(gate2Stage())
                        .withConstantHeading(270)
                        .build(config.maxPathPower),
                Groups.race(
                        new FollowPathBuilder(robot, alliance)
                                .from(gate2Stage())
                                .to(openGate2())
                                .withConstantHeading(270)
                                .build(config.maxPathPower),
                        Commands.waitMs(config.gatePushTimeoutMs)
                ),
                Commands.waitMs(config.secondsOpeningGate * 1000.0), // dwell so the gate opens

                // 7. Shoot set 2 — back off the gate to the staging point FIRST (constant
                //    heading, pure translation off the wall), THEN rotate and return to
                //    launch. Without the back-off the robot turns into the gate.
                new FollowPathBuilder(robot, alliance)
                        .from(openGate2())
                        .to(gate2Stage())
                        .withConstantHeading(270)
                        .build(config.maxPathPower),
                new FollowPathBuilder(robot, alliance)
                        .from(gate2Stage())
                        .to(launchClose3())
                        .withLinearHeadingCompletion(config.endTimeForLinearHeadingInterpolation)
                        .build(config.maxPathPower),
                ModeAwareLaunchCommand.create(robot.launcher, robot.intake, false),

                // 8. Park
                new FollowPathBuilder(robot, alliance)
                        .from(launchClose3())
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

    private static Pose artifactsSet1Control0() {
        return new Pose(waypoints.artifactsSet1Control0X, waypoints.artifactsSet1Control0Y, 0);
    }

    private static Pose openGate() {
        return new Pose(waypoints.openGateX, waypoints.openGateY, Math.toRadians(waypoints.openGateHeading));
    }

    private static Pose openGateControl() {
        return new Pose(waypoints.openGateControlX, waypoints.openGateControlY, 0);
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

    private static Pose openGate2() {
        return new Pose(waypoints.openGate2X, waypoints.openGate2Y, Math.toRadians(waypoints.openGate2Heading));
    }

    private static Pose openGate2Control() {
        return new Pose(waypoints.openGate2ControlX, waypoints.openGate2ControlY, 0);
    }

    private static Pose gate2Stage() {
        return new Pose(waypoints.gate2StageX, waypoints.gate2StageY, Math.toRadians(waypoints.gate2StageHeading));
    }

    private static Pose launchClose3() {
        return new Pose(waypoints.launchClose3X, waypoints.launchClose3Y, Math.toRadians(waypoints.launchClose3Heading));
    }

    private static Pose nearGate() {
        return new Pose(waypoints.nearGateX, waypoints.nearGateY, Math.toRadians(waypoints.nearGateHeading));
    }

    private static Pose nearGateControl0() {
        return new Pose(waypoints.nearGateControl0X, waypoints.nearGateControl0Y, 0);
    }
}
