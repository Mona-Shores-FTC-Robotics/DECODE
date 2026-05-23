package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.PresetRangeSpinCommand;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.FollowPathBuilder;
import org.firstinspires.ftc.teamcode.util.IntakeMode;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

/**
 * Michiana Gate: 5-artifact close-side auto with gate cycling.
 *
 * All shots fire on the fly at T=fireAtT — robot never stops to shoot.
 *
 * Structure:
 *   Start → ShootZone           fire preloads on fly
 *   ShootZone → Art1 → OpenGate → ShootZone    fire Art1 set on fly
 *   ShootZone → Art2 → OpenGate → ShootZone    fire Art2 set on fly
 *   ShootZone → GateSlant → ShootZone          fire Art3 on fly
 *   ShootZone → GateSlant → ShootZone          fire Art4 on fly
 *   ShootZone → GateSlant → FinalShoot         fire Art5 on fly
 *   if ≥ minParkTime left → drive to NearGate, else stay at FinalShoot
 */
public class MichianaGateCommand {

    @Configurable
    public static class Config {
        public static double maxPathPower = 0.85;
        /** Linear heading interpolation completion weight for start → shoot zone path. */
        public static double headingInterpEnd = 0.7;
        /** Return path T at which shots fire. */
        public static double fireAtT = 0.85;
        /** Time to hold at openGate position so the gate opens (ms). */
        public static double openGateWaitMs = 400;
        /** Time to wait at gate slant position for artifacts to roll into intake (ms). */
        public static double gateSlantWaitMs = 1200;
        public static double autoDurationSeconds = 30.0;
        /** Minimum time remaining to bother driving to NearGate for parking. */
        public static double minParkTimeSeconds = 3.0;
    }

    @Configurable
    public static class Waypoints {
        public static double startX = 26.5, startY = 130, startHeading = 0;

        /** Robot fires from here for all shots except the final one. */
        public static double shootZoneX = 53, shootZoneY = 87, shootZoneHeading = 134;

        public static double art1X = 24, art1Y = 83.8;

        /** Sideways slide position that physically opens the gate. */
        public static double openGateX = 17, openGateY = 81, openGateHeading = 270;

        public static double art2X = 24, art2Y = 61;

        /** Slanted gate approach: opens gate and collects ~3 artifacts at once. */
        public static double gateSlantX = 12.5, gateSlantY = 60, gateSlantHeading = 150;

        /** Return endpoint for the final (Art5) gate cycle. */
        public static double finalShootX = 65, finalShootY = 100, finalShootHeading = 145;

        public static double nearGateX = 35, nearGateY = 70.4, nearGateHeading = 180;
    }

    private MichianaGateCommand() {}

    public static Pose getDefaultStartPose() {
        return new Pose(Waypoints.startX, Waypoints.startY, Math.toRadians(Waypoints.startHeading));
    }

    public static Command create(Robot robot, Alliance alliance) {
        return create(robot, alliance, null);
    }

    public static Command create(Robot robot, Alliance alliance, @Nullable Pose startOverride) {
        Pose goalCenter = alliance == Alliance.BLUE
                ? FieldConstants.BLUE_GOAL_CENTER
                : FieldConstants.RED_GOAL_CENTER;
        double goalX = goalCenter.getX();
        double goalY = goalCenter.getY();

        // ── Preloads: start → shoot zone ─────────────────────────────────────
        FollowPathBuilder preloadBuilder = new FollowPathBuilder(robot, alliance);
        if (startOverride != null) {
            preloadBuilder.fromWorldCoordinates(startOverride);
        } else {
            preloadBuilder.from(start());
        }
        Command preload = Groups.deadline(
                preloadBuilder
                        .to(shootZone())
                        .withLinearHeadingCompletion(Config.headingInterpEnd)
                        .withFireAtT(Config.fireAtT, robot.launcher, robot.intake)
                        .build(Config.maxPathPower),
                robot.intake.setIntakeModeCmd(IntakeMode.PASSIVE_REVERSE),
                PresetRangeSpinCommand.create(
                        robot.launcher, LauncherRange.SHORT_AUTO, true,
                        robot.drive, robot.lighting, null)
        );

        return Groups.sequential(
                ConditionalFinalLaunchCommand.createTimerReset(),
                preload,

                // ── Art1: pick up then slide sideways into gate ───────────────
                Groups.deadline(
                        new FollowPathBuilder(robot, alliance)
                                .from(shootZone()).to(art1())
                                .withConstantHeading(270)
                                .build(Config.maxPathPower),
                        robot.intake.autoSmartIntakeCmd()
                ),
                new FollowPathBuilder(robot, alliance)
                        .from(art1()).to(openGate())
                        .withConstantHeading(270)
                        .build(Config.maxPathPower),
                Commands.waitMs(Config.openGateWaitMs),
                new FollowPathBuilder(robot, alliance)
                        .from(openGate()).to(shootZone())
                        .withFacingPoint(goalX, goalY)
                        .withFireAtT(Config.fireAtT, robot.launcher, robot.intake)
                        .build(Config.maxPathPower),

                // ── Art2: pick up then slide sideways into gate ───────────────
                Groups.deadline(
                        new FollowPathBuilder(robot, alliance)
                                .from(shootZone()).to(art2())
                                .withConstantHeading(270)
                                .build(Config.maxPathPower),
                        robot.intake.autoSmartIntakeCmd()
                ),
                new FollowPathBuilder(robot, alliance)
                        .from(art2()).to(openGate())
                        .withConstantHeading(270)
                        .build(Config.maxPathPower),
                Commands.waitMs(Config.openGateWaitMs),
                new FollowPathBuilder(robot, alliance)
                        .from(openGate()).to(shootZone())
                        .withFacingPoint(goalX, goalY)
                        .withFireAtT(Config.fireAtT, robot.launcher, robot.intake)
                        .build(Config.maxPathPower),

                // ── Art3: slant into gate, collect, return to shoot zone ───────
                gateSlantCycle(robot, alliance, goalX, goalY, shootZone()),

                // ── Art4: same pattern ────────────────────────────────────────
                gateSlantCycle(robot, alliance, goalX, goalY, shootZone()),

                // ── Art5: slant into gate, collect, return to final shoot spot ─
                gateSlantCycle(robot, alliance, goalX, goalY, finalShoot()),

                // ── Conditional park ──────────────────────────────────────────
                ConditionalFinalLaunchCommand.create(
                        Config.autoDurationSeconds,
                        Config.minParkTimeSeconds,
                        new FollowPathBuilder(robot, alliance)
                                .from(finalShoot()).to(nearGate())
                                .withLinearHeadingCompletion(Config.headingInterpEnd)
                                .build(Config.maxPathPower),
                        Commands.instant(() -> {})
                )
        );
    }

    private static Command gateSlantCycle(Robot robot, Alliance alliance,
                                           double goalX, double goalY, Pose returnTarget) {
        return Groups.sequential(
                Groups.deadline(
                        new FollowPathBuilder(robot, alliance)
                                .from(shootZone()).to(gateSlant())
                                .withConstantHeading(Waypoints.gateSlantHeading)
                                .build(Config.maxPathPower),
                        robot.intake.autoSmartIntakeCmd()
                ),
                Commands.waitMs(Config.gateSlantWaitMs),
                new FollowPathBuilder(robot, alliance)
                        .from(gateSlant()).to(returnTarget)
                        .withFacingPoint(goalX, goalY)
                        .withFireAtT(Config.fireAtT, robot.launcher, robot.intake)
                        .build(Config.maxPathPower)
        );
    }

    // ── Pose helpers ──────────────────────────────────────────────────────────

    private static Pose start() {
        return new Pose(Waypoints.startX, Waypoints.startY, Math.toRadians(Waypoints.startHeading));
    }

    private static Pose shootZone() {
        return new Pose(Waypoints.shootZoneX, Waypoints.shootZoneY, Math.toRadians(Waypoints.shootZoneHeading));
    }

    private static Pose art1() {
        return new Pose(Waypoints.art1X, Waypoints.art1Y, Math.toRadians(270));
    }

    private static Pose openGate() {
        return new Pose(Waypoints.openGateX, Waypoints.openGateY, Math.toRadians(Waypoints.openGateHeading));
    }

    private static Pose art2() {
        return new Pose(Waypoints.art2X, Waypoints.art2Y, Math.toRadians(270));
    }

    private static Pose gateSlant() {
        return new Pose(Waypoints.gateSlantX, Waypoints.gateSlantY, Math.toRadians(Waypoints.gateSlantHeading));
    }

    private static Pose finalShoot() {
        return new Pose(Waypoints.finalShootX, Waypoints.finalShootY, Math.toRadians(Waypoints.finalShootHeading));
    }

    private static Pose nearGate() {
        return new Pose(Waypoints.nearGateX, Waypoints.nearGateY, Math.toRadians(Waypoints.nearGateHeading));
    }
}
