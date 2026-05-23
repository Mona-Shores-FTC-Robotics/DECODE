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
 * Michiana Gate: 5-artifact close-side auto with on-the-fly shooting and gate cycling.
 *
 * Structure:
 *   Start → PreloadZone                           fire preloads on fly
 *   PreloadZone → Art1 → OpenGate → ShootZone    fire Art1 set on fly (no gate wait)
 *   ShootZone → Art2Stage → Art2 → OpenGate → ShootZone    fire Art2 set on fly
 *   ShootZone → GateSlant → wait 750ms → ShootZone    fire Art3 on fly
 *   ShootZone → GateSlant → wait 750ms → ShootZone    fire Art4 on fly
 *   ShootZone → GateSlant → wait 750ms → FinalShoot   fire Art5 on fly
 *   if ≥ minParkTime left → NearGate, else stay at FinalShoot
 */
public class MichianaGateCommand {

    @Configurable
    public static class Config {
        public static double maxPathPower = 0.85;
        /** Linear heading completion weight for the start → preload zone path. */
        public static double headingInterpEnd = 0.7;
        /** Path T at which shots fire on return legs. */
        public static double fireAtT = 0.85;
        /** Wait at gate slant position for artifacts to roll in (ms). */
        public static double gateSlantWaitMs = 750;
        public static double autoDurationSeconds = 30.0;
        /** Minimum remaining time to drive to NearGate for parking. */
        public static double minParkTimeSeconds = 3.0;
    }

    @Configurable
    public static class Waypoints {
        public static double startX = 29.67, startY = 127.56, startHeading = 0;

        /** Close first-stop for firing preloads. */
        public static double preloadZoneX = 35.57, preloadZoneY = 109.96, preloadZoneHeading = 135.5;

        /** Art1 pickup — robot sweeps through here on the way to the gate. */
        public static double art1X = 23.63, art1Y = 95.99;

        /** Gate opening position (used for both Art1 and Art2 gate slides). */
        public static double openGateX = 15.7, openGateY = 73.8, openGateHeading = 270;
        /** Control point for Art1 → OpenGate curve. */
        public static double openGate1CtrlX = 25.42, openGate1CtrlY = 72.87;
        /** Control point for Art2 → OpenGate curve (different approach angle). */
        public static double openGate2CtrlX = 22.70, openGate2CtrlY = 72.96;

        /** Intermediate waypoint on the way to Art2 (curved approach from shoot zone). */
        public static double art2StageX = 23.76, art2StageY = 70.99;
        public static double art2StageCtrlX = 25.92, art2StageCtrlY = 84.12;

        public static double art2X = 23.49, art2Y = 60.09;

        /** Main shooting zone — all shots except final Art5 return here. */
        public static double shootZoneX = 53, shootZoneY = 87, shootZoneHeading = 134;

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

        // ── Preloads: start → preload zone (close first stop, fire on fly) ────
        FollowPathBuilder preloadBuilder = new FollowPathBuilder(robot, alliance);
        if (startOverride != null) {
            preloadBuilder.fromWorldCoordinates(startOverride);
        } else {
            preloadBuilder.from(start());
        }
        Command preload = Groups.deadline(
                preloadBuilder
                        .to(preloadZone())
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

                // ── Art1: sweep through Art1 → slide into gate → return to shoot zone ──
                // Heading linearly interpolates from preloadZone heading to 270° over the Art1 path
                Groups.deadline(
                        new FollowPathBuilder(robot, alliance)
                                .from(preloadZone()).to(art1())
                                .withLinearHeadingCompletion(1.0)
                                .build(Config.maxPathPower),
                        robot.intake.autoSmartIntakeCmd()
                ),
                new FollowPathBuilder(robot, alliance)
                        .from(art1()).to(openGate())
                        .withControl(openGate1Ctrl())
                        .withConstantHeading(270)
                        .build(Config.maxPathPower),
                new FollowPathBuilder(robot, alliance)
                        .from(openGate()).to(shootZone())
                        .withFacingPoint(goalX, goalY)
                        .withFireAtT(Config.fireAtT, robot.launcher, robot.intake)
                        .build(Config.maxPathPower),

                // ── Art2: curved approach via staging point → continue to Art2 → gate → shoot ──
                Groups.deadline(
                        Groups.sequential(
                                new FollowPathBuilder(robot, alliance)
                                        .from(shootZone()).to(art2Stage())
                                        .withControl(art2StageCtrl())
                                        .withLinearHeadingCompletion(1.0)
                                        .build(Config.maxPathPower),
                                new FollowPathBuilder(robot, alliance)
                                        .from(art2Stage()).to(art2())
                                        .withConstantHeading(270)
                                        .build(Config.maxPathPower)
                        ),
                        robot.intake.autoSmartIntakeCmd()
                ),
                new FollowPathBuilder(robot, alliance)
                        .from(art2()).to(openGate())
                        .withControl(openGate2Ctrl())
                        .withConstantHeading(270)
                        .build(Config.maxPathPower),
                new FollowPathBuilder(robot, alliance)
                        .from(openGate()).to(shootZone())
                        .withFacingPoint(goalX, goalY)
                        .withFireAtT(Config.fireAtT, robot.launcher, robot.intake)
                        .build(Config.maxPathPower),

                // ── Art3: slant gate cycle → shoot zone ───────────────────────
                gateSlantCycle(robot, alliance, goalX, goalY, shootZone()),

                // ── Art4: slant gate cycle → shoot zone ───────────────────────
                gateSlantCycle(robot, alliance, goalX, goalY, shootZone()),

                // ── Art5: slant gate cycle → final shoot spot ─────────────────
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
                                .withLinearHeadingCompletion(1.0)
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

    private static Pose preloadZone() {
        return new Pose(Waypoints.preloadZoneX, Waypoints.preloadZoneY, Math.toRadians(Waypoints.preloadZoneHeading));
    }

    private static Pose art1() {
        return new Pose(Waypoints.art1X, Waypoints.art1Y, Math.toRadians(270));
    }

    private static Pose openGate() {
        return new Pose(Waypoints.openGateX, Waypoints.openGateY, Math.toRadians(Waypoints.openGateHeading));
    }

    private static Pose openGate1Ctrl() {
        return new Pose(Waypoints.openGate1CtrlX, Waypoints.openGate1CtrlY, 0);
    }

    private static Pose openGate2Ctrl() {
        return new Pose(Waypoints.openGate2CtrlX, Waypoints.openGate2CtrlY, 0);
    }

    private static Pose art2Stage() {
        return new Pose(Waypoints.art2StageX, Waypoints.art2StageY, Math.toRadians(270));
    }

    private static Pose art2StageCtrl() {
        return new Pose(Waypoints.art2StageCtrlX, Waypoints.art2StageCtrlY, 0);
    }

    private static Pose art2() {
        return new Pose(Waypoints.art2X, Waypoints.art2Y, Math.toRadians(270));
    }

    private static Pose shootZone() {
        return new Pose(Waypoints.shootZoneX, Waypoints.shootZoneY, Math.toRadians(Waypoints.shootZoneHeading));
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
