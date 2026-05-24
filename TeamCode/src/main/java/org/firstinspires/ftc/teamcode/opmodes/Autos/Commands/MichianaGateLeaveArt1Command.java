package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.PresetRangeSpinCommand;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FollowPathBuilder;
import org.firstinspires.ftc.teamcode.util.IntakeMode;
import org.firstinspires.ftc.teamcode.util.LauncherRange;
import org.firstinspires.ftc.teamcode.util.RobotState;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

/**
 * Michiana Gate (Art1 Last): leaves Art1 for the final pickup so it fills
 * the chute at the end of auto. After preloads, goes straight to the gate
 * area to open it, picks up Art2 from the released pile, runs 3 gate slant
 * cycles (Art3-5), then picks up Art1 on the way back and fires a final shot.
 *
 * Structure:
 *   Start → PreloadZone (curved)          fire preloads on fly
 *   PreloadZone → StageGate (curved)      intake running — sweeps toward gate
 *   StageGate → OpenGate (straight)       opens gate, no intake
 *   OpenGate → Art2 (curved)              intake running
 *   Art2 → ShootZone (curved)             fire Art2 set on fly
 *   ShootZone → GateSlant → wait → ShootZone    fire Art3 on fly  (×2)
 *   ShootZone → GateSlant → wait → ShootZone    fire Art4 on fly
 *   ShootZone → GateSlant → wait → ShootZone    fire Art5 on fly
 *   ShootZone → Art1Stage → Art1          intake running
 *   Art1 → FinalShot (curved)             fire Art1 set on fly near start line
 */
public class MichianaGateLeaveArt1Command {

    @Configurable
    public static class Config {
        public static double maxPathPower = 0.8;

        /**
         * Fraction of the preload path over which heading linearly interpolates to its
         * target. Heading reaches the target at this T and then holds.
         * Must be meaningfully less than fireAtT to allow settling time.
         */
        public static double headingInterpEnd = 0.7;

        /**
         * Path T at which the launcher fires on outbound/return legs.
         *
         * Two constraints must both hold at this T:
         *   1. For withLinearHeadingCompletion paths: T must be > headingInterpEnd so
         *      the heading controller has had time to reach and settle on its target.
         *      (Current gap: 0.85 - 0.70 = 0.15 of path travel — watch this if either
         *      value is tuned.)
         *   2. For withFacingPoint paths: the heading is tracking continuously, but
         *      faster paths mean more heading lag at any given T. If shots are
         *      consistently off-angle, try increasing this value or reducing maxPathPower
         *      on return legs to give the heading controller more time to settle.
         */
        public static double fireAtT = 0.90;

        /** Wait at gate slant for artifacts to roll in (ms). */
        public static double gateSlantWaitMs = 750;
    }

    @Configurable
    public static class Waypoints {
        public static double startX = 29.67, startY = 127.56, startHeading = 0;

        /** First stop — fires preloads here on the way in. */
        public static double preloadZoneX = 37.5, preloadZoneY = 106, preloadZoneHeading = 134;
        public static double preloadZoneCtrlX = 32, preloadZoneCtrlY = 113;

        /** Staging position just right of the gate before opening it. */
        public static double stageGateX = 39, stageGateY = 74.75, stageGateHeading = 270;
        public static double stageGateCtrlX = 56.5, stageGateCtrlY = 84;

        /** Gate opening position — robot slides left to trigger the gate. */
        public static double openGateX = 15.2, openGateY = 74.75, openGateHeading = 270;

        /** Art2 pickup — collected immediately after gate opens. */
        public static double art2X = 21, art2Y = 59, art2Heading = 270;
        public static double art2CtrlX = 25, art2CtrlY = 72;

        /** Main shooting zone for Art2 and all gate cycle returns. */
        public static double shootZoneX = 51, shootZoneY = 85, shootZoneHeading = 130;
        /** Control point curving the return from Art2 toward the shoot zone. */
        public static double shoot2CtrlX = 21.6, shoot2CtrlY = 44.8;

        /** Slanted gate approach — opens gate and collects 3 artifacts. */
public static double gateSlantX = 13.5, gateSlantY = 58.5, gateSlantHeading = 152;
        /** Shared control for all gate slant outbound and return curves. */
        public static double gateSlantCtrlX = 42, gateSlantCtrlY = 63;

        /** Art1 staging position (on the way to Art1 from shoot zone). */
        public static double art1StageX = 24, art1StageY = 71, art1StageHeading = 90;

        /** Art1 pickup — near the top of the artifact zone, saved for last. */
        public static double art1X = 23.73, art1Y = 89.68, art1Heading = 90;

        /** Final shot position — near starting line, fires Art1 set. */
        public static double finalShotX = 40, finalShotY = 123, finalShotHeading = 154;
        public static double finalShotCtrlX = 23.28, finalShotCtrlY = 109.57;
    }

    private MichianaGateLeaveArt1Command() {}

    public static Pose getDefaultStartPose() {
        return new Pose(Waypoints.startX, Waypoints.startY, Math.toRadians(Waypoints.startHeading));
    }

    public static Command create(Robot robot, Alliance alliance) {
        return create(robot, alliance, null);
    }

    public static Command create(Robot robot, Alliance alliance, @Nullable Pose startOverride) {
        // ── Preloads: start → preload zone via curve, fire on fly ─────────────
        FollowPathBuilder preloadBuilder = new FollowPathBuilder(robot, alliance);
        if (startOverride != null) {
            preloadBuilder.fromWorldCoordinates(startOverride);
        } else {
            preloadBuilder.from(start());
        }
        Command preload = Groups.deadline(
                preloadBuilder
                        .to(preloadZone())
                        .withControl(preloadZoneCtrl())
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
                Commands.instant(() -> RobotState.packet.put("Auto/phase", "preload")),
                preload,

                // ── Sweep toward gate: switch to MID_AUTO while driving ───────────
                Commands.instant(() -> RobotState.packet.put("Auto/phase", "stageGate")),
                Groups.deadline(
                        new FollowPathBuilder(robot, alliance)
                                .from(preloadZone()).to(stageGate())
                                .withControl(stageGateCtrl())
                                .withLinearHeadingCompletion(1.0)
                                .build(Config.maxPathPower),
                        robot.intake.autoSmartIntakeCmd(),
                        PresetRangeSpinCommand.create(
                                robot.launcher, LauncherRange.MID_AUTO, false,
                                null, null, null)
                ),

                // ── Open gate: straight left slide, no intake ─────────────────────
                Commands.instant(() -> RobotState.packet.put("Auto/phase", "openGate")),
                new FollowPathBuilder(robot, alliance)
                        .from(stageGate()).to(openGate())
                        .withConstantHeading(270)
                        .build(Config.maxPathPower),

                // ── Art2: collect field artifact ──────────────────────────────────
                Commands.instant(() -> RobotState.packet.put("Auto/phase", "art2Pickup")),
                Groups.deadline(
                        new FollowPathBuilder(robot, alliance)
                                .from(openGate()).to(art2())
                                .withControl(art2Ctrl())
                                .withConstantHeading(270)
                                .build(Config.maxPathPower),
                        robot.intake.autoSmartIntakeCmd()
                ),

                // ── Return and fire Art2 set on fly ───────────────────────────────
                Commands.instant(() -> RobotState.packet.put("Auto/phase", "shoot2")),
                new FollowPathBuilder(robot, alliance)
                        .from(art2()).to(shootZone())
                        .withControl(shoot2Ctrl())
                        .withLinearHeadingCompletion(Config.headingInterpEnd)
                        .withFireAtT(Config.fireAtT, robot.launcher, robot.intake)
                        .build(Config.maxPathPower),

                // ── Art3, Art4, Art5: gate slant cycles ───────────────────────────
                Commands.instant(() -> RobotState.packet.put("Auto/phase", "gateSlant1")),
                gateSlantCycle(robot, alliance),
                Commands.instant(() -> RobotState.packet.put("Auto/phase", "gateSlant2")),
                gateSlantCycle(robot, alliance),
                Commands.instant(() -> RobotState.packet.put("Auto/phase", "gateSlant3")),
                gateSlantCycle(robot, alliance),

                // ── Art1 pickup: switch back to SHORT_AUTO while driving ──────────
                Commands.instant(() -> RobotState.packet.put("Auto/phase", "art1Pickup")),
                Groups.deadline(
                        Groups.sequential(
                                new FollowPathBuilder(robot, alliance)
                                        .from(shootZone()).to(art1Stage())
                                        .withControl(gateSlantCtrl())
                                        .withLinearHeadingCompletion(1.0)
                                        .build(Config.maxPathPower),
                                new FollowPathBuilder(robot, alliance)
                                        .from(art1Stage()).to(art1())
                                        .withConstantHeading(Waypoints.art1Heading)
                                        .build(Config.maxPathPower)
                        ),
                        robot.intake.autoSmartIntakeCmd(),
                        PresetRangeSpinCommand.create(
                                robot.launcher, LauncherRange.SHORT_AUTO, false,
                                null, null, null)
                ),

                // ── Final shot: curve back toward start line, fire Art1 set on fly ─
                Commands.instant(() -> RobotState.packet.put("Auto/phase", "finalShot")),
                new FollowPathBuilder(robot, alliance)
                        .from(art1()).to(finalShot())
                        .withControl(finalShotCtrl())
                        .withLinearHeadingCompletion(Config.headingInterpEnd)
                        .withFireAtT(Config.fireAtT, robot.launcher, robot.intake)
                        .build(Config.maxPathPower),

                Commands.instant(() -> RobotState.packet.put("Auto/phase", "done"))
        );
    }

    private static Command gateSlantCycle(Robot robot, Alliance alliance) {
        return Groups.sequential(
                Groups.deadline(
                        new FollowPathBuilder(robot, alliance)
                                .from(shootZone()).to(gateSlant())
                                .withControl(gateSlantCtrl())
                                .withLinearHeadingCompletion(1.0)
                                .build(Config.maxPathPower),
                        robot.intake.autoSmartIntakeCmd()
                ),
                Commands.instant(() -> RobotState.packet.put("Auto/phase/gateSlant", "waiting")),
                Commands.waitMs(Config.gateSlantWaitMs),
                Commands.instant(() -> RobotState.packet.put("Auto/phase/gateSlant", "returning")),
                new FollowPathBuilder(robot, alliance)
                        .from(gateSlant()).to(shootZone())
                        .withControl(gateSlantCtrl())
                        .withLinearHeadingCompletion(Config.headingInterpEnd)
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

    private static Pose preloadZoneCtrl() {
        return new Pose(Waypoints.preloadZoneCtrlX, Waypoints.preloadZoneCtrlY, 0);
    }

    private static Pose stageGate() {
        return new Pose(Waypoints.stageGateX, Waypoints.stageGateY, Math.toRadians(Waypoints.stageGateHeading));
    }

    private static Pose stageGateCtrl() {
        return new Pose(Waypoints.stageGateCtrlX, Waypoints.stageGateCtrlY, 0);
    }

    private static Pose openGate() {
        return new Pose(Waypoints.openGateX, Waypoints.openGateY, Math.toRadians(Waypoints.openGateHeading));
    }

    private static Pose art2() {
        return new Pose(Waypoints.art2X, Waypoints.art2Y, Math.toRadians(Waypoints.art2Heading));
    }

    private static Pose art2Ctrl() {
        return new Pose(Waypoints.art2CtrlX, Waypoints.art2CtrlY, 0);
    }

    private static Pose shootZone() {
        return new Pose(Waypoints.shootZoneX, Waypoints.shootZoneY, Math.toRadians(Waypoints.shootZoneHeading));
    }

    private static Pose shoot2Ctrl() {
        return new Pose(Waypoints.shoot2CtrlX, Waypoints.shoot2CtrlY, 0);
    }

    private static Pose gateSlant() {
        return new Pose(Waypoints.gateSlantX, Waypoints.gateSlantY, Math.toRadians(Waypoints.gateSlantHeading));
    }

    private static Pose gateSlantCtrl() {
        return new Pose(Waypoints.gateSlantCtrlX, Waypoints.gateSlantCtrlY, 0);
    }

    private static Pose art1Stage() {
        return new Pose(Waypoints.art1StageX, Waypoints.art1StageY, Math.toRadians(Waypoints.art1StageHeading));
    }

    private static Pose art1() {
        return new Pose(Waypoints.art1X, Waypoints.art1Y, Math.toRadians(Waypoints.art1Heading));
    }

    private static Pose finalShot() {
        return new Pose(Waypoints.finalShotX, Waypoints.finalShotY, Math.toRadians(Waypoints.finalShotHeading));
    }

    private static Pose finalShotCtrl() {
        return new Pose(Waypoints.finalShotCtrlX, Waypoints.finalShotCtrlY, 0);
    }
}
