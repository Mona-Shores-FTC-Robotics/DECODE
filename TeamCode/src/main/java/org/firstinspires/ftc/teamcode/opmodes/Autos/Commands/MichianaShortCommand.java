package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.PresetRangeSpinCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.LauncherRange;
import org.firstinspires.ftc.teamcode.util.PpPathLoader;
import org.firstinspires.ftc.teamcode.util.PpPathLoader.ParsedLine;
import org.firstinspires.ftc.teamcode.util.PpPathLoader.ParsedPp;

/**
 * MichianaShort: entire auto as a single PathChain with ParametricCallbacks.
 *
 * Structure:
 *   Seg 0 : start    → launch  (spin up + fire preloads at T=fireAtT)
 *   Seg 1 : launch   → art1    (face 270°, brakes near artifacts, starts intake)
 *   Seg 2 : art1     → launch  (face goal dynamically, fire at T=fireAtT)
 *   Seg 3 : launch   → art2    (same as seg 1)
 *   Seg 4 : art2     → launch  (same as seg 2, curved return)
 *   Seg 5 : launch   → art3    (same as seg 1)
 *   Seg 6 : art3     → launch  (same as seg 2, curved return)
 *   Seg 7 : launch   → park
 *
 * Return paths use HeadingInterpolator.facingPoint() so the robot continuously
 * faces the fixed goal position as it moves — the aim angle changes along the path
 * because the goal is a fixed point on the field, not a fixed heading.
 *
 * Two tuning modes (adjust on FTC Dashboard, no recompile):
 *   Fast       : fireAtT=0.6,  returnBrakingStart=0.9  — fire while moving fast
 *   Consistent : fireAtT=0.85, returnBrakingStart=0.5  — fire while decelerating
 */
public class MichianaShortCommand {

    @Configurable
    public static class Config {
        /** Max power for the full auto chain */
        public static double maxPathPower = 0.85;

        /**
         * Linear heading interpolation completion weight for the first path (start → launch).
         * Heading reaches launch heading at this T fraction, then holds for the rest of the segment.
         */
        public static double headingInterpEnd = 0.7;

        /**
         * Outbound braking start (launch → artifacts).
         * Robot begins decelerating at this T fraction so it arrives slowly for intake.
         */
        public static double outboundBrakingStart = 0.7;

        /**
         * Return path T value at which shots fire.
         * Pair with returnBrakingStart:
         *   Fast mode       : fireAtT=0.60, returnBrakingStart=0.90
         *   Consistent mode : fireAtT=0.85, returnBrakingStart=0.50
         */
        public static double fireAtT = 0.85;

        /**
         * Return path braking start (artifacts → launch).
         * High = robot still moving fast when shots fire (faster cycle).
         * Low  = robot decelerating when shots fire (more consistent RPM).
         */
        public static double returnBrakingStart = 0.5;
    }

    @Configurable
    public static class Waypoints {
        public static double startX = 26.5, startY = 130, startHeading = 0;

        public static double launchX = 36, launchY = 107, launchHeading = 134;

        // Art1: outbound control only (return is a straight line)
        public static double art1X = 24, art1Y = 83.8;
        public static double art1CtrlX = 24, art1CtrlY = 113;

        // Art2: outbound + return control
        public static double art2X = 24, art2Y = 61;
        public static double art2CtrlX = 24, art2CtrlY = 100;
        public static double art2RetCtrlX = 50.5, art2RetCtrlY = 72;

        // Art3: outbound + return control
        public static double art3X = 24, art3Y = 35.5;
        public static double art3CtrlX = 24, art3CtrlY = 100;
        public static double art3RetCtrlX = 44.5, art3RetCtrlY = 73.5;

        public static double nearGateX = 35, nearGateY = 70.4, nearGateHeading = 180;
        public static double nearGateCtrlX = 41, nearGateCtrlY = 85;
    }

    private MichianaShortCommand() {}

    public static Pose getDefaultStartPose() {
        return wp(Waypoints.startX, Waypoints.startY, Waypoints.startHeading, Alliance.BLUE);
    }

    public static Command create(Robot robot, Alliance alliance) {
        return create(robot, alliance, null);
    }

    public static Command create(Robot robot, Alliance alliance, Pose startOverride) {
        LauncherSubsystem launcher = robot.launcher;
        IntakeSubsystem intake = robot.intake;
        Follower follower = robot.drive.getFollower();

        Runnable fireAll = () -> {
            launcher.spinUpAllLanesToLaunch();
            for (LauncherLane lane : LauncherLane.values()) launcher.queueShot(lane);
            intake.setGateAllowArtifacts();
        };

        Pose start = startOverride != null
                ? startOverride
                : wp(Waypoints.startX, Waypoints.startY, Waypoints.startHeading, alliance);
        Pose launch = wp(Waypoints.launchX, Waypoints.launchY, Waypoints.launchHeading, alliance);
        Pose nearGate = wp(Waypoints.nearGateX, Waypoints.nearGateY, Waypoints.nearGateHeading, alliance);

        PathBuilder b = follower.pathBuilder();

        // ── Seg 0: start → launch ────────────────────────────────────────────
        // Spin up at path start, fire preloads at T=fireAtT
        b.addPath(new BezierLine(start, launch))
         .setLinearHeadingInterpolation(start.getHeading(), launch.getHeading(),
                 Config.headingInterpEnd)
         .addParametricCallback(0.0, launcher::spinUpAllLanesToLaunch)
         .addParametricCallback(Config.fireAtT, fireAll);

        // ── Cycles 1–3 ───────────────────────────────────────────────────────
        addCycle(b, launch,
                wp(Waypoints.art1X, Waypoints.art1Y, 0, alliance),
                ctrl(Waypoints.art1CtrlX, Waypoints.art1CtrlY, alliance),
                null,   // straight return, no control point
                alliance, fireAll, intake);

        addCycle(b, launch,
                wp(Waypoints.art2X, Waypoints.art2Y, 0, alliance),
                ctrl(Waypoints.art2CtrlX, Waypoints.art2CtrlY, alliance),
                ctrl(Waypoints.art2RetCtrlX, Waypoints.art2RetCtrlY, alliance),
                alliance, fireAll, intake);

        addCycle(b, launch,
                wp(Waypoints.art3X, Waypoints.art3Y, 0, alliance),
                ctrl(Waypoints.art3CtrlX, Waypoints.art3CtrlY, alliance),
                ctrl(Waypoints.art3RetCtrlX, Waypoints.art3RetCtrlY, alliance),
                alliance, fireAll, intake);

        // ── Final: launch → park ─────────────────────────────────────────────
        b.addPath(new BezierCurve(launch,
                ctrl(Waypoints.nearGateCtrlX, Waypoints.nearGateCtrlY, alliance), nearGate))
         .setLinearHeadingInterpolation(launch.getHeading(), nearGate.getHeading(), 0.8);

        PathChain fullAuto = b.build();

        // PresetRangeSpinCommand(finishWhenReady=false) runs alongside the full chain,
        // ensuring SHORT_AUTO RPM targets are always set even if something clears them.
        return Groups.deadline(
                PedroCommands.follow(follower, fullAuto, true, Config.maxPathPower),
                PresetRangeSpinCommand.create(
                        launcher, LauncherRange.SHORT_AUTO, false,
                        robot.drive, robot.lighting, null)
        );
    }

    /**
     * Builds the auto from a parsed .pp file ({@link PpPathLoader.ParsedPp}). Callbacks
     * are attached by segment-name convention (case-insensitive substring match):
     * <ul>
     *   <li>{@code Launch*}  → fire all lanes at T={@code Config.fireAtT},
     *                          braking at {@code Config.returnBrakingStart}</li>
     *   <li>{@code Artifact*} → intake roller on at T=0.05,
     *                          braking at {@code Config.outboundBrakingStart}</li>
     *   <li>anything else (e.g. "Open Gate", "Park") → no callbacks, geometry only</li>
     * </ul>
     * The first segment always also gets {@code spinUpAllLanesToLaunch} at T=0.
     *
     * <p>All waypoints + control points are mirrored for the active alliance via
     * {@link AutoField#poseForAlliance}. Heading interpolation uses each segment's
     * {@code startDeg}/{@code endDeg} (linear) or {@code degrees} (constant).
     */
    public static Command createFromPp(Robot robot, Alliance alliance, ParsedPp pp,
                                       Pose startOverride) {
        if (pp == null || pp.lines.isEmpty()) {
            throw new IllegalArgumentException("ParsedPp has no segments");
        }
        LauncherSubsystem launcher = robot.launcher;
        IntakeSubsystem intake = robot.intake;
        Follower follower = robot.drive.getFollower();

        Runnable fireAll = () -> {
            launcher.spinUpAllLanesToLaunch();
            for (LauncherLane lane : LauncherLane.values()) launcher.queueShot(lane);
            intake.setGateAllowArtifacts();
        };

        Pose chainStart = startOverride != null
                ? startOverride
                : AutoField.poseForAlliance(pp.startX, pp.startY, pp.startHeadingDeg, alliance);

        PathBuilder b = follower.pathBuilder();
        Pose prev = chainStart;

        for (int i = 0; i < pp.lines.size(); i++) {
            ParsedLine seg = pp.lines.get(i);
            boolean isFirst = (i == 0);

            // Endpoint pose, mirrored. The heading we store on the Pose is what
            // "linear" interpolation will reach at T=1.0 (segment endDeg).
            Pose end = AutoField.poseForAlliance(seg.endX, seg.endY, seg.endDeg, alliance);

            // Geometry: 0 control points → BezierLine; 1+ → BezierCurve.
            if (seg.controlPoints.isEmpty()) {
                b.addPath(new BezierLine(prev, end));
            } else if (seg.controlPoints.size() == 1) {
                double[] c = seg.controlPoints.get(0);
                Pose ctrl = AutoField.poseForAlliance(c[0], c[1], 0, alliance);
                b.addPath(new BezierCurve(prev, ctrl, end));
            } else {
                // Higher-order curves: Pedro BezierCurve takes (start, ctrls..., end).
                // Build the Pose array dynamically.
                Pose[] points = new Pose[seg.controlPoints.size() + 2];
                points[0] = prev;
                for (int j = 0; j < seg.controlPoints.size(); j++) {
                    double[] c = seg.controlPoints.get(j);
                    points[j + 1] = AutoField.poseForAlliance(c[0], c[1], 0, alliance);
                }
                points[points.length - 1] = end;
                b.addPath(new BezierCurve(points));
            }

            // Heading interpolation.
            if (seg.headingMode == PpPathLoader.HeadingMode.CONSTANT) {
                // For constant heading we still pass the .pp's degrees value through alliance mirror.
                double mirroredConstantRad = AutoField.poseForAlliance(0, 0, seg.constantDeg, alliance).getHeading();
                b.setConstantHeadingInterpolation(mirroredConstantRad);
            } else {
                b.setLinearHeadingInterpolation(prev.getHeading(), end.getHeading(),
                        Config.headingInterpEnd);
            }

            // Reverse: robot traverses the segment backward (e.g. driving rear-first
            // through a tight gate). Pedro PathBuilder.setReversed() applies to the
            // most recently added path.
            if (seg.reverse) {
                b.setReversed();
            }

            // Name-based callback policy.
            String n = seg.name == null ? "" : seg.name.toLowerCase();
            if (isFirst) {
                b.addParametricCallback(0.0, launcher::spinUpAllLanesToLaunch);
            }
            if (n.contains("launch")) {
                b.setBrakingStart(Config.returnBrakingStart);
                b.addParametricCallback(Config.fireAtT, fireAll);
            } else if (n.contains("artifact")) {
                b.setBrakingStart(Config.outboundBrakingStart);
                b.addParametricCallback(0.05, intake::forwardRoller);
            }
            // Everything else (Open Gate, Park, etc.) → no callbacks.

            prev = end;
        }

        PathChain fullAuto = b.build();

        return Groups.deadline(
                PedroCommands.follow(follower, fullAuto, true, Config.maxPathPower),
                PresetRangeSpinCommand.create(
                        launcher, LauncherRange.SHORT_AUTO, false,
                        robot.drive, robot.lighting, null)
        );
    }

    /**
     * Appends one outbound + return cycle to the PathBuilder.
     *
     * Outbound (launch → artifacts):
     *   - Constant 270° heading (robot faces the artifact area)
     *   - Brakes starting at outboundBrakingStart so it arrives slowly
     *   - Starts intake at T=0.05
     *
     * Return (artifacts → launch):
     *   - HeadingInterpolator.facingPoint() continuously aims at the fixed goal
     *     position. The required heading changes along the path as the robot moves,
     *     so a constant heading would drift off-target at different field positions.
     *   - Brakes starting at returnBrakingStart (tune for fast vs consistent)
     *   - Fires at T=fireAtT
     *
     * @param retCtrl Control point for the return curve, or null for a straight line.
     */
    private static void addCycle(PathBuilder b,
                                  Pose launch, Pose artifacts,
                                  Pose outCtrl, @Nullable Pose retCtrl,
                                  Alliance alliance,
                                  Runnable fireAll, IntakeSubsystem intake) {
        // Outbound
        b.addPath(new BezierCurve(launch, outCtrl, artifacts))
         .setConstantHeadingInterpolation(Math.toRadians(270))
         .setBrakingStart(Config.outboundBrakingStart)
         .addParametricCallback(0.05, intake::forwardRoller);

        // Return: dynamically face the fixed goal as the robot's field position changes
        Pose goalCenter = alliance == Alliance.BLUE
                ? FieldConstants.BLUE_GOAL_CENTER
                : FieldConstants.RED_GOAL_CENTER;

        if (retCtrl != null) {
            b.addPath(new BezierCurve(artifacts, retCtrl, launch));
        } else {
            b.addPath(new BezierLine(artifacts, launch));
        }
        b.setHeadingInterpolation(HeadingInterpolator.facingPoint(goalCenter.getX(), goalCenter.getY()))
         .setBrakingStart(Config.returnBrakingStart)
         .addParametricCallback(Config.fireAtT, fireAll);
    }

    // ── Pose helpers ─────────────────────────────────────────────────────────

    private static Pose wp(double x, double y, double headingDeg, Alliance alliance) {
        return AutoField.poseForAlliance(x, y, headingDeg, alliance);
    }

    private static Pose ctrl(double x, double y, Alliance alliance) {
        return AutoField.poseForAlliance(x, y, 0, alliance);
    }
}
