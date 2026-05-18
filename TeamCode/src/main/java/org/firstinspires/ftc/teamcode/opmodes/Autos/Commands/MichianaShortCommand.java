package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.PresetRangeSpinCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.LauncherLane;
import org.firstinspires.ftc.teamcode.util.LauncherRange;
import org.firstinspires.ftc.teamcode.util.PpPathLoader;
import org.firstinspires.ftc.teamcode.util.PpPathLoader.ParsedLine;
import org.firstinspires.ftc.teamcode.util.PpPathLoader.ParsedPp;

import java.util.ArrayList;
import java.util.List;

/**
 * MichianaShort: a 3-cycle close-side auto built from a Pedro visualizer .pp file
 * via {@link PpPathLoader}. Mid-path triggers (spin-up, fire, intake-on) are
 * attached as Ivy {@code follow.with(waitUntil(t≥X).then(action))} composables,
 * not {@code PathBuilder.addParametricCallback} — matching the pattern used by
 * Baron Henderson's 22131-Decode (the Pedro/Ivy author's own production code).
 *
 * <p>Each .pp segment becomes its own single-path {@link PathChain} so
 * {@code follower.getCurrentTValue()} returns the T of THAT segment, and a
 * single {@code waitUntil} fires exactly once per segment.
 *
 * <p>Two tuning modes (adjust on FTC Dashboard, no recompile):
 * <ul>
 *   <li>Fast       : fireAtT=0.6,  returnBrakingStart=0.9  — fire while moving fast</li>
 *   <li>Consistent : fireAtT=0.85, returnBrakingStart=0.5  — fire while decelerating</li>
 * </ul>
 */
public class MichianaShortCommand {

    @Configurable
    public static class Config {
        /** Max power for every follow segment in the chain. */
        public static double maxPathPower = 0.85;

        /**
         * Linear heading interpolation completion weight for each linear segment.
         * Heading reaches the end value at this T fraction, then holds for the rest.
         */
        public static double headingInterpEnd = 0.7;

        /**
         * Outbound braking start (Artifact* segments).
         * Robot begins decelerating at this T fraction so it arrives slowly for intake.
         */
        public static double outboundBrakingStart = 0.7;

        /**
         * Return path T value at which shots fire (Launch* segments).
         * Pair with returnBrakingStart:
         *   Fast mode       : fireAtT=0.60, returnBrakingStart=0.90
         *   Consistent mode : fireAtT=0.85, returnBrakingStart=0.50
         */
        public static double fireAtT = 0.85;

        /**
         * Return path braking start (Launch* segments).
         * High = robot still moving fast when shots fire (faster cycle).
         * Low  = robot decelerating when shots fire (more consistent RPM).
         */
        public static double returnBrakingStart = 0.5;

        /** T at which the intake roller turns on during an outbound (Artifact*) segment. */
        public static double intakeOnAtT = 0.05;
    }

    private MichianaShortCommand() {}

    /**
     * Builds the auto from a parsed .pp file. Callback policy (case-insensitive
     * substring match on segment name):
     * <ul>
     *   <li>{@code Launch*}  → fire all lanes at T={@code Config.fireAtT},
     *                          braking at {@code Config.returnBrakingStart}</li>
     *   <li>{@code Artifact*} → intake roller on at T={@code Config.intakeOnAtT},
     *                          braking at {@code Config.outboundBrakingStart}</li>
     *   <li>anything else (e.g. "Open Gate", "Park") → no triggers, geometry only</li>
     * </ul>
     * The first segment additionally spins up flywheels at the start of the follow.
     *
     * <p>All waypoints + control points are mirrored for the active alliance via
     * {@link AutoField#poseForAlliance}.
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

        List<Command> segmentCommands = new ArrayList<>(pp.lines.size());
        Pose prev = chainStart;

        for (int i = 0; i < pp.lines.size(); i++) {
            ParsedLine seg = pp.lines.get(i);
            Pose end = AutoField.poseForAlliance(seg.endX, seg.endY, seg.endDeg, alliance);

            PathChain segChain = buildSegmentChain(follower, seg, prev, end, alliance);

            Command followCmd = PedroCommands.follow(follower, segChain, true, Config.maxPathPower);
            Command decorated = attachTriggers(followCmd, follower, seg, i == 0,
                    launcher, intake, fireAll);
            segmentCommands.add(decorated);

            prev = end;
        }

        Command segmentSequence = Groups.sequential(segmentCommands.toArray(new Command[0]));

        // SpinUp runs in parallel with the whole auto so SHORT_AUTO RPM targets are
        // always set, even if a per-segment callback clears them between fires.
        return Groups.deadline(
                segmentSequence,
                PresetRangeSpinCommand.create(
                        launcher, LauncherRange.SHORT_AUTO, false,
                        robot.drive, robot.lighting, null)
        );
    }

    private static PathChain buildSegmentChain(Follower follower, ParsedLine seg,
                                                Pose prev, Pose end, Alliance alliance) {
        PathBuilder b = follower.pathBuilder();

        // Geometry: 0 control points → BezierLine; 1+ → BezierCurve.
        if (seg.controlPoints.isEmpty()) {
            b.addPath(new BezierLine(prev, end));
        } else if (seg.controlPoints.size() == 1) {
            double[] c = seg.controlPoints.get(0);
            Pose ctrl = AutoField.poseForAlliance(c[0], c[1], 0, alliance);
            b.addPath(new BezierCurve(prev, ctrl, end));
        } else {
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
            double mirroredConstantRad =
                    AutoField.poseForAlliance(0, 0, seg.constantDeg, alliance).getHeading();
            b.setConstantHeadingInterpolation(mirroredConstantRad);
        } else {
            b.setLinearHeadingInterpolation(prev.getHeading(), end.getHeading(),
                    Config.headingInterpEnd);
        }

        // Per-segment braking based on name (set on PathBuilder; applies to most recent path).
        String n = seg.name == null ? "" : seg.name.toLowerCase();
        if (n.contains("launch")) {
            b.setBrakingStart(Config.returnBrakingStart);
        } else if (n.contains("artifact")) {
            b.setBrakingStart(Config.outboundBrakingStart);
        }

        if (seg.reverse) {
            b.setReversed();
        }
        return b.build();
    }

    /**
     * Wraps {@code follow} with Ivy {@code .with(...)} hooks that fire mid-segment:
     * spin-up at follow start (first segment only), fire on Launch* at T≥fireAtT,
     * intake-on on Artifact* at T≥intakeOnAtT.
     */
    private static Command attachTriggers(Command follow, Follower follower, ParsedLine seg,
                                           boolean isFirst, LauncherSubsystem launcher,
                                           IntakeSubsystem intake, Runnable fireAll) {
        String n = seg.name == null ? "" : seg.name.toLowerCase();
        if (isFirst) {
            // Instant runs at the start of the parallel group, before the follower has moved.
            follow = follow.with(Commands.instant(launcher::spinUpAllLanesToLaunch));
        }
        if (n.contains("launch")) {
            follow = follow.with(
                    Commands.waitUntil(() -> follower.getCurrentTValue() >= Config.fireAtT)
                            .then(Commands.instant(fireAll))
            );
        } else if (n.contains("artifact")) {
            follow = follow.with(
                    Commands.waitUntil(() -> follower.getCurrentTValue() >= Config.intakeOnAtT)
                            .then(Commands.instant(intake::forwardRoller))
            );
        }
        return follow;
    }
}
