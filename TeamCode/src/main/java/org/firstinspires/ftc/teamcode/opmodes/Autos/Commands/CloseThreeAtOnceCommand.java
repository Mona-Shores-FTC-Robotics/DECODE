package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.AutoSmartIntakeCommand;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;

/**
 * Generated autonomous command from Pedro Pathing .pp file
 * Usage in OpMode:
 *   Command auto = LocalizeCommand.create(robot, activeAlliance);
 *   CommandManager.INSTANCE.scheduleCommand(auto);
 * 
 */
@Configurable
public class CloseThreeAtOnceCommand {

    @Configurable
    public static class Config {
        public double maxPathPower = 0.75;
        public double endTimeForLinearHeadingInterpolation = .7;
        public double autoDurationSeconds = 30.0;
        public double minTimeForFinalLaunchSeconds = 5.0;

        // Pedro PathBuilder constraints (now exposed for tuning!)
        // Set to -1 to use Pedro's built-in defaults, or set explicit value to override

        /** Braking strength multiplier (default -1 = use Pedro default of 1.0) */
        public double brakingStrength = -1;

        /** When braking starts 0.0-1.0 (default -1 = use Pedro default of 1.0) */
        public double brakingStart = -1;

        /** Translational constraint in inches (default -1 = use Pedro default of 0.1") */
        public double translationalConstraint = -1;

        /** Heading constraint in radians (default -1 = use Pedro default of 0.007 rad = ~0.4°) */
        public double headingConstraint = -1;

        /** Timeout in ms for final corrections (default -1 = use Pedro default of 100ms) */
        public double timeoutConstraintMs = -1;
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

        // LaunchClose2
        public double launchClose2X = 30.0;
        public double launchClose2Y = 113.0;
        public double launchClose2Heading = 134.0;

        // ArtifactsSet2
        public double artifactsSet2X = 23.75;
        public double artifactsSet2Y = 61.0;
        public double artifactsSet2Heading = 270;

        // Control point for segment: ArtifactsSet2
        public double artifactsSet2Control0X = 23.75;
        public double artifactsSet2Control0Y = 70;

        // LaunchClose3
        public double launchClose3X = 30.0;
        public double launchClose3Y = 113.0;
        public double launchClose3Heading = 134.0;

        // Control point for segment: LaunchClose3
        public double launchClose3Control0X = 50.5;
        public double launchClose3Control0Y = 72;

        // ArtifactsSet3
        public double artifactsSet3X = 23.75;
        public double artifactsSet3Y = 35.5;
        public double artifactsSet3Heading = 270.0;

        // Control point for segment: ArtifactsSet3
        public double artifactsSet3Control0X = 23.75;
        public double artifactsSet3Control0Y = 72;

        // LaunchClose4
        public double launchClose4X = 30;
        public double launchClose4Y = 113.0;
        public double launchClose4Heading = 134.0;

        // Control point for segment: LaunchOffLine
        public double launchClose4Control0X = 44.5;
        public double launchClose4Control0Y = 73.5;

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

    private CloseThreeAtOnceCommand() {}

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
     * Uses native Pedro PathBuilder API for full control over path constraints.
     * @param robot Robot instance with all subsystems
     * @param alliance Current alliance (BLUE or RED)
     * @param startOverride Vision-detected start pose (or null to use waypoints)
     * @return Complete autonomous command
     */
    public static Command create(Robot robot, Alliance alliance, Pose startOverride) {
        LauncherCommands launcherCommands = new LauncherCommands(robot.launcher, robot.intake, robot.drive, robot.lighting);
        AutoSmartIntakeCommand autoSmartIntake = new AutoSmartIntakeCommand(robot.intake);

        // Determine actual start pose (vision override or waypoint default)
        Pose actualStart = startOverride != null ? robot.drive.getFollower().getPose() : mirror(start(), alliance);

        Command mainSequence = new SequentialGroup(
                // Reset timer when auto actually starts (not when command is created)
                ConditionalFinalLaunchCommand.createTimerReset(),

                // Launch Preloads
                new ParallelDeadlineGroup(
                    buildPath(robot, actualStart, mirror(launchClose1(), alliance), null, false, config.maxPathPower),
                    launcherCommands.presetRangeSpinUp(LauncherRange.SHORT_AUTO, true)
                ),

                launcherCommands.launchAccordingToMode(false),

                // Pickup Artifact Set 1
                buildPath(robot, mirror(launchClose1(), alliance), mirror(artifactsSet1(), alliance),
                        mirror(artifactsSet1Control0(), alliance), true, config.maxPathPower),

                // Return and Launch Set 1
                buildPath(robot, mirror(artifactsSet1(), alliance), mirror(launchClose2(), alliance),
                        null, false, config.maxPathPower),

                launcherCommands.launchAccordingToMode(false),

                // Pickup Artifact Set 2
                buildPath(robot, mirror(launchClose2(), alliance), mirror(artifactsSet2(), alliance),
                        mirror(artifactsSet2Control0(), alliance), true, config.maxPathPower),

                // Return and Launch Set 2
                buildPath(robot, mirror(artifactsSet2(), alliance), mirror(launchClose3(), alliance),
                        mirror(launchClose3Control0(), alliance), false, config.maxPathPower),

                launcherCommands.launchAccordingToMode(false),

                // Pickup Artifact Set 3
                buildPath(robot, mirror(launchClose3(), alliance), mirror(artifactsSet3(), alliance),
                        mirror(artifactsSet3Control0(), alliance), true, config.maxPathPower),

                // Conditionally return and launch if time permits, otherwise go straight to park
                new ConditionalFinalLaunchCommand(
                        config.autoDurationSeconds,
                        config.minTimeForFinalLaunchSeconds,
                        // If enough time: return to launch, shoot, then park
                        new SequentialGroup(
                                buildPath(robot, mirror(artifactsSet3(), alliance), mirror(launchClose4(), alliance),
                                        mirror(launchClose4Control0(), alliance), false, config.maxPathPower),

                                launcherCommands.launchAccordingToMode(false),

                                buildPath(robot, mirror(launchClose4(), alliance), mirror(nearGate(), alliance),
                                        mirror(nearGateControl0(), alliance), false, config.maxPathPower)
                        ),
                        // If not enough time: go straight to park
                        new SequentialGroup(
                                buildPath(robot, mirror(artifactsSet3(), alliance), mirror(nearGate(), alliance),
                                        null, false, config.maxPathPower)
                        )
                )
        );

        return new ParallelDeadlineGroup(
                mainSequence,
                autoSmartIntake
        );
    }

    /**
     * Builds a path using native Pedro PathBuilder API with full constraint control.
     * @param robot Robot instance
     * @param start Start pose (already mirrored for alliance)
     * @param end End pose (already mirrored for alliance)
     * @param control Optional control point for bezier curve (already mirrored)
     * @param constantHeading If true, use constant heading at end pose heading; if false, use linear interpolation
     * @param maxPower Maximum power for path following
     * @return FollowPath command
     */
    private static Command buildPath(Robot robot, Pose start, Pose end, Pose control,
                                     boolean constantHeading, double maxPower) {
        return buildPath(robot, start, end, control, constantHeading, maxPower, -1);
    }

    /**
     * Build a path with optional per-path timeout override.
     * @param timeoutMs Per-path timeout in milliseconds. Use -1 to use config default.
     */
    private static Command buildPath(Robot robot, Pose start, Pose end, Pose control,
                                     boolean constantHeading, double maxPower, double timeoutMs) {
        PathBuilder builder = robot.drive.getFollower().pathBuilder();

        // Add path geometry
        if (control == null) {
            builder.addPath(new BezierLine(start, end));
        } else {
            builder.addPath(new BezierCurve(start, control, end));
        }

        // Set heading interpolation
        if (constantHeading) {
            builder.setConstantHeadingInterpolation(end.getHeading());
        } else {
            builder.setLinearHeadingInterpolation(start.getHeading(), end.getHeading(),
                    config.endTimeForLinearHeadingInterpolation);
        }

        // Apply Pedro path constraints only if explicitly overridden (not -1)
        if (config.brakingStrength >= 0) {
            builder.setBrakingStrength(config.brakingStrength);
        }
        if (config.brakingStart >= 0) {
            builder.setBrakingStart(config.brakingStart);
        }
        if (config.translationalConstraint >= 0) {
            builder.setTranslationalConstraint(config.translationalConstraint);
        }
        if (config.headingConstraint >= 0) {
            builder.setHeadingConstraint(config.headingConstraint);
        }

        // Per-path timeout override takes precedence over config default
        if (timeoutMs >= 0) {
            builder.setTimeoutConstraint(timeoutMs);
        } else if (config.timeoutConstraintMs >= 0) {
            builder.setTimeoutConstraint(config.timeoutConstraintMs);
        }

        PathChain chain = builder.build();

        return new FollowPath(chain, false, maxPower);
    }

    /**
     * Mirrors a pose for the red alliance.
     * @param pose Pose in blue alliance coordinates
     * @param alliance Current alliance
     * @return Mirrored pose for red, or original pose for blue
     */
    private static Pose mirror(Pose pose, Alliance alliance) {
        return AutoField.poseForAlliance(
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading()),
                alliance
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

    private static Pose launchClose2() {
        return new Pose(waypoints.launchClose2X, waypoints.launchClose2Y, Math.toRadians(waypoints.launchClose2Heading));
    }

    private static Pose artifactsSet2() {
        return new Pose(waypoints.artifactsSet2X, waypoints.artifactsSet2Y, Math.toRadians(waypoints.artifactsSet2Heading));
    }

    private static Pose artifactsSet2Control0() {
        return new Pose(waypoints.artifactsSet2Control0X, waypoints.artifactsSet2Control0Y, 0);
    }

//    private static Pose artifactsSet2Control1() {
//        return new Pose(waypoints.artifactsSet2Control1X, waypoints.artifactsSet2Control1Y, 0);
//    }

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

//    private static Pose artifactsSet3Control1() {
//        return new Pose(waypoints.artifactsSet3Control1X, waypoints.artifactsSet3Control1Y, 0);
//    }

    private static Pose launchClose4() {
        return new Pose(waypoints.launchClose4X , waypoints.launchClose4Y , Math.toRadians(waypoints.launchClose4Heading));
    }

    private static Pose launchClose4Control0() {
        return new Pose(waypoints.launchClose4Control0X , waypoints.launchClose4Control0Y , 0);
    }

    private static Pose nearGate() {
        return new Pose(waypoints.nearGateX, waypoints.nearGateY, Math.toRadians(waypoints.nearGateHeading));
    }

    private static Pose nearGateControl0() {
        return new Pose(waypoints.nearGateControl0X, waypoints.nearGateControl0Y, 0);
    }

}
