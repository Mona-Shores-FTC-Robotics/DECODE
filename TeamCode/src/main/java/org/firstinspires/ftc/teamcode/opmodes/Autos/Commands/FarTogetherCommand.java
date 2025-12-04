package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.TryRelocalizeForShotCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.AutoSmartIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherCommands;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
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
public class FarTogetherCommand {

    @Configurable
    public static class Config {
        public double maxPathPower = .85;
        public double endTimeForLinearHeadingInterpolation = .7;
        public double delayForGateToOpen = 1.0;
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
        public double startX = 56;
        public double startY = 5;
        public double startHeading = 90.0;

        // LaunchFar
        public double launchFarX = 58;
        public double launchFarY = 11;
        public double launchFarHeadingDeg = 108;

        // Artifacts at Alliance Wall)
        public double artifactsWallX = 13;
        public double artifactsWallY = 6.5;
        public double artifactsWallHeading = 180;

        // Control point for segment: leavewall
        public double wallControlPointX = 27.5;
        public double wallControlPointY = 10;


        // Chute Released Artifacts Try 1
        public double releasedTry1X = 13;
        public double releasedTry1Y = 23;
        public double releasedTry1Heading = 180;

        // Chute Released Artifacts Try 1
        public double releasedTry2X = 13;
        public double releasedTry2Y = 37;
        public double releasedTry2Heading = 180;

        public double releasedTry2controlX = 35.3;
        public double releasedTry2controlY = 13.6;

        // Off Line
        public double readyForTeleopX = 58;
        public double readyForTeleopY = 36;
        public double readyForTeleopHeading = 0;

    }

    public static Config config = new Config();
    public static Waypoints waypoints = new Waypoints();

    private FarTogetherCommand() {}

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
                        buildPath(robot, actualStart, mirror(launchFar(), alliance), null, true, config.maxPathPower),
                        launcherCommands.presetRangeSpinUp(LauncherRange.FAR_AUTO, true)
                ),

                new TryRelocalizeForShotCommand(robot.drive, robot.vision),
                launcherCommands.launchAccordingToMode(false),

                // Pickup Alliance Wall Artifacts
                buildPath(robot, mirror(launchFar(), alliance), mirror(artifactsAllianceWall(), alliance),
                        mirror(wallControl(), alliance), true, config.maxPathPower),

                // Return and launch alliance wall artifacts
                buildPath(robot, mirror(artifactsAllianceWall(), alliance), mirror(launchFar(), alliance),
                        mirror(wallControl(), alliance), false, config.maxPathPower),

                new TryRelocalizeForShotCommand(robot.drive, robot.vision),
                new Delay(config.delayForGateToOpen),
                launcherCommands.launchAccordingToMode(false),

                // Pickup Released Artifacts Try 1
                buildPath(robot, mirror(launchFar(), alliance), mirror(artifactsAllianceWall(), alliance),
                        mirror(wallControl(), alliance), true, config.maxPathPower),

                // Return and Launch
                buildPath(robot, mirror(artifactsAllianceWall(), alliance), mirror(launchFar(), alliance),
                        mirror(wallControl(), alliance), false, config.maxPathPower),

                new TryRelocalizeForShotCommand(robot.drive, robot.vision),
                launcherCommands.launchAccordingToMode(false),

                // Pickup Released Artifacts Try 2
                buildPath(robot, mirror(launchFar(), alliance), mirror(artifactsAllianceWall(), alliance),
                        mirror(wallControl(), alliance), true, config.maxPathPower),

                // Conditionally return and launch if time permits, otherwise go straight to park
                new ConditionalFinalLaunchCommand(
                        config.autoDurationSeconds,
                        config.minTimeForFinalLaunchSeconds,
                        // If enough time: return to launch, shoot, then park
                        new SequentialGroup(
                                buildPath(robot, mirror(artifactsAllianceWall(), alliance), mirror(launchFar(), alliance),
                                        mirror(wallControl(), alliance), false, config.maxPathPower),

                                new TryRelocalizeForShotCommand(robot.drive, robot.vision),
                                launcherCommands.launchAccordingToMode(false),

                                buildPath(robot, mirror(launchFar(), alliance), mirror(readyForTeleop(), alliance),
                                        null, false, config.maxPathPower)
                        ),
                        // If not enough time: go straight to park
                        new SequentialGroup(
                                buildPath(robot, mirror(artifactsAllianceWall(), alliance), mirror(readyForTeleop(), alliance),
                                        null, true, config.maxPathPower)
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
        if (config.timeoutConstraintMs >= 0) {
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

    private static Pose launchFar() {
        return new Pose(waypoints.launchFarX, waypoints.launchFarY, Math.toRadians(waypoints.launchFarHeadingDeg));
    }

    private static Pose artifactsAllianceWall() {
        return new Pose(waypoints.artifactsWallX, waypoints.artifactsWallY, Math.toRadians(waypoints.artifactsWallHeading));
    }

    private static Pose wallControl() {
        return new Pose(waypoints.wallControlPointX , waypoints.wallControlPointY , 0);
    }

    private static Pose releasedArtifacts1() {
        return new Pose(waypoints.releasedTry1X , waypoints.releasedTry1Y , Math.toRadians(waypoints.releasedTry1Heading));
    }

    private static Pose releasedArtifacts2() {
        return new Pose(waypoints.releasedTry2X , waypoints.releasedTry2Y , Math.toRadians(waypoints.releasedTry2Heading));
    }

    private static Pose releasedArtifacts2Control0() {
        return new Pose(waypoints.releasedTry2controlX, waypoints.releasedTry2controlY, 0);
    }

    private static Pose readyForTeleop() {
        return new Pose(waypoints.readyForTeleopX, waypoints.readyForTeleopY, Math.toRadians(waypoints.readyForTeleopHeading));
    }
}
