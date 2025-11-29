package org.firstinspires.ftc.teamcode.commands.DriveCommands;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.Objects;

/**
 * Autonomous aiming command that uses continuous heading tracking instead of discrete turnTo().
 *
 * This command continuously applies DriveSubsystem.aimAndDrive() (same method used in TeleOp)
 * until the heading error is within tolerance and the robot is stationary.
 *
 * Unlike CaptureAndAimCommand which uses Pedro's turnTo() (which can get stuck),
 * this uses a proven continuous tracking approach that's already working in TeleOp.
 */
@Configurable
public class AimAtBasketCommand extends Command {

    @Configurable
    public static class Config {
        /** Heading tolerance in degrees - command completes when within this error */
        public double headingToleranceDeg = 1.0;

        /** Speed threshold in inches/sec - robot must be stationary to complete */
        public double stationaryThresholdIps = 1.5;

        /** Number of consecutive loops within tolerance before completing */
        public int settledLoops = 2;

        /** Maximum time to spend aiming before giving up (milliseconds) */
        public double timeoutMs = 5000.0;

        /** Use direct motor control instead of aimAndDrive for more aggressive turning */
        public boolean useDirectTurnControl = true;

        /** Turn power for direct control mode (0.0 to 1.0) */
        public double turnPower = 0.6;

        /** P gain for direct turn control */
        public double kP = 1.2;

        /** Minimum turn power to overcome friction */
        public double kStatic = 0.15;
    }

    public static Config config = new Config();

    private final DriveSubsystem drive;
    private final VisionSubsystemLimelight vision;

    private long startTimeMs = -1L;
    private int loopsSettled = 0;

    public AimAtBasketCommand(DriveSubsystem drive, VisionSubsystemLimelight vision) {
        this.drive = Objects.requireNonNull(drive, "drive subsystem required");
        this.vision = Objects.requireNonNull(vision, "vision subsystem required");
        requires(this.drive);
        setInterruptible(true);
    }

    @Override
    public void start() {
        startTimeMs = System.currentTimeMillis();
        loopsSettled = 0;

        // Break any existing path following and switch to teleop drive mode
        if (drive.getFollower().isBusy()) {
            drive.getFollower().breakFollowing();
        }

        // Ensure follower is in teleop drive mode for direct motor control
        if (config.useDirectTurnControl) {
            drive.getFollower().startTeleopDrive();
        }
    }

    @Override
    public void update() {
        // Calculate target heading
        Pose currentPose = drive.getFollowerPose();
        Alliance alliance = vision.getAlliance();
        Pose targetPose = (alliance == Alliance.RED)
            ? FieldConstants.getRedBasketTarget()
            : FieldConstants.getBlueBasketTarget();
        double targetHeadingRad = FieldConstants.getAimAngleTo(currentPose, targetPose);
        double currentHeadingRad = drive.getFollower().getHeading();
        double headingErrorRad = normalizeAngle(targetHeadingRad - currentHeadingRad);
        double headingErrorDeg = Math.toDegrees(headingErrorRad);

        // Calculate turn command
        double turnCommand = 0.0;
        if (config.useDirectTurnControl) {
            // Direct turn control with more aggressive gains
            turnCommand = config.kP * headingErrorRad;

            // Add static feedforward to overcome friction
            if (Math.abs(headingErrorRad) > Math.toRadians(config.headingToleranceDeg)) {
                if (Math.abs(turnCommand) < config.kStatic) {
                    turnCommand = Math.copySign(config.kStatic, headingErrorRad);
                }
            } else {
                turnCommand = 0.0; // Within tolerance, stop turning
            }

            // Clamp to max power
            turnCommand = Math.max(-config.turnPower, Math.min(config.turnPower, turnCommand));

            // Apply turn directly to follower (teleop drive mode)
            drive.getFollower().setTeleOpDrive(0.0, 0.0, turnCommand, false);
        } else {
            // Use existing aimAndDrive method (less aggressive)
            drive.aimAndDrive(0.0, 0.0, false);
        }

        // Publish diagnostics
        RobotState.packet.put("AimAtBasket/Target Heading (deg)", Math.toDegrees(targetHeadingRad));
        RobotState.packet.put("AimAtBasket/Current Heading (deg)", Math.toDegrees(currentHeadingRad));
        RobotState.packet.put("AimAtBasket/Heading Error (deg)", headingErrorDeg);
        RobotState.packet.put("AimAtBasket/Turn Command", turnCommand);
        RobotState.packet.put("AimAtBasket/Robot Speed (ips)", drive.getRobotSpeedInchesPerSecond());
        RobotState.packet.put("AimAtBasket/Settled Loops", loopsSettled);
        RobotState.packet.put("AimAtBasket/Is Aimed", isAimedAndSettled(headingErrorDeg));
        RobotState.packet.put("AimAtBasket/Mode", config.useDirectTurnControl ? "Direct" : "AimAndDrive");

        // Check if we're aimed and settled
        if (isAimedAndSettled(headingErrorDeg)) {
            loopsSettled++;
        } else {
            loopsSettled = 0;
        }
    }

    @Override
    public boolean isDone() {
        // Timeout safety
        if (System.currentTimeMillis() - startTimeMs >= config.timeoutMs) {
            return true;
        }

        // Complete when settled for required number of loops
        return loopsSettled >= config.settledLoops;
    }

    @Override
    public void stop(boolean interrupted) {
        loopsSettled = 0;

        // Stop turning and hold position
        if (config.useDirectTurnControl) {
            drive.getFollower().setTeleOpDrive(0.0, 0.0, 0.0, false);
        }
    }

    /**
     * Checks if robot is aimed at target and stationary.
     * @param headingErrorDeg Current heading error in degrees (absolute value)
     */
    private boolean isAimedAndSettled(double headingErrorDeg) {
        // Check heading error
        if (Math.abs(headingErrorDeg) > config.headingToleranceDeg) {
            return false;
        }

        // Check velocity
        double speedIps = drive.getRobotSpeedInchesPerSecond();
        if (speedIps > config.stationaryThresholdIps) {
            return false;
        }

        return true;
    }

    /**
     * Normalizes an angle to [-π, π] range.
     */
    private static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
