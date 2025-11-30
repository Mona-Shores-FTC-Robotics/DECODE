package org.firstinspires.ftc.teamcode.commands.DriveCommands;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
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
public class AimAtGoalCommand extends Command {

    @Configurable
    public static class Config {
        /** Heading tolerance in degrees - command completes when within this error */
        public double headingToleranceDeg = 2.0;

        /** Speed threshold in inches/sec - robot must be stationary to complete */
        public double stationaryThresholdIps = 1.5;

        /** Number of consecutive loops within tolerance before completing */
        public int settledLoops = 3;

        /** Maximum time to spend aiming before giving up (milliseconds) */
        public double timeoutMs = 3000.0;
    }

    public static Config config = new Config();

    private final DriveSubsystem drive;
    private final VisionSubsystemLimelight vision;

    private long startTimeMs = -1L;
    private int loopsSettled = 0;

    public AimAtGoalCommand(DriveSubsystem drive, VisionSubsystemLimelight vision) {
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
    }

    @Override
    public void update() {
        // Reuse shared aim controller (same as TeleOp) for consistent tuning and telemetry
        drive.aimAndDrive(0.0, 0.0, false);

        double headingErrorDeg = Math.toDegrees(drive.getLastAimErrorRadians());
        double speedIps = drive.getRobotSpeedInchesPerSecond();

        // Publish diagnostics
        RobotState.packet.put("AimAtGoal/Heading Error (deg)", headingErrorDeg);
        RobotState.packet.put("AimAtGoal/Robot Speed (ips)", speedIps);
        RobotState.packet.put("AimAtGoal/Settled Loops", loopsSettled);
        RobotState.packet.put("AimAtGoal/Is Aimed", isAimedAndSettled(headingErrorDeg, speedIps));
        RobotState.packet.put("AimAtGoal/Mode", "AimAndDrive");

        // Check if we're aimed and settled
        if (isAimedAndSettled(headingErrorDeg, speedIps)) {
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
        drive.getFollower().setTeleOpDrive(0.0, 0.0, 0.0, false);
    }

    /**
     * Checks if robot is aimed at target and stationary.
     * @param headingErrorDeg Current heading error in degrees (absolute value)
     */
    private boolean isAimedAndSettled(double headingErrorDeg, double speedIps) {
        return Math.abs(headingErrorDeg) <= config.headingToleranceDeg
                && speedIps <= config.stationaryThresholdIps;
    }
}
