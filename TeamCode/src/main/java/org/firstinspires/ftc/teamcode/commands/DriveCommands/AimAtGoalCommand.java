package org.firstinspires.ftc.teamcode.commands.DriveCommands;

import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.core.commands.Command;

import com.pedropathing.geometry.Pose;
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
public class AimAtGoalCommand extends Command {

    public static class Config {
        /** Heading tolerance in degrees - command completes when within this error */
        public double headingToleranceDeg = 2.0;

        /** Speed threshold in inches/sec - robot should be nearly stationary to complete (<= 0 disables speed gate) */
        public double stationaryThresholdIps = 1.5;

        /** Number of consecutive loops within tolerance before completing */
        public int settledLoops = 2;

        /** Maximum time to spend aiming before giving up (milliseconds) */
        public double timeoutMs = 3000;
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

        // Break any existing path following before issuing a heading hold
        if (drive.getFollower().isBusy()) {
            drive.getFollower().breakFollowing();
        }

        // Kick off a holdPoint to the current XY with the computed basket heading
        Pose target = computeTargetPose();
        if (target != null) {
            drive.getFollower().holdPoint(target);
        }
    }

    @Override
    public void update() {
        Pose target = computeTargetPose();
        if (target != null) {
            // Refresh the target each loop in case pose drifts during the turn
            drive.getFollower().holdPoint(target);
        }

        double headingErrorDeg = computeHeadingErrorDeg();
        double speedIps = drive.getRobotSpeedInchesPerSecond();

        // Publish diagnostics
        RobotState.packet.put("AimAtGoal/Heading Error (deg)", headingErrorDeg);
        RobotState.packet.put("AimAtGoal/Robot Speed (ips)", speedIps);
        RobotState.packet.put("AimAtGoal/Settled Loops", loopsSettled);
        RobotState.packet.put("AimAtGoal/Is Aimed", isAimedAndSettled(headingErrorDeg, speedIps));
        RobotState.packet.put("AimAtGoal/Mode", "PedroHoldPoint");
        RobotState.packet.put("AimAtGoal/Follower Busy", drive.getFollower().isBusy());

        // Check if we're aimed and settled
        if (isAimedAndSettled(headingErrorDeg, speedIps)) {
            loopsSettled++;
        } else {
            loopsSettled = 0;
        }

        // If follower reports done (e.g., holdPoint finished), force a settle to avoid immediate exit
        if (!drive.getFollower().isBusy() && loopsSettled == 0) {
            loopsSettled = 1;
        }
    }

    @Override
    public boolean isDone() {
        // Timeout safety
        if (System.currentTimeMillis() - startTimeMs >= config.timeoutMs) {
            return true;
        }

        // Complete only when settled; follower busy flag alone is noisy for holdPoint
        return loopsSettled >= config.settledLoops;
    }

    @Override
    public void stop(boolean interrupted) {
        loopsSettled = 0;

        // Stop turning and hold position
        drive.getFollower().breakFollowing();
        drive.getFollower().setTeleOpDrive(0.0, 0.0, 0.0, false);
    }

    /**
     * Checks if robot is aimed at target and stationary.
     * @param headingErrorDeg Current heading error in degrees (absolute value)
     */
    private boolean isAimedAndSettled(double headingErrorDeg, double speedIps) {
        boolean headingOk = Math.abs(headingErrorDeg) <= config.headingToleranceDeg;
        boolean speedOk = config.stationaryThresholdIps <= 0.0
                || speedIps <= config.stationaryThresholdIps;
        return headingOk && speedOk;
    }

    /**
     * Builds a holdPoint target at the current XY with heading aimed at the basket.
     */
    private Pose computeTargetPose() {
        Pose pose = drive.getFollower().getPose();
        if (pose == null) {
            return null;
        }
        Alliance alliance = vision.getAlliance();
        Pose targetPose = (alliance == Alliance.RED)
                ? FieldConstants.getRedBasketTarget()
                : FieldConstants.getBlueBasketTarget();
        double targetHeading = FieldConstants.getAimAngleTo(pose, targetPose);
        return new Pose(pose.getX(), pose.getY(), targetHeading);
    }

    /**
     * Returns the current heading error (deg) between robot heading and basket target.
     */
    private double computeHeadingErrorDeg() {
        Pose pose = drive.getFollower().getPose();
        if (pose == null) {
            return Double.NaN;
        }
        double robotHeading = drive.getFollower().getHeading();
        Alliance alliance = vision.getAlliance();
        Pose targetPose = (alliance == Alliance.RED)
                ? FieldConstants.getRedBasketTarget()
                : FieldConstants.getBlueBasketTarget();
        double targetHeading = FieldConstants.getAimAngleTo(pose, targetPose);
        double errorRad = normalizeAngle(targetHeading - robotHeading);
        return Math.toDegrees(errorRad);
    }

    private static double normalizeAngle(double angleRad) {
        while (angleRad > Math.PI) angleRad -= 2 * Math.PI;
        while (angleRad <= -Math.PI) angleRad += 2 * Math.PI;
        return angleRad;
    }
}
