package org.firstinspires.ftc.teamcode.telemetry.data;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bindings.DriverBindings;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FieldConstants;

/**
 * Drive subsystem telemetry data.
 * Shows commanded robot behavior (after processing) and actual motor outputs.
 * <p>
 * For raw driver inputs, see GamepadTelemetryData.
 * </p>
 */
public class DriveTelemetryData {
    // Drive mode and state
    public final String driveMode;
    public final boolean aimMode;
    public final boolean slowMode;

    // Commanded values (after processing)
    public final double commandDrive;
    public final double commandStrafe;
    public final double commandTurn;

    // Motor data (organized by motor)
    public final MotorData leftFront;
    public final MotorData rightFront;
    public final MotorData leftBack;
    public final MotorData rightBack;

    // Distance to goal (based on odometry)
    public final double distanceToGoalIn;

    public DriveTelemetryData(
            String driveMode,
            boolean aimMode,
            boolean slowMode,
            double commandDrive,
            double commandStrafe,
            double commandTurn,
            MotorData leftFront,
            MotorData rightFront,
            MotorData leftBack,
            MotorData rightBack,
            double distanceToGoalIn
    ) {
        this.driveMode = driveMode;
        this.aimMode = aimMode;
        this.slowMode = slowMode;
        this.commandDrive = commandDrive;
        this.commandStrafe = commandStrafe;
        this.commandTurn = commandTurn;
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.distanceToGoalIn = distanceToGoalIn;
    }

    /**
     * Individual motor telemetry data.
     */
    public static class MotorData {
        public final double power;
        public final double velocityIps;

        public MotorData(double power, double velocityTicksPerSec) {
            this.power = power;
            this.velocityIps = DistanceUnit.METER.toInches(
                    Constants.Speed.ticksPerSecToMps(velocityTicksPerSec)
            );
        }
    }

    public static DriveTelemetryData capture(DriveSubsystem drive, DriverBindings.DriveRequest request, VisionSubsystemLimelight vision) {
        boolean slowMode = request != null && request.slowMode;
        boolean aimMode = request != null && request.aimMode;

        MotorData lf = new MotorData(drive.getLfPower(), drive.getLfVelocityTicksPerSec());
        MotorData rf = new MotorData(drive.getRfPower(), drive.getRfVelocityTicksPerSec());
        MotorData lb = new MotorData(drive.getLbPower(), drive.getLbVelocityTicksPerSec());
        MotorData rb = new MotorData(drive.getRbPower(), drive.getRbVelocityTicksPerSec());

        // Calculate distance to goal using odometry
        double distanceToGoal = calculateDistanceToGoal(drive, vision);

        return new DriveTelemetryData(
                drive.getDriveMode().name(),
                aimMode,
                slowMode,
                drive.getLastCommandDrive(),
                drive.getLastCommandStrafe(),
                drive.getLastCommandTurn(),
                lf, rf, lb, rb,
                distanceToGoal
        );
    }

    /**
     * Calculates distance from current robot pose to goal basket.
     * Uses odometry pose and alliance-specific goal coordinates.
     *
     * @param drive Drive subsystem (for odometry pose)
     * @param vision Vision subsystem (for alliance)
     * @return Distance to goal in inches, or NaN if unavailable
     */
    private static double calculateDistanceToGoal(DriveSubsystem drive, VisionSubsystemLimelight vision) {
        // Get current robot pose from odometry
        Pose robotPose = drive.getFollower().getPose();
        if (robotPose == null) {
            return Double.NaN;
        }

        // Get goal pose based on alliance
        Alliance alliance = vision.getAlliance();

        // Debug: publish alliance and goal coordinates
        org.firstinspires.ftc.teamcode.util.RobotState.packet.put("drive/distance_calc_alliance", alliance.name());

        Pose goalPose = (alliance == Alliance.RED)
                ? FieldConstants.getRedBasketTarget()
                : FieldConstants.getBlueBasketTarget();

        // Debug: publish goal coordinates being used
        org.firstinspires.ftc.teamcode.util.RobotState.packet.put("drive/goal_x", goalPose.getX());
        org.firstinspires.ftc.teamcode.util.RobotState.packet.put("drive/goal_y", goalPose.getY());
        org.firstinspires.ftc.teamcode.util.RobotState.packet.put("drive/robot_x", robotPose.getX());
        org.firstinspires.ftc.teamcode.util.RobotState.packet.put("drive/robot_y", robotPose.getY());

        // Calculate Euclidean distance
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        return Math.hypot(dx, dy);
    }
}
