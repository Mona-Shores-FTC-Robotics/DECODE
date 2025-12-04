package org.firstinspires.ftc.teamcode.subsystems.drive.config;

/**
 * Vision-centered aim configuration.
 * Controls how the robot rotates to center a vision target using camera feedback (tx).
 */
public class DriveVisionCenteredAimConfig {
    /** Proportional gain for vision-centered aiming (motor power per radian) */
    public double kP = 1.7;  // Equivalent to 0.03 per degree
    /** Max turn speed when aiming (0.0-1.0) */
    public double kMaxTurn = 0.7;
    /** Deadband - stop turning when tx error is below this (degrees) */
    public double deadbandDeg = 1.0;
}
