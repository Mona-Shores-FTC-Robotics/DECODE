package org.firstinspires.ftc.teamcode.subsystems.drive.config;

/**
 * Fixed angle aim configuration.
 * Controls aiming at fixed alliance-specific target headings.
 */
public class DriveFixedAngleAimConfig {
    /** Fixed target heading for blue alliance (degrees, 0=forward, 90=left) */
    public double blueHeadingDeg = 133.9;
    /** Fixed target heading for red alliance (degrees, 0=forward, 90=left) */
    public double redHeadingDeg = 45.2;
    /** Proportional gain for fixed-angle aiming */
    public double kP = 0.5;
    /** Max turn speed when aiming (0.0-1.0) */
    public double kMaxTurn = 0.7;
}
