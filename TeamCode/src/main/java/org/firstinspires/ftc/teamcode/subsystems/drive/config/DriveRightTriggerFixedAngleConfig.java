package org.firstinspires.ftc.teamcode.subsystems.drive.config;

/**
 * Right trigger fixed angle aim configuration.
 * Controls aiming at fixed alliance-specific target headings when right trigger is held.
 */
public class DriveRightTriggerFixedAngleConfig {
    /** Fixed target heading for blue alliance (degrees, 0=forward, 90=left) */
    public double blueParkHeadingDeg = 264.0;
    /** Fixed target heading for red alliance (degrees, 0=forward, 90=left) */
    public double redParkHeadingDeg = 274.0;
    /** Proportional gain for right trigger fixed-angle aiming */
    public double kP = 0.5;
    /** Max turn speed when aiming (0.0-1.0) */
    public double kMaxTurn = 0.7;
}
