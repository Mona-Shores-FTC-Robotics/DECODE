package org.firstinspires.ftc.teamcode.subsystems.drive.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Aim assist configuration for geometry-based targeting.
 * Controls how the robot rotates to face a target based on field position.
 */
@Configurable
public class DriveAimAssistConfig {
    /** Proportional gain for geometry-based aiming - higher = faster response to error */
    public double kP = 1.0;
    /** Max turn speed when aiming (0.0-1.0) */
    public double kMaxTurn = 0.7;
    /** Minimum turn command to overcome drivetrain static friction */
    public double kStatic = 0.15;
    /** Heading deadband (degrees) where aim is considered settled */
    public double deadbandDeg = 0.5;
    /** Distance threshold for MegaTag2 relocalization (inches) */
    public double MT2DistanceThreshold = 12;
    /** Heading agreement threshold between MT1/MT2 (degrees) */
    public double MT2HeadingThresholdDeg = 15.0;
}
