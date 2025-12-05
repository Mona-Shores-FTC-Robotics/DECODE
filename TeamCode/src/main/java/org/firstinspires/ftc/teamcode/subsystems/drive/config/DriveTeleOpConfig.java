package org.firstinspires.ftc.teamcode.subsystems.drive.config;

/**
 * TeleOp drive control configuration.
 * Controls drive power multipliers and turn rates for normal and slow modes.
 */
public class DriveTeleOpConfig {
    /** Max power multiplier for normal (non-slow) teleop driving (0.0-1.0) */
    public double normalMultiplier = 0.7;
    /** Max power multiplier for slow mode teleop driving (0.0-1.0) */
    public double slowMultiplier = 0.2;
    /** Turn multiplier for normal mode */
    public double normalTurnMultiplier = 0.4;
    /** Turn multiplier for slow mode */
    public double slowTurnMultiplier = 0.2;
    /** Rotation override threshold for aim assist */
    public double rotationOverrideThreshold = 0.05;
}
