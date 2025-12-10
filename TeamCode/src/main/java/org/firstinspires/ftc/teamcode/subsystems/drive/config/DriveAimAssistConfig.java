package org.firstinspires.ftc.teamcode.subsystems.drive.config;

/**
 * Aim assist configuration for geometry-based targeting.
 * Controls how the robot rotates to face a target based on field position.
 *
 * Configuration Pattern:
 * - Values with defaults are SHARED across both robots (same hardware/behavior)
 * - Values without defaults are ROBOT-SPECIFIC (tune per robot in Dashboard)
 * - If you edit a value with a default here and nothing changes, check if it's
 *   being overridden in the robot-specific create methods in DriveSubsystem
 */
public class DriveAimAssistConfig {
    // ===== SHARED VALUES (same for both robots by default) =====
    /** Proportional gain for geometry-based aiming when error is large */
    public double kP = 0.35;
    /** Inner-zone proportional gain (used when inside innerZoneDeg) to avoid overshoot */
    public double kPInner = 2.85;
    /** Error threshold (deg) where the controller switches to inner kP */
    public double innerZoneDeg = 3.5;
    /** Derivative gain on heading error rate (helps damp waggle) */
    public double kD = 0.05;
    /** Max turn speed when aiming (0.0-1.0) */
    public double kMaxTurn = 0.6;
    /** Minimum turn command to overcome drivetrain static friction */
    public double kStatic = 0.1;
    /** Apply kStatic only when error is above this magnitude (deg) */
    public double staticApplyAboveDeg = 3.0;
    /** Turn command slew rate (units per second), 0 disables slew limiting */
    public double turnSlewRatePerSec = 8.0;
    /** Heading deadband (degrees) where aim is considered settled */
    public double deadbandDeg = 2.0;
}
