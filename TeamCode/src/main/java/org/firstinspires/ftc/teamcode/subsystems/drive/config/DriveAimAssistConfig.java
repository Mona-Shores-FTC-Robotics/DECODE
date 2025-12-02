package org.firstinspires.ftc.teamcode.subsystems.drive.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Aim assist configuration for geometry-based targeting.
 * Controls how the robot rotates to face a target based on field position.
 */
@Configurable
public class DriveAimAssistConfig {
    /** Proportional gain for geometry-based aiming when error is large */
    public double kP = .65;
    /** Inner-zone proportional gain (used when inside innerZoneDeg) to avoid overshoot */
    public double kPInner = 1.2;
    /** Error threshold (deg) where the controller switches to inner kP */
    public double innerZoneDeg = 12.0;
    /** Derivative gain on heading error rate (helps damp waggle) */
    public double kD = 0.01;
    /** Max turn speed when aiming (0.0-1.0) */
    public double kMaxTurn = 0.6;
    /** Minimum turn command to overcome drivetrain static friction */
    public double kStatic = 0.080;
    /** Apply kStatic only when error is above this magnitude (deg) */
    public double staticApplyAboveDeg = 1.0;
    /** Turn command slew rate (units per second), 0 disables slew limiting */
    public double turnSlewRatePerSec = 4.0;
    /** Heading deadband (degrees) where aim is considered settled */
    public double deadbandDeg = 2.0;
}
