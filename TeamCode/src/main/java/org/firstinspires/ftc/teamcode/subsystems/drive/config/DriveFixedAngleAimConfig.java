package org.firstinspires.ftc.teamcode.subsystems.drive.config;

/**
 * Fixed angle aim configuration.
 * Controls aiming at fixed alliance-specific target headings.
 *
 * Configuration Pattern:
 * - Values with defaults are SHARED across both robots (same hardware/behavior)
 * - Values without defaults are ROBOT-SPECIFIC (tune per robot in Dashboard)
 * - If you edit a value with a default here and nothing changes, check if it's
 *   being overridden in the robot-specific create methods in DriveSubsystem
 */
public class DriveFixedAngleAimConfig {
    // ===== SHARED VALUES (same for both robots) =====
    /** Fixed target heading for blue alliance (degrees, 0=forward, 90=left) */
    public double blueHeadingDeg = 135.2;

    /** Fixed target heading for red alliance (degrees, 0=forward, 90=left) */
    public double redHeadingDeg = 45.2;

    /** Proportional gain for fixed-angle aiming */
    public double kP = 0.5;

    /** Max turn speed when aiming (0.0-1.0) */
    public double kMaxTurn = 0.7;

    // ===== ROBOT-SPECIFIC VALUES (no defaults - set per robot) =====
    // (none currently - all values are shared)
}
