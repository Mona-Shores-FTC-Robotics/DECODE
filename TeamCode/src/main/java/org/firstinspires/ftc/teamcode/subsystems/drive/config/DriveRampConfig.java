package org.firstinspires.ftc.teamcode.subsystems.drive.config;

/**
 * Drive ramping configuration.
 * Controls acceleration rate limits for smooth drive input changes.
 */
public class DriveRampConfig {
    /** Forward acceleration rate limit (power units per second) */
    public double forwardRatePerSec = 0.3;
    /** Strafe acceleration rate limit (power units per second) */
    public double strafeRatePerSec = 0.3;
    /** Turn acceleration rate limit (power units per second) */
    public double turnRatePerSec = 0.6;
    /** Fallback delta time if loop timing unavailable (seconds) */
    public double fallbackDtSeconds = 0.02;
}
