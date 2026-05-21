package org.firstinspires.ftc.teamcode.subsystems.drive.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Tuning for the pose-fusion filter that blends Pinpoint odometry with
 * Limelight AprilTag measurements. The filter lives in
 * {@code pedroPathing/FusionLocalizer.java}; these values control how much it
 * trusts each input source.
 *
 * <p>Single shared global (static fields) — same for both robots.
 */
@Configurable
public class DriveFusionConfig {
    /**
     * Minimum target area % (ta) to accept a Limelight measurement.
     * At competition range ta is typically 0.2–2%; set to 0 to disable.
     */
    public static double minTargetAreaPercent = 0.1;

    /** Estimated Limelight pipeline latency in milliseconds for timestamp compensation. */
    public static double limelightLatencyMs = 20.0;

    /**
     * Post-relocalize xy covariance (inches). Lower = filter trusts the
     * manual pose more, future vision corrections pull less aggressively.
     */
    public static double relocalizeCovarianceXY = 0.5;

    /**
     * Post-relocalize heading covariance (degrees, converted to radians at use).
     * Lower = filter trusts the manual heading more.
     */
    public static double relocalizeCovarianceHeadingDeg = 2.0;
}
