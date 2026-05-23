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
     * Master escape hatch for ALL vision→pose correction. When {@code false},
     * the robot runs on pure Pinpoint odometry from its configured/seeded start
     * pose: the continuous Kalman vision feed, every vision relocalization
     * (init, shot, manual), and the auto init start-pose-from-vision override
     * are all skipped. Flip this in Panels to test whether vision fusion is the
     * cause of a pose problem. Live-tunable — takes effect next loop.
     */
    public static boolean fusionVisionEnabled = true;

    /**
     * When {@code false}, the robot applies NO vision pose correction during auto
     * init — it holds the configured/seeded start pose while you place the robot,
     * and only begins fusing vision once the OpMode starts. Prevents a vision read
     * taken while you're still moving the robot into position from shifting the
     * start pose. Init still detects alliance + motif and still displays the vision
     * pose for sanity-checking; it just doesn't apply it to the follower.
     */
    public static boolean relocalizeDuringInit = false;

    /**
     * Drop vision measurements older than this (ms) in the continuous fusion feed.
     * Stops a stale snapshot — e.g. one captured during init while repositioning —
     * from being replayed into the filter on the first auto loop.
     */
    public static double maxMeasurementAgeMs = 200.0;

    /**
     * Reject a continuous-fusion vision measurement that disagrees with the
     * current fused pose by more than this (inches). The feed had no jump check,
     * so a bad single-tag solution (common from a far/opposite goal tag) could
     * drag the estimate off the map. Real corrections are far smaller than this;
     * genuine large drift gets corrected by the (guarded) relocalize paths.
     */
    public static double maxFusionInnovationIn = 24.0;

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
