package org.firstinspires.ftc.teamcode.subsystems.drive.config;

/**
 * Vision relocalization agreement thresholds (MT1 vs MT2) and jump safeguards.
 * Kept separate from aim assist so tuning stays discoverable.
 */
public class DriveVisionRelocalizeConfig {
    /** Distance threshold for MT1 vs MT2 agreement (inches) */
    public double MT2DistanceThreshold = 12.0;
    /** Heading agreement threshold between MT1/MT2 (degrees) */
    public double MT2HeadingThresholdDeg = 15.0;

    // === Jump Safeguards ===
    // These prevent vision from making large, likely-erroneous pose corrections.
    // Compare proposed vision pose against a reference pose (expected start or current odometry).

    /** Maximum allowed XY jump distance from reference pose (inches).
     *  Vision poses further than this from reference will be rejected. */
    public double maxJumpDistanceInches = 18.0;

    /** Maximum allowed heading jump from reference pose (degrees).
     *  Vision poses with heading differing more than this from reference will be rejected. */
    public double maxJumpHeadingDeg = 20.0;
}
