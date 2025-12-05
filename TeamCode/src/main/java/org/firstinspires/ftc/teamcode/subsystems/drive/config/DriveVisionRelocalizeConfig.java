package org.firstinspires.ftc.teamcode.subsystems.drive.config;

/**
 * Vision relocalization agreement thresholds (MT1 vs MT2).
 * Kept separate from aim assist so tuning stays discoverable.
 */
public class DriveVisionRelocalizeConfig {
    /** Distance threshold for MT1 vs MT2 agreement (inches) */
    public double MT2DistanceThreshold = 12.0;
    /** Heading agreement threshold between MT1/MT2 (degrees) */
    public double MT2HeadingThresholdDeg = 15.0;
}
