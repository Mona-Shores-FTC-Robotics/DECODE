package org.firstinspires.ftc.teamcode.commands.DriveCommands.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Configuration for CaptureAndAimCommand.
 * Controls vision sampling and timing for target acquisition.
 */
@Configurable
public class CaptureAndAimConfig {
    /**
     * Number of frames to average for robustness (1 = no averaging).
     * Averaging multiple vision samples helps filter out single-frame noise.
     */
    public int sampleFrames = 3;

    /**
     * Time between frame samples (milliseconds).
     * Spreading samples over time improves robustness to transient vision errors.
     */
    public double frameSampleIntervalMs = 50.0;

    /**
     * Maximum time for sampling phase before giving up (milliseconds).
     */
    public double samplingTimeoutMs = 500.0;

    /**
     * Heading tolerance in degrees - if already within this tolerance, skip the turn.
     * This prevents Pedro turnTo() from getting stuck when heading error is tiny.
     */
    public double headingToleranceDeg = 3.0;
}
