package org.firstinspires.ftc.teamcode.subsystems.drive.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Initial pose configuration for autonomous start position.
 * Defines the robot's starting position and heading on the field.
 */
@Configurable
public class DriveInitialPoseConfig {
    /** Starting X position in Pedro coordinates (inches) */
    public double startX = 56;
    /** Starting Y position in Pedro coordinates (inches) */
    public double startY = 8;
    /** Starting heading in degrees */
    public double startHeadingDeg = 90;
}
