package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Geometry for drawing the robot at its true <em>frame</em> center in telemetry /
 * AdvantageScope.
 *
 * <p>Pedro tracks the drivetrain/odometry center, which usually isn't the frame's
 * geometric center — so the AS robot box would sit offset from the real body and
 * misrepresent where the robot is. These offsets shift ONLY the published
 * {@code Pose/Robot} visualization to the frame center. They do <b>not</b> affect
 * control, path following, relocalization, or the odometry pose.
 *
 * <p>Single shared global (live-tunable in Panels). Measured on the 19429/20245
 * 17.5"×17.5" chassis: drivetrain center sits 5.75" from the back, frame center at
 * 8.75" → frame center is 3.0" forward of the drivetrain center, centered laterally.
 * If the two robots' drivetrain placement ever differs, tweak this live.
 */
@Configurable
public class RobotFrameConfig {
    /** Forward (+toward front) distance from the drivetrain center to the frame center, inches. */
    public static double frameCenterForwardIn = 3.0;

    /** Left (+) distance from the drivetrain center to the frame center, inches. */
    public static double frameCenterLeftIn = 0.0;
}
