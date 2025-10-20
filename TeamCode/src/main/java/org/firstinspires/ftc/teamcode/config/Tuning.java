package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Dashboard-tunable values that affect FieldCentricTeleOp behaviour.
 */
@Config
public final class Tuning {
    private Tuning() {}

    public static double SLOW_MULTIPLIER = 0.35;
    public static double AUTO_HEADING_KP = 2.25;
    public static double FLYWHEEL_RPM_LOW = 3200.0;
    public static double FLYWHEEL_RPM_HIGH = 4200.0;
    public static double MAX_LINEAR_SPEED_IPS = 60.0;
    public static double MAX_ANGULAR_SPEED_DEG_PER_SEC = 360.0;
    public static double LINEAR_RAMP_TIME_SEC = 0.25;
    public static double ANGULAR_RAMP_TIME_SEC = 0.25;
    public static double PINPOINT_LINEAR_KP = 0.8;
}
