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
}
