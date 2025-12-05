package org.firstinspires.ftc.teamcode.subsystems.lighting.config;

/**
 * Lighting indicator servo configuration.
 * Controls servo names for left, center, and right lane indicators.
 */
public class LightingIndicatorConfig {
    public LeftIndicatorConfig left = new LeftIndicatorConfig();
    public CenterIndicatorConfig center = new CenterIndicatorConfig();
    public RightIndicatorConfig right = new RightIndicatorConfig();

    public static class LeftIndicatorConfig {
        public String servoName = "indicator_left";
    }

    public static class CenterIndicatorConfig {
        public String servoName = "indicator_center";
    }

    public static class RightIndicatorConfig {
        public String servoName = "indicator_right";
    }
}
