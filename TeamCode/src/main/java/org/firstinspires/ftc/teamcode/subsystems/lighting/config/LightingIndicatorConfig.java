package org.firstinspires.ftc.teamcode.subsystems.lighting.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Lighting indicator servo configuration.
 * Controls servo names for left, center, and right lane indicators.
 */
@Configurable
public class LightingIndicatorConfig {
    public LeftIndicatorConfig left = new LeftIndicatorConfig();
    public CenterIndicatorConfig center = new CenterIndicatorConfig();
    public RightIndicatorConfig right = new RightIndicatorConfig();

    @Configurable
    public static class LeftIndicatorConfig {
        public String servoName = "indicator_left";
    }

    @Configurable
    public static class CenterIndicatorConfig {
        public String servoName = "indicator_center";
    }

    @Configurable
    public static class RightIndicatorConfig {
        public String servoName = "indicator_right";
    }
}
