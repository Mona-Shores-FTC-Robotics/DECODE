package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Feeder (kicker) configuration for launcher servos.
 * Contains base configuration and robot-specific instances.
 */
@Configurable
public class LauncherFeederConfig {
    public LeftFeederConfig left = new LeftFeederConfig();
    public CenterFeederConfig center = new CenterFeederConfig();
    public RightFeederConfig right = new RightFeederConfig();

    @Configurable
    public static class CenterFeederConfig {
        public String servoName = "feeder_center";
        public boolean reversed = false;
        public double holdMs = 1000;

        public double loadPosition;
        public double pinchPosition;
        public double firePosition;
    }


    @Configurable
    public static class LeftFeederConfig {
        public String servoName = "feeder_left";
        public boolean reversed = false;
        public double holdMs = 1000;

        public double loadPosition;
        public double pinchPosition;
        public double firePosition;
    }

    @Configurable
    public static class RightFeederConfig {
        public String servoName = "feeder_right";
        public boolean reversed = false;
        public double holdMs = 1000;

        public double loadPosition;
        public double pinchPosition;
        public double firePosition;
    }

    // Robot-specific instances
    public static LauncherFeederConfig feederConfig19429 = createFeederConfig19429();
    public static LauncherFeederConfig feederConfig20245 = createFeederConfig20245();

    /**
     * Creates feeder configuration for robot 19429.
     */
    private static LauncherFeederConfig createFeederConfig19429() {
        LauncherFeederConfig config = new LauncherFeederConfig();
        // Apply 19429-specific values
        config.center.loadPosition = .93;
        config.center.pinchPosition = .87;
        config.center.firePosition = .75;

        config.left.loadPosition = .8;
        config.left.pinchPosition = .74;
        config.left.firePosition = .61;

        config.right.loadPosition = .75;
        config.right.pinchPosition = .69;
        config.right.firePosition = .56;

        return config;
    }

    private static LauncherFeederConfig createFeederConfig20245() {
        LauncherFeederConfig config = new LauncherFeederConfig();
        // Apply 20245-specific values
        config.center.loadPosition = .18;
        config.center.pinchPosition = .12;
        config.center.firePosition = .03;

        config.right.loadPosition = .78;
        config.right.pinchPosition = .72;
        config.right.firePosition = .58;

        config.left.loadPosition = .33;
        config.left.pinchPosition = .27;
        config.left.firePosition = .18;
        return config;
    }
}
