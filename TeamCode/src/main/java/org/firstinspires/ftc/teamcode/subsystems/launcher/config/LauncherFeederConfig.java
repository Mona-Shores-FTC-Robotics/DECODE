package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Feeder (bootkicker) configuration for launcher servos.
 * Contains base configuration and robot-specific instances.
 */
@Configurable
public class LauncherFeederConfig {
    public LeftFeederConfig left = new LeftFeederConfig();
    public CenterFeederConfig center = new CenterFeederConfig();
    public RightFeederConfig right = new RightFeederConfig();

    @Configurable
    public static class LeftFeederConfig {
        public String servoName = "feeder_left";
        public boolean reversed = false;
        public double loadPosition = .8;
        public double firePosition = .61; //toward 0 moves toward fire position
        public double holdMs = 1000;
    }

    @Configurable
    public static class CenterFeederConfig {
        public String servoName = "feeder_center";
        public boolean reversed = false;
        public double loadPosition = .93;
        public double firePosition = .75; //toward 0 moves toward fire position
        public double holdMs = 1000;
    }

    @Configurable
    public static class RightFeederConfig {
        public String servoName = "feeder_right";
        public boolean reversed = false;
        public double loadPosition = .75;
        public double firePosition = .56; //toward 0 moves toward fire position
        public double holdMs = 1000;
    }

    // Robot-specific instances
    public static LauncherFeederConfig feederConfig19429 = createFeederConfig19429();
    public static LauncherFeederConfig feederConfig20245 = createFeederConfig20245();

    /**
     * Creates feeder configuration for robot 19429.
     */
    private static LauncherFeederConfig createFeederConfig19429() {
        return new LauncherFeederConfig(); // Uses default values
    }

    /**
     * Creates feeder configuration for robot 20245.
     */
    private static LauncherFeederConfig createFeederConfig20245() {
        LauncherFeederConfig config = new LauncherFeederConfig();
        // Apply 20245-specific values
        config.center.loadPosition = .18;
        config.center.firePosition = .03;

        config.right.loadPosition = .78;
        config.right.firePosition = .58;

        config.left.loadPosition = .33;
        config.left.firePosition = .18;
        return config;
    }
}
