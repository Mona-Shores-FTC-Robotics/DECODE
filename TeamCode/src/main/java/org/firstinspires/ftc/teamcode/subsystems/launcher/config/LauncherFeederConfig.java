package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

/**
 * Feeder (kicker) configuration for launcher servos.
 *
 * Configuration Pattern:
 * - Values with defaults are SHARED across both robots (same hardware/behavior)
 * - Values without defaults are ROBOT-SPECIFIC (tune per robot in Dashboard)
 * - If you edit a value with a default here and nothing changes, check if it's
 *   being overridden in the robot-specific create methods below
 *
 * This config CORRECTLY follows best practices:
 * - Hardware names and timing have defaults (shared)
 * - Servo positions have NO defaults (robot-specific - vary significantly)
 */
public class LauncherFeederConfig {
    public LeftFeederConfig left = new LeftFeederConfig();
    public CenterFeederConfig center = new CenterFeederConfig();
    public RightFeederConfig right = new RightFeederConfig();

    public static class CenterFeederConfig {
        // SHARED - hardware name same for both robots
        public String servoName = "feeder_center";

        // SHARED - servo direction same for both robots (based on mounting)
        public boolean reversed = false;

        // SHARED - hold time same for both robots
        public double holdMs = 1000;

        // ROBOT-SPECIFIC - servo positions differ significantly between robots
        /** Servo position for loading artifact into feeder */
        public double loadPosition;

        /** Servo position for pinching artifact (pre-fire) */
        public double pinchPosition;

        /** Servo position for firing artifact */
        public double firePosition;
    }


    public static class LeftFeederConfig {
        // SHARED - hardware name same for both robots
        public String servoName = "feeder_left";

        // SHARED - servo direction same for both robots (based on mounting)
        public boolean reversed = false;

        // SHARED - hold time same for both robots
        public double holdMs = 1000;

        // ROBOT-SPECIFIC - servo positions differ significantly between robots
        /** Servo position for loading artifact into feeder */
        public double loadPosition;

        /** Servo position for pinching artifact (pre-fire) */
        public double pinchPosition;

        /** Servo position for firing artifact */
        public double firePosition;
    }

    public static class RightFeederConfig {
        // SHARED - hardware name same for both robots
        public String servoName = "feeder_right";

        // SHARED - servo direction same for both robots (based on mounting)
        public boolean reversed = false;

        // SHARED - hold time same for both robots
        public double holdMs = 1000;

        // ROBOT-SPECIFIC - servo positions differ significantly between robots
        /** Servo position for loading artifact into feeder */
        public double loadPosition;

        /** Servo position for pinching artifact (pre-fire) */
        public double pinchPosition;

        /** Servo position for firing artifact */
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
