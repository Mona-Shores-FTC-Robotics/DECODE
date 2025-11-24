package org.firstinspires.ftc.teamcode.commands.generated;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

/**
 * AUTO-GENERATED configuration for TrajectoryAuto
 *
 * Waypoints and path settings are Dashboard-tunable.
 * Regenerate this file when you change the path in Pedro GUI.
 */
@Configurable
public class TrajectoryAutoConfig {

    // ===== WAYPOINTS (Dashboard tunable) =====
    @Configurable
    public static class Waypoints {

        @Configurable
        public static class Start {
            public double x = 56.0;
            public double y = 8.0;
            public double headingDeg = 90.0;

            public Pose toPose() {
                return new Pose(x, y, Math.toRadians(headingDeg));
            }
        }
        public static Start start = new Start();

        @Configurable
        public static class Path1 {
            public double x = 56.279;
            public double y = 19.817;
            public double headingDeg = 90.0;

            public Pose toPose() {
                return new Pose(x, y, Math.toRadians(headingDeg));
            }
        }
        public static Path1 path1 = new Path1();

        @Configurable
        public static class Path2 {
            public double x = 23.780;
            public double y = 23.780;
            public double headingDeg = 90.0;

            public Pose toPose() {
                return new Pose(x, y, Math.toRadians(headingDeg));
            }
        }
        public static Path2 path2 = new Path2();

        @Configurable
        public static class Path3 {
            public double x = 23.516;
            public double y = 39.633;
            public double headingDeg = 0.0;  // tangential - no fixed heading

            public Pose toPose() {
                return new Pose(x, y, Math.toRadians(headingDeg));
            }
        }
        public static Path3 path3 = new Path3();

        @Configurable
        public static class Path4 {
            public double x = 56.279;
            public double y = 19.552;
            public double headingDeg = 109.0;

            public Pose toPose() {
                return new Pose(x, y, Math.toRadians(headingDeg));
            }
        }
        public static Path4 path4 = new Path4();
    }

    // ===== PATH POWER SETTINGS (Dashboard tunable) =====
    @Configurable
    public static class PathPower {
        public double path1 = 0.8;
        public double path2 = 0.8;
        public double path3 = 0.8;
        public double path4 = 0.8;
    }
    public static PathPower pathPower = new PathPower();
}
