package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;

/**
 * Central place for the ExampleGateNipAuto field coordinates. Values are in inches with
 * headings in degrees for easy dashboard tuning.
 */
@Config
public class Waypoints {

    public static double poseLaunchFarX = 48.0;
    public static double poseLaunchFarY = 12.0;
    public static double poseLaunchFarHeadingDeg = 0.0;

    public static double A_entryX = 32.0;
    public static double A_entryY = 16.0;
    public static double A_entryHeadingDeg = 15.0;
    public static double A_exitX = 60.0;
    public static double A_exitY = 18.0;
    public static double A_exitHeadingDeg = -6.0;

    public static double B_entryX = 34.0;
    public static double B_entryY = 32.0;
    public static double B_entryHeadingDeg = 10.0;
    public static double B_exitX = 58.0;
    public static double B_exitY = 34.0;
    public static double B_exitHeadingDeg = -10.0;

    public static double C_entryX = 36.0;
    public static double C_entryY = 50.0;
    public static double C_entryHeadingDeg = 8.0;
    public static double C_exitX = 60.0;
    public static double C_exitY = 52.0;
    public static double C_exitHeadingDeg = -8.0;

    public static double D_entryX = 38.0;
    public static double D_entryY = 68.0;
    public static double D_entryHeadingDeg = 5.0;
    public static double D_exitX = 62.0;
    public static double D_exitY = 70.0;
    public static double D_exitHeadingDeg = -5.0;

    public static double gateEntryX = 66.0;
    public static double gateEntryY = 36.0;
    public static double gateEntryHeadingDeg = 0.0;
    public static double gateExitX = 76.0;
    public static double gateExitY = 36.0;
    public static double gateExitHeadingDeg = 0.0;

    public static double posTolIn = 1.25;
    public static double headTolDeg = 4.0;
    public static double velTolMps = 0.25;

    private static final double FIELD_WIDTH_IN = 144.0;

    private Waypoints() { }

    public static FieldLayout mirrorForAlliance(boolean isBlue) {
        if (!isBlue) {
            return buildFieldLayout(false);
        }
        return buildFieldLayout(true);
    }

    private static FieldLayout buildFieldLayout(boolean mirrored) {
        Pose launchFar = pose(poseLaunchFarX, poseLaunchFarY, poseLaunchFarHeadingDeg, mirrored);
        return new FieldLayout(
                launchFar,
                pose(A_entryX, A_entryY, A_entryHeadingDeg, mirrored),
                pose(A_exitX, A_exitY, A_exitHeadingDeg, mirrored),
                pose(B_entryX, B_entryY, B_entryHeadingDeg, mirrored),
                pose(B_exitX, B_exitY, B_exitHeadingDeg, mirrored),
                pose(C_entryX, C_entryY, C_entryHeadingDeg, mirrored),
                pose(C_exitX, C_exitY, C_exitHeadingDeg, mirrored),
                pose(D_entryX, D_entryY, D_entryHeadingDeg, mirrored),
                pose(D_exitX, D_exitY, D_exitHeadingDeg, mirrored),
                pose(gateEntryX, gateEntryY, gateEntryHeadingDeg, mirrored),
                pose(gateExitX, gateExitY, gateExitHeadingDeg, mirrored),
                new PathConstraints(0.99, 100, 1, 1)
        );
    }

    private static Pose pose(double x, double y, double headingDeg, boolean mirrored) {
        double headingRad = Math.toRadians(headingDeg);
        if (!mirrored) {
            return new Pose(x, y, headingRad);
        }

        double mirroredX = FIELD_WIDTH_IN - x;
        double mirroredHeading = normalizeAngle(Math.PI - headingRad);
        return new Pose(mirroredX, y, mirroredHeading);
    }

    private static double normalizeAngle(double angle) {
        while (angle <= -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        return angle;
    }

    public static class FieldLayout {
        public final Pose poseLaunchFar;
        public final Pose A_entry;
        public final Pose A_exit;
        public final Pose B_entry;
        public final Pose B_exit;
        public final Pose C_entry;
        public final Pose C_exit;
        public final Pose D_entry;
        public final Pose D_exit;
        public final Pose gateEntry;
        public final Pose gateExit;
        public final PathConstraints baseConstraints;

        FieldLayout(Pose poseLaunchFar,
                    Pose aEntry, Pose aExit,
                    Pose bEntry, Pose bExit,
                    Pose cEntry, Pose cExit,
                    Pose dEntry, Pose dExit,
                    Pose gateEntry, Pose gateExit,
                    PathConstraints baseConstraints) {
            this.poseLaunchFar = poseLaunchFar;
            this.A_entry = aEntry;
            this.A_exit = aExit;
            this.B_entry = bEntry;
            this.B_exit = bExit;
            this.C_entry = cEntry;
            this.C_exit = cExit;
            this.D_entry = dEntry;
            this.D_exit = dExit;
            this.gateEntry = gateEntry;
            this.gateExit = gateExit;
            this.baseConstraints = baseConstraints;
        }
    }
}
