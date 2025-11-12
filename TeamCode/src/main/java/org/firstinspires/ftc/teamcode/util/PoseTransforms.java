package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Helpers for converting between Pedro field coordinates (origin at the blue driver wall with X
 * pointing up-field) and the FTC field coordinate system (origin at field centre with X pointing
 * toward the red wall and Y toward the far driver stations).
 */
public final class PoseTransforms {

    private PoseTransforms() {}

    public static Pose toFtcPose(Pose pedroPose) {
        if (pedroPose == null) {
            return null;
        }
        double halfField = AutoField.Waypoints.fieldWidthIn / 2.0;
        double ftcX = halfField - pedroPose.getY();
        double ftcY = pedroPose.getX() - halfField;
        double heading = AngleUnit.normalizeRadians(pedroPose.getHeading() + Math.PI / 2.0);
        return new Pose(ftcX, ftcY, heading);
    }

    public static Pose toPedroPose(Pose ftcPose) {
        if (ftcPose == null) {
            return null;
        }
        double halfField = AutoField.Waypoints.fieldWidthIn / 2.0;
        double pedroX = ftcPose.getY() + halfField;
        double pedroY = halfField - ftcPose.getX();
        double heading = AngleUnit.normalizeRadians(ftcPose.getHeading() - Math.PI / 2.0);
        return new Pose(pedroX, pedroY, heading);
    }
}
