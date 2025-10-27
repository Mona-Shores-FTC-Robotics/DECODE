package org.firstinspires.ftc.teamcode.util;

/**
 * Shared enumeration describing the three launcher lanes from left to right.
 * Used by the shooter, intake, and lighting subsystems to agree on ordering.
 */
public enum LauncherLane {
    LEFT,
    CENTER,
    RIGHT;

    public static LauncherLane fromIndex(int index) {
        LauncherLane[] lanes = values();
        if (index < 0) {
            index = 0;
        }
        if (index >= lanes.length) {
            index = lanes.length - 1;
        }
        return lanes[index];
    }
}
