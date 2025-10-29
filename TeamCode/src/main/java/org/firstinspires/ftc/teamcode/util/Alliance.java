package org.firstinspires.ftc.teamcode.util;

/**
 * Represents the alliance a routine should run for.
 */
public enum Alliance {
    BLUE,
    RED,
    UNKNOWN;

    public String displayName() {
        switch (this) {
            case BLUE:
                return "Blue";
            case RED:
                return "Red";
            case UNKNOWN:
            default:
                return "Unknown";
        }
    }
}
