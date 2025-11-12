package org.firstinspires.ftc.teamcode.util;

/**
 * Represents the colour of an indexed artifact for lighting and automation.
 */
public enum ArtifactColor {
    NONE,
    GREEN,
    PURPLE,
    UNKNOWN;

    public boolean isKnown() {
        return this != NONE && this != UNKNOWN;
    }
}
