package org.firstinspires.ftc.teamcode.util;

/**
 * Represents the colour of an indexed artifact for lighting and automation.
 */
public enum ArtifactColor {
    /** No signal / sensor error / out of range */
    NONE,
    /** Green artifact detected */
    GREEN,
    /** Purple artifact detected */
    PURPLE,
    /** Unknown artifact color (for range-based classifier gap zones) */
    UNKNOWN,
    /** Background/empty space detected (sensor working, no artifact present) */
    BACKGROUND;

    public boolean isKnown() {
        return this != NONE && this != UNKNOWN && this != BACKGROUND;
    }

    /**
     * Returns true if this represents an actual artifact (GREEN or PURPLE).
     */
    public boolean isArtifact() {
        return this == GREEN || this == PURPLE;
    }

    /**
     * Returns true if no artifact is present (NONE, BACKGROUND, or UNKNOWN).
     */
    public boolean isAbsent() {
        return !isArtifact();
    }
}
