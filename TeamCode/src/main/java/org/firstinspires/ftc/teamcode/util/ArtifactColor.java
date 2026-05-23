package org.firstinspires.ftc.teamcode.util;

/**
 * Color of an indexed artifact, as seen by the intake lane sensors.
 * Used by both the intake classification logic and the lighting subsystem.
 */
public enum ArtifactColor {
    /** Sensor read with no artifact present (or sensor unreachable). */
    NONE,
    /** Green artifact detected. */
    GREEN,
    /** Purple artifact detected. */
    PURPLE,
    /** Something is there but classification didn't pick GREEN or PURPLE.
     *  Also overloaded by LightingSubsystem to mean "blink white" on the indicator strip. */
    UNKNOWN;

    /** True for GREEN or PURPLE — i.e. we have a confident artifact color. */
    public boolean isKnown() {
        return this == GREEN || this == PURPLE;
    }

    /** True if this represents an actual artifact (GREEN or PURPLE). */
    public boolean isArtifact() {
        return this == GREEN || this == PURPLE;
    }

    /** True if no artifact is confidently present (NONE or UNKNOWN). */
    public boolean isAbsent() {
        return !isArtifact();
    }
}
