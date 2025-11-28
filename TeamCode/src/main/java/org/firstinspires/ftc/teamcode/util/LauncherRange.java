package org.firstinspires.ftc.teamcode.util;

/**
 * Represents the shooting range configurations for the launcher.
 * Each range corresponds to a different set of flywheel RPM targets.
 */
public enum LauncherRange {
    /** Short-range shot configuration (~2700 RPM) */
    SHORT("Short"),

    /** Mid-range shot configuration (~3600 RPM) */
    MID("Mid"),

    /** Long-range shot configuration (~4200 RPM) */
    LONG("Long"),

    /** Short autonomous shot configuration (tuned separately from SHORT for auto) */
    SHORT_AUTO("Short Auto");

    private final String displayName;

    LauncherRange(String displayName) {
        this.displayName = displayName;
    }

    public String getDisplayName() {
        return displayName;
    }

    @Override
    public String toString() {
        return displayName;
    }
}
