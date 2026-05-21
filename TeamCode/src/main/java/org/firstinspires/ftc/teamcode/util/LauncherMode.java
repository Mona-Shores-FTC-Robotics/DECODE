package org.firstinspires.ftc.teamcode.util;

/**
 * Controls how the launcher decides which lanes to fire and in what order.
 *
 * Switch between modes with the operator Back button (auto-switches at ~50 s remaining).
 */
public enum LauncherMode {
    /**
     * Fire all three lanes as fast as possible — no pattern required.
     * Best for early match when filling obelisks quickly matters more than color order.
     */
    THROUGHPUT,

    /**
     * Fire lanes in the motif color sequence (detected by vision from AprilTags 21–23).
     * Each artifact lands in the correct left/center/right obelisk to score the bonus.
     * Switch to this automatically at ~50 seconds remaining, or manually with Back button.
     */
    DECODE
}
