package org.firstinspires.ftc.teamcode.util;

/**
 * Launcher operating mode that determines firing behavior.
 *
 * THROUGHPUT mode: Fire all lanes rapidly for maximum scoring rate
 * DECODE mode: Fire in detected obelisk pattern sequence with spacing for precise scoring
 */
public enum LauncherMode {
    /**
     * Throughput mode - rapid firing of all lanes without pattern consideration.
     * Used during early match when speed matters more than pattern accuracy.
     */
    THROUGHPUT,

    /**
     * Decode mode - fires in obelisk pattern sequence with motif tail offset.
     * Used during endgame (last 30 seconds) for precise obelisk scoring.
     */
    DECODE
}
