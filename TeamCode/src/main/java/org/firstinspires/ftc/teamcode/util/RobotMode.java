package org.firstinspires.ftc.teamcode.util;

/**
 * Identifies whether the robot should behave with match-ready automation or in
 * developer-focused debug mode. Subsystems can inspect the active mode to
 * enable or disable background behaviours such as automatic spin-up,
 * continuous sensor polling, or hardware safety interlocks.
 */
public enum RobotMode {
    MATCH,
    DEBUG;

    public static RobotMode orDefault(RobotMode mode) {
        return mode == null ? DEBUG : mode;
    }
}
