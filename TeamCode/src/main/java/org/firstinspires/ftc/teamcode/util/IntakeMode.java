package org.firstinspires.ftc.teamcode.util;

/**
 * What the intake is doing right now. Referenced from gamepad bindings,
 * autonomous command builders, and the intake subsystem itself.
 */
public enum IntakeMode {
    /** Gently push artifacts back out — used while waiting between intake passes. */
    PASSIVE_REVERSE,
    /** Run reverse at full power — used to eject jammed artifacts. */
    AGGRESSIVE_REVERSE,
    /** Pull artifacts in. */
    ACTIVE_FORWARD,
    /** Cut motor power. */
    STOPPED
}
