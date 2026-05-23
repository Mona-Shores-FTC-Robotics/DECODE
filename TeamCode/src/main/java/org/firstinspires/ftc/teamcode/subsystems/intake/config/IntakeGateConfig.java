package org.firstinspires.ftc.teamcode.subsystems.intake.config;

/**
 * Intake gate servo configuration. Currently identical for both robots —
 * see {@code util/RobotProfile.java} if you ever need to diverge.
 */
public class IntakeGateConfig {
    /** Hardware name for gate servo */
    public String servoName = "gate";

    /** Servo position to prevent artifacts from entering launcher */
    public double preventArtifacts = 0.3;

    /** Servo position to allow artifacts to flow into launcher */
    public double allowArtifacts = 0.0;

    /** Servo position for reverse/ejecting mode */
    public double reverseConfig = .6;
}
