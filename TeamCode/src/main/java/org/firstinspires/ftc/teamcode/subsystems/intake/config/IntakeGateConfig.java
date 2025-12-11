package org.firstinspires.ftc.teamcode.subsystems.intake.config;

/**
 * Intake gate servo configuration.
 * Controls the gate that prevents/allows artifacts to flow into launcher.
 *
 * Configuration Pattern:
 * - Values with defaults are SHARED across both robots (same hardware/behavior)
 * - Values without defaults are ROBOT-SPECIFIC (tune per robot in Dashboard)
 * - If you edit a value with a default here and nothing changes, check if it's
 *   being overridden in the robot-specific create methods below
 */
public class IntakeGateConfig {
    // ===== SHARED VALUES (same for both robots) =====
    /** Hardware name for gate servo */
    public String servoName = "gate";

    /** Servo position to prevent artifacts from entering launcher */
    public double preventArtifacts = 0.3;

    /** Servo position to allow artifacts to flow into launcher */
    public double allowArtifacts = 0.0;

    /** Servo position for reverse/ejecting mode */
    public double reverseConfig = .6;

    // ===== ROBOT-SPECIFIC VALUES (no defaults - set per robot) =====
    // (none currently - all values are shared)

    // Robot-specific instances
    public static IntakeGateConfig gateConfig19429 = createGateConfig19429();
    public static IntakeGateConfig gateConfig20245 = createGateConfig20245();

    /**
     * Creates gate configuration for robot 19429.
     * Currently uses all default values - both robots have identical gate tuning.
     */
    private static IntakeGateConfig createGateConfig19429() {
        IntakeGateConfig config = new IntakeGateConfig();
        // No overrides needed - using shared defaults
        return config;
    }

    /**
     * Creates gate configuration for robot 20245.
     * Currently uses all default values - both robots have identical gate tuning.
     */
    private static IntakeGateConfig createGateConfig20245() {
        IntakeGateConfig config = new IntakeGateConfig();
        // No overrides needed - using shared defaults
        return config;
    }
}
