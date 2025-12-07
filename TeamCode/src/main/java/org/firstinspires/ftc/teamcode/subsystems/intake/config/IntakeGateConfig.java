package org.firstinspires.ftc.teamcode.subsystems.intake.config;

/**
 * Intake gate servo configuration.
 * Controls the gate that prevents/allows artifacts to flow into launcher.
 * Contains base configuration and robot-specific instances.
 */
public class IntakeGateConfig {
    public String servoName = "gate";
    public double preventArtifacts;
    public double allowArtifacts;
    public double reverseConfig;

    // Robot-specific instances
    public static IntakeGateConfig gateConfig19429 = createGateConfig19429();
    public static IntakeGateConfig gateConfig20245 = createGateConfig20245();

    /**
     * Creates gate configuration for robot 19429.
     */
    private static IntakeGateConfig createGateConfig19429() {
        IntakeGateConfig config = new IntakeGateConfig();
        config.preventArtifacts = .3;
        config.allowArtifacts = 0;
        config.reverseConfig = .8;
        return config;
    }

    /**
     * Creates gate configuration for robot 20245.
     */
    private static IntakeGateConfig createGateConfig20245() {
        IntakeGateConfig config = new IntakeGateConfig();
        config.preventArtifacts = .3;
        config.allowArtifacts = 0;
        config.reverseConfig = .8;
        return config;
    }
}
