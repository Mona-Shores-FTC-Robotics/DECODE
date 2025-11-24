package org.firstinspires.ftc.teamcode.subsystems.intake.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Intake gate servo configuration.
 * Controls the gate that prevents/allows artifacts to flow into launcher.
 * Contains base configuration and robot-specific instances.
 */
@Configurable
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
        config.preventArtifacts = .5;
        config.allowArtifacts = .1;
        config.reverseConfig = .8;
        return config;
    }

    /**
     * Creates gate configuration for robot 20245.
     */
    private static IntakeGateConfig createGateConfig20245() {
        IntakeGateConfig config = new IntakeGateConfig();
        config.preventArtifacts = .7;
        config.allowArtifacts = .3;
        config.reverseConfig = 1.0;
        return config;
    }
}
