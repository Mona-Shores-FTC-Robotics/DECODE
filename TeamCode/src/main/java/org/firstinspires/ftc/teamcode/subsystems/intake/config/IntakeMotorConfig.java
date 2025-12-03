package org.firstinspires.ftc.teamcode.subsystems.intake.config;

/**
 * Intake motor configuration.
 * Controls motor power, direction, and behavior.
 */
public class IntakeMotorConfig {
    public String motorName = "intake";
    public double defaultForwardPower = -1;
    public double defaultReversePower = .75; //this was .3, if we have problems maybe go back
    public boolean brakeOnZero = true;
    public boolean reverseDirection = true;
}
