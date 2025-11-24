package org.firstinspires.ftc.teamcode.subsystems.intake.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Intake roller servo configuration.
 * Controls the roller servo positions for forward, reverse, and inactive states.
 */
@Configurable
public class IntakeRollerConfig {
    public String servoName = "intake_roller";
    public double forward = 0;
    public double inactivePosition = 0.5;
    public double reverse = 1;
}
