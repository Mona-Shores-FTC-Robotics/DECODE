package org.firstinspires.ftc.teamcode.subsystems.intake.config;

import static org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.IntakeMode.STOPPED;

/**
 * Manual mode override configuration.
 * Allows manual control of intake mode via FTC Dashboard.
 */
public class IntakeManualModeConfig {
    public boolean enableOverride = false;
    public String overrideMode = STOPPED.name();
}
