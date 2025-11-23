package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Voltage compensation configuration for consistent launcher performance.
 * Adjusts motor power based on battery voltage to maintain consistent RPM.
 */
@Configurable
public class LauncherVoltageCompensationConfig {
    /** Enable battery voltage compensation for consistent motor performance */
    public boolean enabled = true;
    /** Nominal battery voltage for calibration (typically 12.5V for full charge) */
    public double nominalVoltage = 12.5;
    /** Minimum voltage threshold - below this, compensation is clamped for safety */
    public double minVoltage = 9.0;
}
