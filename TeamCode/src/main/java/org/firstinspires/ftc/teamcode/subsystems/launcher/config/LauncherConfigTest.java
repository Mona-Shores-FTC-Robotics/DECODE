package org.firstinspires.ftc.teamcode.subsystems.launcher.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Test config to verify @Configurable pattern works.
 *
 * KEY PATTERN: The class has @Configurable, and we use a SINGLE public static instance.
 * FTC Dashboard will modify THIS instance's fields, and the code reads from THIS instance.
 */
@Configurable
public class LauncherConfigTest {

    /** Test value - change in Dashboard and verify robot sees it */
    public double testRpm = 1000.0;

    /** Another test value */
    public double testKp = 0.5;

    /** Test boolean */
    public boolean testEnabled = true;
}
