package org.firstinspires.ftc.teamcode.commands.LauncherCommands.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LauncherIdleConfig {
    /** RPM held on all lanes when no artifacts are loaded in any lane. */
    public double lowIdleRpm = 600.0;

    /**
     * How long (ms) to stay spun up without lanes clearing before giving up and returning
     * to low idle. Guards against jammed artifacts (lane sensor stays lit indefinitely)
     * and stuck color sensors (always reporting an artifact). Operator can re-trigger
     * by holding the aim button manually.
     */
    public double spinUpTimeoutMs = 20000.0;
}
