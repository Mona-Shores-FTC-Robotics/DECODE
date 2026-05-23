package org.firstinspires.ftc.teamcode.commands.LauncherCommands.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LauncherIdleConfig {
    /** RPM held on all lanes when no artifacts are loaded in any lane. */
    public double lowIdleRpm = 600.0;
}
