package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LauncherMode;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.Objects;

/**
 * Mode-aware launch command. Picks the underlying launch behavior at execution time
 * (not at binding setup) based on {@link RobotState#getLauncherMode()}.
 *
 * Uses {@link Commands#lazy(java.util.function.Supplier)} so the mode check defers
 * until the scheduler starts this command.
 */
public final class ModeAwareLaunchCommand {

    private ModeAwareLaunchCommand() {}

    public static Command create(LauncherSubsystem launcher,
                                  IntakeSubsystem intake,
                                  boolean spinDownAfterShot) {
        Objects.requireNonNull(launcher, "launcher required");
        Objects.requireNonNull(intake, "intake required");
        return Commands.lazy(() -> {
            LauncherMode currentMode = RobotState.getLauncherMode();
            if (currentMode == LauncherMode.DECODE) {
                return LaunchInSequenceCommand.create(launcher, intake, spinDownAfterShot);
            }
            return LaunchAllCommand.create(launcher, intake, spinDownAfterShot);
        });
    }
}
