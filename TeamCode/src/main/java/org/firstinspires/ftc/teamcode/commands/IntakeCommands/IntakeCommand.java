package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Base class for intake-oriented commands.
 * Handles mode management and intake requirements so derived commands only
 * need to define their specific run conditions.
 */
public abstract class IntakeCommand extends Command {

    private final IntakeSubsystem intake;

    protected IntakeCommand(IntakeSubsystem intake, Subsystem... additionalRequirements) {
        this.intake = intake;
        requires(intake);
        if (additionalRequirements != null) {
            for (Subsystem requirement : additionalRequirements) {
                if (requirement != null && requirement != intake) {
                    requires(requirement);
                }
            }
        }
        setInterruptible(true);
    }

    protected IntakeSubsystem getIntake() {
        return intake;
    }

    @Override
    public void update() {
        // Most intake commands use static modes, so no default periodic logic here.
    }
}
