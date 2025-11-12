package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.core.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Convenience factory for intake-related commands and immediate helpers.
 */
@Configurable
public class IntakeCommands {

    public static final double DEFAULT_INTAKE_TIMEOUT_S = 5.0;

    private final IntakeSubsystem intake;

    public IntakeCommands(IntakeSubsystem intake) {
        this.intake = intake;
    }

    /** Runs the intake forward for a fixed time, then returns to passive reverse. */
    public TimedIntakeCommand timedIntake(double seconds) {
        return new TimedIntakeCommand(intake, seconds);
    }

    /** Shortcut for a timed intake with the default timeout. */
    public TimedIntakeCommand timedIntake() {
        return timedIntake(DEFAULT_INTAKE_TIMEOUT_S);
    }

    /** Runs the intake forward until all lanes are full (or timeout reached). */
    public IntakeUntilFullCommand intakeUntilFull(double timeoutSeconds) {
        return new IntakeUntilFullCommand(intake, timeoutSeconds);
    }

    /** Shortcut for default "until full" timeout. */
    public IntakeUntilFullCommand intakeUntilFull() {
        return intakeUntilFull(DEFAULT_INTAKE_TIMEOUT_S);
    }

}
