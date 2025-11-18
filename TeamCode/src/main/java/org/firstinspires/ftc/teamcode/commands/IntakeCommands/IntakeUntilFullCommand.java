package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Runs the intake until all lanes are filled (or until a timeout expires).
 * Automatically manages intake mode:
 * - ACTIVE_FORWARD when fewer than 3 artifacts detected
 * - PASSIVE_REVERSE when 3 artifacts detected (full)
 *
 * Use this command in autonomous for intelligent intake automation.
 * In teleop, operators should have direct manual control instead.
 */
public class IntakeUntilFullCommand extends IntakeCommand {

    private final ElapsedTime timer = new ElapsedTime();
    private final double timeoutSeconds;

    public IntakeUntilFullCommand(IntakeSubsystem intake, double timeoutSeconds) {
        super(intake);
        this.timeoutSeconds = timeoutSeconds;
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void execute() {
        // Auto-manage intake mode based on artifact count
        int count = getIntake().getArtifactCount();
        if (count >= 3) {
            getIntake().setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);
        } else {
            getIntake().setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
        }
    }

    @Override
    public boolean isDone() {
        if (timer.seconds() >= timeoutSeconds) {
            return true;
        }
        return getIntake().getArtifactCount() >= 3;
    }

    @Override
    public void stop(boolean interrupted) {
        getIntake().setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);
    }
}
