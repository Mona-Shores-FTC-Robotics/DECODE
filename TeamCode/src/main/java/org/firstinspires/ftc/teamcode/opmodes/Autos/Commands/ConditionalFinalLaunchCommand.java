package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;

/**
 * Generic time-based conditional command that executes different command sequences
 * based on remaining autonomous time. Evaluates the condition when start() is called,
 * avoiding the lateinit error that occurs with NextFTC's IfElseCommand.
 *
 * Usage example:
 * <pre>
 * new ConditionalFinalLaunchCommand(
 *     timer,
 *     30.0,  // total auto duration
 *     5.0,   // minimum time needed for "has time" branch
 *     new SequentialGroup(...),  // command if enough time remains
 *     new SequentialGroup(...)   // command if not enough time
 * )
 * </pre>
 */
public class ConditionalFinalLaunchCommand extends Command {

    private final ElapsedTime timer;
    private final double autoDurationSeconds;
    private final double minTimeForFullSequenceSeconds;
    private final Command commandIfEnoughTime;
    private final Command commandIfNotEnoughTime;
    private Command selectedCommand;

    /**
     * Creates a time-based conditional command.
     * @param timer Elapsed time tracker for the autonomous period
     * @param autoDurationSeconds Total autonomous duration (typically 30 seconds)
     * @param minTimeForFullSequenceSeconds Minimum time required to run the "enough time" branch
     * @param commandIfEnoughTime Command to run if enough time remains
     * @param commandIfNotEnoughTime Command to run if not enough time remains
     */
    public ConditionalFinalLaunchCommand(
            ElapsedTime timer,
            double autoDurationSeconds,
            double minTimeForFullSequenceSeconds,
            Command commandIfEnoughTime,
            Command commandIfNotEnoughTime) {
        this.timer = timer;
        this.autoDurationSeconds = autoDurationSeconds;
        this.minTimeForFullSequenceSeconds = minTimeForFullSequenceSeconds;
        this.commandIfEnoughTime = commandIfEnoughTime;
        this.commandIfNotEnoughTime = commandIfNotEnoughTime;
    }

    @Override
    public void start() {
        // Evaluate condition when command starts
        double timeRemaining = autoDurationSeconds - timer.seconds();
        if (timeRemaining >= minTimeForFullSequenceSeconds) {
            selectedCommand = commandIfEnoughTime;
        } else {
            selectedCommand = commandIfNotEnoughTime;
        }
        selectedCommand.start();
    }

    @Override
    public void update() {
        if (selectedCommand != null) {
            selectedCommand.update();
        }
    }

    @Override
    public void stop(boolean interrupted) {
        if (selectedCommand != null) {
            selectedCommand.stop(interrupted);
        }
    }

    @Override
    public boolean isDone() {
        return selectedCommand != null && selectedCommand.isDone();
    }
}
