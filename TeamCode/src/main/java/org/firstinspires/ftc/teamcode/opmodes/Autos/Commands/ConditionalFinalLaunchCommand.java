package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.RobotState;

import dev.nextftc.core.commands.Command;

/**
 * Generic time-based conditional command that executes different command sequences
 * based on remaining autonomous time. Evaluates the condition when start() is called.
 *
 * IMPORTANT: This command uses a shared static timer that must be reset at the start
 * of the autonomous sequence using {@link #resetTimer()}. Call this at the beginning
 * of your auto sequence (e.g., as the first command) to ensure accurate time tracking.
 *
 * Usage example:
 * <pre>
 * new SequentialGroup(
 *     ConditionalFinalLaunchCommand.createTimerReset(),  // Reset timer at auto start
 *     // ... other commands ...
 *     new ConditionalFinalLaunchCommand(
 *         30.0,  // total auto duration
 *         5.0,   // minimum time needed for "has time" branch
 *         new SequentialGroup(...),  // command if enough time remains
 *         new SequentialGroup(...)   // command if not enough time
 *     )
 * )
 * </pre>
 */
public class ConditionalFinalLaunchCommand extends Command {

    // Shared timer across all instances - reset at auto start
    private static final ElapsedTime autoTimer = new ElapsedTime();

    private final double autoDurationSeconds;
    private final double minTimeForFullSequenceSeconds;
    private final Command commandIfEnoughTime;
    private final Command commandIfNotEnoughTime;
    private Command selectedCommand;

    /**
     * Resets the shared auto timer. Call this at the start of autonomous.
     */
    public static void resetTimer() {
        autoTimer.reset();
    }

    /**
     * Creates a command that resets the auto timer when started.
     * Use this as the first command in your autonomous sequence.
     * @return Command that resets the timer
     */
    public static Command createTimerReset() {
        return new Command() {
            @Override
            public void start() {
                resetTimer();
            }

            @Override
            public void update() {}

            @Override
            public boolean isDone() {
                return true;
            }
        };
    }

    /**
     * Creates a time-based conditional command.
     * @param autoDurationSeconds Total autonomous duration (typically 30 seconds)
     * @param minTimeForFullSequenceSeconds Minimum time required to run the "enough time" branch
     * @param commandIfEnoughTime Command to run if enough time remains
     * @param commandIfNotEnoughTime Command to run if not enough time remains
     */
    public ConditionalFinalLaunchCommand(
            double autoDurationSeconds,
            double minTimeForFullSequenceSeconds,
            Command commandIfEnoughTime,
            Command commandIfNotEnoughTime) {
        this.autoDurationSeconds = autoDurationSeconds;
        this.minTimeForFullSequenceSeconds = minTimeForFullSequenceSeconds;
        this.commandIfEnoughTime = commandIfEnoughTime;
        this.commandIfNotEnoughTime = commandIfNotEnoughTime;
    }

    @Override
    public void start() {
        // Evaluate condition when command starts
        double elapsedSeconds = autoTimer.seconds();
        double timeRemaining = autoDurationSeconds - elapsedSeconds;
        boolean hasEnoughTime = timeRemaining >= minTimeForFullSequenceSeconds;

        // Log decision info to telemetry
        RobotState.packet.put("Auto/ConditionalLaunch/Elapsed (s)", elapsedSeconds);
        RobotState.packet.put("Auto/ConditionalLaunch/Remaining (s)", timeRemaining);
        RobotState.packet.put("Auto/ConditionalLaunch/Min Required (s)", minTimeForFullSequenceSeconds);
        RobotState.packet.put("Auto/ConditionalLaunch/Has Enough Time", hasEnoughTime);

        if (hasEnoughTime) {
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
