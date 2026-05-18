package org.firstinspires.ftc.teamcode.opmodes.Autos.Commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.RobotState;

/**
 * Time-based conditional command. Picks {@code commandIfEnoughTime} or
 * {@code commandIfNotEnoughTime} based on remaining auto time when the command starts.
 *
 * Uses a shared static timer that must be reset at the start of autonomous via
 * {@link #createTimerReset()} (typically as the first command in the sequence).
 *
 * Ported from NextFTC to an Ivy static factory using {@link Commands#conditional}.
 */
public final class ConditionalFinalLaunchCommand {

    private static final ElapsedTime autoTimer = new ElapsedTime();

    private ConditionalFinalLaunchCommand() {}

    /** Resets the shared auto timer. */
    public static void resetTimer() {
        autoTimer.reset();
    }

    /** One-shot Command that resets the timer when it starts. Put first in the auto sequence. */
    public static Command createTimerReset() {
        return Commands.instant(ConditionalFinalLaunchCommand::resetTimer);
    }

    /**
     * Build a Command that picks one of two branches based on remaining auto time
     * when the Ivy scheduler starts the conditional.
     */
    public static Command create(double autoDurationSeconds,
                                  double minTimeForFullSequenceSeconds,
                                  Command commandIfEnoughTime,
                                  Command commandIfNotEnoughTime) {
        return Commands.conditional(
                () -> {
                    double elapsedSeconds = autoTimer.seconds();
                    double timeRemaining = autoDurationSeconds - elapsedSeconds;
                    boolean hasEnoughTime = timeRemaining >= minTimeForFullSequenceSeconds;
                    RobotState.packet.put("Auto/ConditionalLaunch/Elapsed (s)", elapsedSeconds);
                    RobotState.packet.put("Auto/ConditionalLaunch/Remaining (s)", timeRemaining);
                    RobotState.packet.put("Auto/ConditionalLaunch/Min Required (s)", minTimeForFullSequenceSeconds);
                    RobotState.packet.put("Auto/ConditionalLaunch/Has Enough Time", hasEnoughTime);
                    return hasEnoughTime;
                },
                commandIfEnoughTime,
                commandIfNotEnoughTime);
    }
}
