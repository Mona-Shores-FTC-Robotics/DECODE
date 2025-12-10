package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotState;

/**
 * Smart intake command with automatic reversal when full.
 *
 * Behavior:
 * - Button press: Start intaking forward
 * - While held: Monitor isFull() continuously
 * - When full detected: Continue intaking for safety delay, then auto-reverse
 * - Button release: Command interrupted, binding handles reversal
 * - Re-press: Fresh command instance, gets new safety delay
 *
 * Usage in OperatorBindings:
 * <pre>
 * SmartIntakeCommand smartIntakeCommand = new SmartIntakeCommand(robot.intake, gamepad2);
 * SetIntakeModeCommand reverseCommand = new SetIntakeModeCommand(robot.intake, PASSIVE_REVERSE);
 *
 * runIntake.whenBecomesTrue(smartIntakeCommand);
 * runIntake.whenBecomesFalse(reverseCommand);
 * </pre>
 */
@Configurable
public class SmartIntakeCommand extends IntakeCommand {

    public static class SmartIntakeConfig {
        /**
         * Continue intaking for this duration AFTER isFull() becomes true.
         * This ensures artifacts are fully seated before reversing.
         *
         * Tuning guide:
         * - Start at 400ms
         * - Increase by 100ms if artifacts fall out after auto-reverse
         * - Decrease by 50ms if intake feels sluggish
         * - Typical range: 300-700ms
         */
        public double fullDebounceMs = 150;

        /**
         * Enable haptic rumble when auto-reversing.
         */
        public boolean enableHapticFeedback = true;

        /**
         * Enable debug telemetry to RobotState.packet for tuning.
         */
        public boolean enableDebugTelemetry = true;
    }

    public static SmartIntakeConfig config = new SmartIntakeConfig();

    private enum State {
        INTAKING,       // Normal forward intake, monitoring fullness
        DEBOUNCING,     // Full detected, continuing intake for safety delay
        AUTO_REVERSED   // Auto-reversed due to fullness, waiting for button release
    }

    private State state;
    private final ElapsedTime debounceTimer = new ElapsedTime();
    private final Gamepad gamepad; // For optional haptic feedback

    public SmartIntakeCommand(IntakeSubsystem intake, Gamepad gamepad) {
        super(intake);
        this.gamepad = gamepad;
    }

    @Override
    public void start() {
        boolean isFull = getIntake().isFull();
        if (isFull)
        { state= State.AUTO_REVERSED;}
        else {
            state = State.INTAKING;
        }
        if (config.enableDebugTelemetry) {
            RobotState.packet.put("SmartIntake/state", state.name());
        }
    }

    @Override
    public void update() {
        boolean isFull = getIntake().isFull();

        switch (state) {
            case INTAKING:
                if (isFull) {
                    // Full detected - start debounce timer
                    state = State.DEBOUNCING;
                    debounceTimer.reset();

                    if (config.enableDebugTelemetry) {
                        RobotState.packet.put("SmartIntake/state", state.name());
                        RobotState.packet.put("SmartIntake/fullDetectedTime", debounceTimer.milliseconds());
                    }
                } else {
                    getIntake().setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
                }
                break;

            case DEBOUNCING:
                // Check if we're no longer full (false alarm)
                if (!isFull) {
                    // Back to intaking
                    state = State.INTAKING;

                    if (config.enableDebugTelemetry) {
                        RobotState.packet.put("SmartIntake/state", state.name());
                        RobotState.packet.put("SmartIntake/falseAlarm", true);
                    }
                    break;
                }

                // Continue intaking for safety delay
                if (debounceTimer.milliseconds() >= config.fullDebounceMs) {
                    state = State.AUTO_REVERSED;
                    getIntake().setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);

                    // Haptic feedback
                    if (config.enableHapticFeedback && gamepad != null) {
                        gamepad.rumble(500); // Strong rumble = auto-reversed
                    }

                    if (config.enableDebugTelemetry) {
                        RobotState.packet.put("SmartIntake/state", state.name());
                        RobotState.packet.put("SmartIntake/autoReversedTime", debounceTimer.milliseconds());
                    }
                }
                break;

            case AUTO_REVERSED:
                if (!isFull) {
                    state = State.INTAKING;
                }
                break;
        }

        // Debug telemetry
        if (config.enableDebugTelemetry) {
            RobotState.packet.put("SmartIntake/isFull", isFull);
            if (state == State.DEBOUNCING) {
                RobotState.packet.put("SmartIntake/debounceTimeMs", debounceTimer.milliseconds());
            }
        }
    }

    @Override
    public boolean isDone() {
        // Never end on its own - runs until button released
        return false;
    }

    @Override
    public void stop(boolean interrupted) {
        // Cleanup happens via whenBecomesFalse binding
        if (config.enableDebugTelemetry) {
            RobotState.packet.put("SmartIntake/stopped", interrupted ? "interrupted" : "normal");
        }
    }
}
