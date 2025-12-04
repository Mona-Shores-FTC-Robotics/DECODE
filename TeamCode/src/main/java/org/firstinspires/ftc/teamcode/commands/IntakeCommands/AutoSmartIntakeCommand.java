package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotState;

/**
 * Autonomous-oriented smart intake:
 * - Run forward until 3 artifacts are detected
 * - Debounce, then auto-reverse to avoid jamming
 * - When count drops below the resume threshold, debounce and return to forward
 *
 * Runs continuously; never finishes on its own.
 */
@Configurable
public class AutoSmartIntakeCommand extends IntakeCommand {

    @Configurable
    public static class AutoSmartIntakeConfig {
        /** Count (or isFull) threshold to consider intake full */
        public int fullCountThreshold = 3;
        /** Count threshold to resume forward after auto-reverse */
        public int resumeCountThreshold = 2;
        /** Debounce before switching to reverse after full (ms) */
        public double fullDebounceMs = 750;
        /** Debounce before returning to forward after dropping below resume threshold (ms) */
        public double resumeDebounceMs = 200.0;
        /** Enable telemetry in RobotState.packet */
        public boolean enableTelemetry = true;
    }

    public static AutoSmartIntakeConfig config = new AutoSmartIntakeConfig();

    private enum State {
        FORWARD,
        FULL_DEBOUNCE,
        FULL_REVERSED,
        RESUME_DEBOUNCE
    }

    private State state;
    private final ElapsedTime timer = new ElapsedTime();

    public AutoSmartIntakeCommand(IntakeSubsystem intake) {
        super(intake);
    }

    @Override
    public void start() {
        state = State.FORWARD;
        timer.reset();
        getIntake().setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
        publishTelemetry(0, false);
    }

    @Override
    public void update() {
        int artifactCount = getIntake().getArtifactCount();
        boolean isFull = artifactCount >= config.fullCountThreshold || getIntake().isFull();
        boolean belowResume = artifactCount <= config.resumeCountThreshold;

        switch (state) {
            case FORWARD:
                if (isFull) {
                    state = State.FULL_DEBOUNCE;
                    timer.reset();
                }
                break;

            case FULL_DEBOUNCE:
                if (!isFull) {
                    state = State.FORWARD;
                    break;
                }
                if (timer.milliseconds() >= config.fullDebounceMs) {
                    getIntake().setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE);
                    state = State.FULL_REVERSED;
                    timer.reset();
                }
                break;

            case FULL_REVERSED:
                if (belowResume) {
                    state = State.RESUME_DEBOUNCE;
                    timer.reset();
                }
                break;

            case RESUME_DEBOUNCE:
                if (!belowResume) {
                    state = State.FULL_REVERSED;
                    break;
                }
                if (timer.milliseconds() >= config.resumeDebounceMs) {
                    getIntake().setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD);
                    state = State.FORWARD;
                    timer.reset();
                }
                break;
        }

        publishTelemetry(artifactCount, isFull);
    }

    @Override
    public boolean isDone() {
        // Runs until interrupted by the scheduler/auto ending
        return false;
    }

    @Override
    public void stop(boolean interrupted) {
        publishTelemetry(getIntake().getArtifactCount(), getIntake().isFull());
    }

    private void publishTelemetry(int artifactCount, boolean isFull) {
        if (!config.enableTelemetry) {
            return;
        }
        RobotState.packet.put("AutoSmartIntake/state", state == null ? "UNKNOWN" : state.name());
        RobotState.packet.put("AutoSmartIntake/artifactCount", artifactCount);
        RobotState.packet.put("AutoSmartIntake/isFull", isFull);
        RobotState.packet.put("AutoSmartIntake/timerMs", timer.milliseconds());
    }
}
