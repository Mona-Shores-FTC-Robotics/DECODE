package org.firstinspires.ftc.teamcode.util;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;

/**
 * Coordinates launcher mode selection (THROUGHPUT vs DECODE) during autonomous init.
 * Uses operator gamepad dpad for mode selection:
 * - Dpad Left: THROUGHPUT mode (rapid firing)
 * - Dpad Right: DECODE mode (sequential pattern firing)
 */
public final class LauncherModeSelector {

    private final LauncherMode defaultMode;
    private final Button throughputButton;
    private final Button decodeButton;

    private boolean selectionLocked;
    private LauncherMode selectedMode;

    /**
     * Creates a launcher mode selector bound to operator gamepad.
     *
     * @param operatorPad The operator gamepad (gamepad2)
     * @param defaultMode The default mode if no selection is made
     */
    public LauncherModeSelector(GamepadEx operatorPad, LauncherMode defaultMode) {
        this.defaultMode = defaultMode == null ? LauncherMode.THROUGHPUT : defaultMode;
        this.selectedMode = this.defaultMode;

        throughputButton = operatorPad.dpadLeft();
        decodeButton = operatorPad.dpadRight();

        throughputButton.whenBecomesTrue(() -> selectMode(LauncherMode.THROUGHPUT));
        decodeButton.whenBecomesTrue(() -> selectMode(LauncherMode.DECODE));
    }

    /**
     * Selects the launcher mode manually via button press.
     *
     * @param mode The mode to select
     */
    public void selectMode(LauncherMode mode) {
        if (selectionLocked) {
            return;
        }
        selectedMode = mode == null ? defaultMode : mode;
        RobotState.setLauncherMode(selectedMode);
    }

    /**
     * Applies the current selected mode to RobotState.
     */
    public void applySelection() {
        LauncherMode modeToApply = selectedMode != null ? selectedMode : defaultMode;
        RobotState.setLauncherMode(modeToApply);
    }

    /**
     * Applies the current selected mode to RobotState and updates lighting feedback.
     *
     * @param lighting Lighting subsystem for visual feedback (can be null)
     */
    public void applySelection(LightingSubsystem lighting) {
        applySelection();
        // Optional: Add lighting feedback for mode selection if desired
        // lighting.showLauncherMode(selectedMode);
    }

    /**
     * Convenience method for updating launcher mode selection during OpMode init (waitForStart loop).
     * Processes button presses and applies the selection to RobotState.
     *
     * <p>Usage example in waitForStart:
     * <pre>{@code
     * BindingManager.update(); // Process button presses
     * LauncherMode currentMode = modeSelector.updateDuringInit(robot.lighting);
     * telemetry.addData("Launcher Mode", currentMode);
     * }</pre>
     *
     * @param lighting Lighting subsystem for visual feedback (can be null)
     * @return The currently selected launcher mode
     */
    public LauncherMode updateDuringInit(LightingSubsystem lighting) {
        applySelection(lighting);
        return selectedMode;
    }

    /**
     * Gets the currently selected launcher mode.
     *
     * @return The selected mode
     */
    public LauncherMode getSelectedMode() {
        return selectedMode;
    }

    /**
     * Locks the selection to prevent changes after start button is pressed.
     * Call this in onStartButtonPressed() to prevent accidental mode changes during auto.
     */
    public void lockSelection() {
        selectionLocked = true;
    }

    /**
     * Unlocks the selection to allow changes in subsequent runs.
     * Call this in onStop() to reset for next match.
     */
    public void unlockSelection() {
        selectionLocked = false;
    }

    /**
     * Gets display text for telemetry showing current selection and controls.
     *
     * @return Formatted string for telemetry display
     */
    public String getDisplayText() {
        return String.format("%s (Op◀THRU ▶DEC)", selectedMode);
    }
}
