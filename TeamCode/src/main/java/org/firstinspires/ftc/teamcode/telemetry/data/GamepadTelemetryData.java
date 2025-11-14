package org.firstinspires.ftc.teamcode.telemetry.data;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Gamepad telemetry data for both driver and operator.
 * Captures raw driver inputs for debugging and analysis.
 * <p>
 * This data shows what the DRIVER is doing, while DriveTelemetryData
 * shows what the ROBOT is commanded to do (after processing).
 * </p>
 */
public class GamepadTelemetryData {
    public final GamepadSnapshot driver;
    public final GamepadSnapshot operator;

    public GamepadTelemetryData(GamepadSnapshot driver, GamepadSnapshot operator) {
        this.driver = driver != null ? driver : GamepadSnapshot.EMPTY;
        this.operator = operator != null ? operator : GamepadSnapshot.EMPTY;
    }

    /**
     * Snapshot of a single gamepad's state.
     */
    public static class GamepadSnapshot {
        // Analog axes
        public final double leftStickX;
        public final double leftStickY;
        public final double rightStickX;
        public final double rightStickY;
        public final double leftTrigger;
        public final double rightTrigger;

        // Digital buttons
        public final boolean buttonA;
        public final boolean buttonB;
        public final boolean buttonX;
        public final boolean buttonY;
        public final boolean leftBumper;
        public final boolean rightBumper;
        public final boolean leftStickButton;
        public final boolean rightStickButton;
        public final boolean dpadUp;
        public final boolean dpadDown;
        public final boolean dpadLeft;
        public final boolean dpadRight;
        public final boolean back;
        public final boolean start;
        public final boolean guide;

        public static final GamepadSnapshot EMPTY = new GamepadSnapshot(
                0, 0, 0, 0, 0, 0,
                false, false, false, false, false, false, false, false,
                false, false, false, false, false, false, false
        );

        public GamepadSnapshot(
                double leftStickX, double leftStickY,
                double rightStickX, double rightStickY,
                double leftTrigger, double rightTrigger,
                boolean buttonA, boolean buttonB, boolean buttonX, boolean buttonY,
                boolean leftBumper, boolean rightBumper,
                boolean leftStickButton, boolean rightStickButton,
                boolean dpadUp, boolean dpadDown, boolean dpadLeft, boolean dpadRight,
                boolean back, boolean start, boolean guide
        ) {
            this.leftStickX = leftStickX;
            this.leftStickY = leftStickY;
            this.rightStickX = rightStickX;
            this.rightStickY = rightStickY;
            this.leftTrigger = leftTrigger;
            this.rightTrigger = rightTrigger;
            this.buttonA = buttonA;
            this.buttonB = buttonB;
            this.buttonX = buttonX;
            this.buttonY = buttonY;
            this.leftBumper = leftBumper;
            this.rightBumper = rightBumper;
            this.leftStickButton = leftStickButton;
            this.rightStickButton = rightStickButton;
            this.dpadUp = dpadUp;
            this.dpadDown = dpadDown;
            this.dpadLeft = dpadLeft;
            this.dpadRight = dpadRight;
            this.back = back;
            this.start = start;
            this.guide = guide;
        }

        public static GamepadSnapshot capture(Gamepad gamepad) {
            if (gamepad == null) {
                return EMPTY;
            }
            return new GamepadSnapshot(
                    gamepad.left_stick_x,
                    gamepad.left_stick_y,
                    gamepad.right_stick_x,
                    gamepad.right_stick_y,
                    gamepad.left_trigger,
                    gamepad.right_trigger,
                    gamepad.a,
                    gamepad.b,
                    gamepad.x,
                    gamepad.y,
                    gamepad.left_bumper,
                    gamepad.right_bumper,
                    gamepad.left_stick_button,
                    gamepad.right_stick_button,
                    gamepad.dpad_up,
                    gamepad.dpad_down,
                    gamepad.dpad_left,
                    gamepad.dpad_right,
                    gamepad.back,
                    gamepad.start,
                    gamepad.guide
            );
        }
    }

    public static GamepadTelemetryData capture(Gamepad gamepad1, Gamepad gamepad2) {
        return new GamepadTelemetryData(
                GamepadSnapshot.capture(gamepad1),
                GamepadSnapshot.capture(gamepad2)
        );
    }
}
