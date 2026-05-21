package org.firstinspires.ftc.teamcode.bindings;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.GamepadBindings;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Driver gamepad bindings. Uses {@link GamepadBindings} on top of raw FTC gamepad
 * polling (no NextFTC GamepadEx). Trigger sources are {@link BooleanSupplier}
 * lambdas; commands come from subsystem Traffic Cones factory methods.
 *
 * The default drive command runs continuously at priority 0 with
 * {@code interruptedBehavior=SUSPEND}; aim commands at priority 1 preempt it
 * and the default resumes when they end.
 */
public class DriverBindings {

    /** Stick magnitudes below this are treated as zero (joysticks never quite center). */
    private static final double TRANSLATION_DEADBAND = 0.05;
    /** Rotation-stick magnitudes below this are treated as zero. */
    private static final double ROTATION_DEADBAND = 0.05;
    /** A trigger is "pressed" once it crosses this fraction of full pull (0.0–1.0). */
    private static final double TRIGGER_THRESHOLD = 0.5;

    private final GamepadBindings bindings = new GamepadBindings();
    private final Supplier<Gamepad> gamepadSupplier;

    /**
     * Takes a {@link Supplier} so the gamepad reference can be created at field-init
     * time (when {@code gamepad1} is still null) and read lazily at runtime.
     */
    public DriverBindings(Supplier<Gamepad> driverGamepadSupplier) {
        this.gamepadSupplier = driverGamepadSupplier;
    }

    private Gamepad gp() { return gamepadSupplier.get(); }

    /**
     * Wire up the bindings against the robot. Must be called once after the
     * Ivy Scheduler has been reset and before {@link #update()} is called.
     * The default drive Command is scheduled here; aim commands are
     * scheduled by edge-detection on each call to {@link #update()}.
     */
    public void configureTeleopBindings(Robot robot) {
        DoubleSupplier fieldX = () -> applyDeadband(-gp().left_stick_x, TRANSLATION_DEADBAND);
        DoubleSupplier fieldY = () -> applyDeadband(-gp().left_stick_y, TRANSLATION_DEADBAND);
        DoubleSupplier rotationCcw = () -> applyDeadband(gp().right_stick_x, ROTATION_DEADBAND);
        BooleanSupplier slowHold = () -> gp().right_bumper;
        BooleanSupplier rampMode = () -> false;  // unused — kept for future

        // Default drive (priority 0, SUSPEND on interrupt) runs continuously.
        robot.drive.defaultDrive(fieldX, fieldY, rotationCcw, slowHold, rampMode).schedule();

        // Aim assist (circle) — geometry to basket centroid.
        bindings.when(() -> gp().circle)
                .whileTrue(robot.drive.aimAndDriveCmd(fieldX, fieldY, slowHold));

        // Aim assist (triangle) — fixed angle to alliance goal.
        bindings.when(() -> gp().triangle)
                .whileTrue(robot.drive.aimAndDriveFixedAngleCmd(fieldX, fieldY, slowHold));

        // Aim assist (right trigger > threshold) — right-trigger fixed angle (park).
        bindings.when(() -> gp().right_trigger > TRIGGER_THRESHOLD)
                .whileTrue(robot.drive.aimAndDriveRightTriggerCmd(fieldX, fieldY, slowHold));

        // Cross: vision-driven relocalization (one-shot, resets Kalman covariance).
        // Square: heading reset to field-forward (one-shot).
        if (robot.vision != null) {
            bindings.when(() -> gp().cross).onTrue(robot.drive.tryRelocalizeCmd());
            bindings.when(() -> gp().square).onTrue(robot.drive.resetHeadingCmd());
        }
    }

    /** Poll bindings each loop. Schedules commands on rising / falling edges. */
    public void update() {
        bindings.update();
    }

    /**
     * Samples current driver inputs for telemetry purposes.
     * Note: this does NOT control the robot — commands handle that.
     */
    public DriveRequest sampleDriveRequest() {
        return new DriveRequest(
                applyDeadband(-gp().left_stick_x, TRANSLATION_DEADBAND),
                applyDeadband(-gp().left_stick_y, TRANSLATION_DEADBAND),
                applyDeadband(gp().right_stick_x, ROTATION_DEADBAND),
                gp().right_bumper,
                false,
                gp().circle
        );
    }

    private static double applyDeadband(double value, double deadband) {
        return Math.abs(value) < deadband ? 0.0 : value;
    }

    /**
     * Human-readable control summary for driver-station telemetry.
     */
    public static String[] controlsSummary() {
        return new String[]{
                "Left stick: drive (field-centric)",
                "Right stick X: rotation",
                "Right bumper: slow mode (hold)",
                "Circle: aim at goal (hold)",
                "Triangle: fixed-angle aim (hold)",
                "Right trigger: park / right-trigger aim (hold)",
                "Cross: vision relocalize",
                "Square: heading reset to field-forward"
        };
    }

    /**
     * Driver input snapshot for telemetry/logging.
     * This is not used for control (commands handle that), only for monitoring.
     */
    public static final class DriveRequest {
        public final double fieldX;
        public final double fieldY;
        public final double rotation;
        public final boolean slowMode;
        public final boolean rampMode;
        public final boolean aimMode;

        private DriveRequest(double fieldX,
                             double fieldY,
                             double rotation,
                             boolean slowMode,
                             boolean rampMode,
                             boolean aimMode) {
            this.fieldX = fieldX;
            this.fieldY = fieldY;
            this.rotation = rotation;
            this.slowMode = slowMode;
            this.rampMode = rampMode;
            this.aimMode = aimMode;
        }
    }
}
