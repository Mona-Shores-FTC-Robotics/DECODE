package org.firstinspires.ftc.teamcode.bindings;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.AimAndDriveCommand;
import org.firstinspires.ftc.teamcode.commands.CaptureAndAimCommand;
import org.firstinspires.ftc.teamcode.commands.DefaultDriveCommand;

import dev.nextftc.bindings.Button;
import dev.nextftc.bindings.Range;
import dev.nextftc.core.commands.Command;
import dev.nextftc.ftc.GamepadEx;


/**
 * Driver-facing bindings that configure drive commands and button mappings.
 *
 * Drive Commands:
 * - Default: Normal field-centric drive with slow/ramp modes
 * - B button (hold): Continuous aim-and-drive (tracks target while allowing translation)
 * - X button: Capture-once aim (samples target, snaps to fixed heading)
 * - A button: Vision relocalization (instant, no movement)
 */
public class DriverBindings {

    private static final double TRANSLATION_DEADBAND = 0.05;
    private static final double ROTATION_DEADBAND = 0.05;

    private final Range fieldX;
    private final Range fieldY;
    private final Range rotationCcw;
    private final Button slowHold;
    private final Button rampHold;
    private final Button aimHold;
    private final Button relocalizeRequest;
    private final Robot robot;

    public DriverBindings(GamepadEx driver, Robot robot) {
        this.robot = robot;

        // Finalized Driver Buttons
        fieldX = driver.leftStickX().deadZone(TRANSLATION_DEADBAND).negate();
        fieldY = driver.leftStickY().deadZone(TRANSLATION_DEADBAND);
        rotationCcw = driver.rightStickX().deadZone(ROTATION_DEADBAND);
        slowHold = driver.rightBumper();
        aimHold = driver.b();

        //Test Buttons
        rampHold = driver.leftBumper();
        relocalizeRequest = driver.a();

        // B button (hold): Continuous aim-and-drive
        // Tracks target continuously, allows driver translation
        // Set up immediately so button binding is ready (command won't run until default is set)
        Command aimAndDrive = new AimAndDriveCommand(
                fieldX::get,
                fieldY::get,
                slowHold::get,
                robot.drive
        );
        aimHold.whenBecomesTrue(aimAndDrive)
                .whenBecomesFalse(aimAndDrive::cancel);
    }

    /**
     * Enables drive control by setting up the default drive command.
     * Should be called when the match starts (after init) to prevent driving during init.
     */
    public void enableDriveControl() {
        Command defaultDrive = new DefaultDriveCommand(
                fieldX::get,
                fieldY::get,
                rotationCcw::get,
                slowHold::get,
                rampHold::get,
                robot.drive
        );
        robot.drive.setDefaultCommand(defaultDrive);
    }

    /**
     * Registers a callback for vision relocalization requests.
     * A button triggers instant relocalization with no drive movement.
     */
    public void onRelocalizeRequested(Runnable action) {
        if (action == null) {
            return;
        }
        relocalizeRequest.whenBecomesTrue(action);
    }

    /**
     * Samples current driver inputs for telemetry purposes.
     * Note: This does NOT control the robot - commands handle that.
     * This is purely for telemetry/logging to see what the driver is doing.
     */
    public DriveRequest sampleDriveRequest() {
        return new DriveRequest(
                fieldX.get(),
                fieldY.get(),
                rotationCcw.get(),
                slowHold.get(),
                rampHold.get(),
                aimHold.get()
        );
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
