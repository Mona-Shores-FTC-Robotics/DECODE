package org.firstinspires.ftc.teamcode.bindings;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.AimAndDriveCommand;
import org.firstinspires.ftc.teamcode.commands.AimAndDriveVisionCenteredCommand;
import org.firstinspires.ftc.teamcode.commands.AimAndDriveFixedAngleCommand;
import org.firstinspires.ftc.teamcode.commands.CaptureAndAimCommand;
import org.firstinspires.ftc.teamcode.commands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import dev.nextftc.bindings.Button;
import dev.nextftc.bindings.Range;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.ftc.GamepadEx;


/**
 * Driver-facing bindings that configure drive commands and button mappings.
 *
 * Drive Commands:
 * - Default: Normal field-centric drive with slow/ramp modes
 * - B button (hold): Geometry-based continuous aim-and-drive (uses odometry + incenter)
 *
 * Aiming Methods (for testing/comparison):
 * - B button: Geometry-based (odometry + incenter) - RECOMMENDED for competition
 * - D-pad Up: Vision-centered continuous tracking (centers AprilTag tx)
 * - D-pad Down: Fixed-angle continuous tracking (134° blue, 46° red) - RELIABLE from known positions
 * - D-pad Left: Capture-and-aim snap turn (samples then discrete turn)
 *
 * Vision Relocalization:
 * - A button: MANUAL vision relocalization (instant, no movement)
 * - Automatic relocalization: DISABLED by default (see Robot.RelocalizationConfig)
 *   - Currently using MT1 which can be noisy (4-8" accuracy)
 *   - Manual A button relocalization available when needed
 *   - Will re-enable automatic when MT2 is fixed
 */
public class DriverBindings {

    private static final double TRANSLATION_DEADBAND = 0.05;
    private static final double ROTATION_DEADBAND = 0.05;

    private final Range fieldX;
    private final Range fieldY;
    private final Range rotationCcw;
    private final Button slowHold;
    private final Button aimHold;
    private final Button relocalizeRequest;

    // D-pad buttons for testing different aiming methods
    private final Button aimVisionCentered;
    private final Button aimFixedAngle;
    private final Button aimCaptureSnap;

    Command defaultDrive;
    Command aimAndDrive;

    public DriverBindings(GamepadEx driver) {

        // Finalized Driver Buttons
        fieldX = driver.leftStickX().deadZone(TRANSLATION_DEADBAND).negate();
        fieldY = driver.leftStickY().deadZone(TRANSLATION_DEADBAND);
        rotationCcw = driver.rightStickX().deadZone(ROTATION_DEADBAND);
        slowHold = driver.rightBumper();
        aimHold = driver.b();

        //Test Buttons
        relocalizeRequest = driver.a();

        // D-pad for testing different aiming methods
        aimVisionCentered = driver.dpadUp();
        aimFixedAngle = driver.dpadDown();
        aimCaptureSnap = driver.dpadLeft();
    }

    /**
     * Enables drive control by setting up the default drive command.
     * Should be called when the match starts (after init) to prevent driving during init.
     */
    public void configureTeleopBindings(Robot robot) {

        defaultDrive = new DefaultDriveCommand(
                fieldX::get,
                fieldY::get,
                rotationCcw::get,
                slowHold::get,
                () -> false,  // rampMode disabled for testing
                robot.drive
        );

//        slowHold.whenBecomesTrue(()->robot.intake.setMode(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))
//                .whenBecomesFalse(new SequentialGroup(
//                        new Delay(1),
//                        new InstantCommand(()->robot.intake.setMode(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE)
//                        )));

        // Method 1: Geometry-based (B button - current default)
        aimAndDrive = new AimAndDriveCommand(
                fieldX::get,
                fieldY::get,
                slowHold::get,
                robot.drive
        );
        robot.drive.setDefaultCommand(defaultDrive);

        aimHold.whenBecomesTrue(aimAndDrive)
                .whenBecomesFalse(aimAndDrive::cancel);

        // Method 2: Vision-centered (D-pad Up)
        Command aimVisionCenteredCmd = new AimAndDriveVisionCenteredCommand(
                fieldX::get,
                fieldY::get,
                slowHold::get,
                robot.drive
        );
        aimVisionCentered.whenBecomesTrue(aimVisionCenteredCmd)
                .whenBecomesFalse(aimVisionCenteredCmd::cancel);

        // Method 3: Fixed-angle (D-pad Down)
        Command aimFixedAngleCmd = new AimAndDriveFixedAngleCommand(
                fieldX::get,
                fieldY::get,
                slowHold::get,
                robot.drive
        );
        aimFixedAngle.whenBecomesTrue(aimFixedAngleCmd)
                .whenBecomesFalse(aimFixedAngleCmd::cancel);

        if (robot.vision != null) {
            // Method 4: Capture-and-aim snap turn (D-pad Left)
            Command captureAndAimCmd = new CaptureAndAimCommand(robot.drive, robot.vision);
            aimCaptureSnap.whenBecomesTrue(captureAndAimCmd)
                    .whenBecomesFalse(captureAndAimCmd::cancel);

            relocalizeRequest.whenBecomesTrue(robot.drive::tryRelocalize);
        }
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
                false,  // rampMode disabled
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
