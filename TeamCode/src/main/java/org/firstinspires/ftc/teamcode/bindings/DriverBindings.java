package org.firstinspires.ftc.teamcode.bindings;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AimAndDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AimAndDriveFixedAngleCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AimAndDriveRightTriggerCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.DefaultDriveCommand;

import dev.nextftc.bindings.Button;
import dev.nextftc.bindings.Range;
import dev.nextftc.core.commands.Command;
import dev.nextftc.ftc.GamepadEx;

public class DriverBindings {

    private static final double TRANSLATION_DEADBAND = 0.05;
    private static final double ROTATION_DEADBAND = 0.05;
    private static final double TRIGGER_THRESHOLD = 0.5;

    private final Range fieldX;
    private final Range fieldY;
    private final Range rotationCcw;
    private final Button slowHold;
    private final Button aimAtGoal;
    private final Button relocalizeRequest;
    private final Button headingResetRequest;
    private final Button aimFixedAngleGoal;
    private final Button aimRightTriggerPark;

    Command defaultDrive;
    Command aimAndDrive;

    public DriverBindings(GamepadEx driver) {

        // Finalized Driver Buttons
        fieldX = driver.leftStickX().deadZone(TRANSLATION_DEADBAND).negate();
        fieldY = driver.leftStickY().deadZone(TRANSLATION_DEADBAND);
        rotationCcw = driver.rightStickX().deadZone(ROTATION_DEADBAND);
        slowHold = driver.rightBumper();
        aimFixedAngleGoal = driver.triangle();
        aimAtGoal = driver.circle();
        aimRightTriggerPark = driver.rightTrigger().greaterThan(TRIGGER_THRESHOLD);

        relocalizeRequest = driver.cross();
        headingResetRequest = driver.square();

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

        aimAndDrive = new AimAndDriveCommand(
                fieldX::get,
                fieldY::get,
                slowHold::get,
                robot.drive
        );
        robot.drive.setDefaultCommand(defaultDrive);

        aimAtGoal.whenBecomesTrue(aimAndDrive)
                .whenBecomesFalse(aimAndDrive::cancel);

        Command aimFixedAngleCmd = new AimAndDriveFixedAngleCommand(
                fieldX::get,
                fieldY::get,
                slowHold::get,
                robot.drive
        );
        aimFixedAngleGoal.whenBecomesTrue(aimFixedAngleCmd)
                .whenBecomesFalse(aimFixedAngleCmd::cancel);

        Command aimRightTriggerCmd = new AimAndDriveRightTriggerCommand(
                fieldX::get,
                fieldY::get,
                slowHold::get,
                robot.drive
        );
        aimRightTriggerPark.whenBecomesTrue(aimRightTriggerCmd)
                .whenBecomesFalse(aimRightTriggerCmd::cancel);

        if (robot.vision != null) {
            relocalizeRequest.whenBecomesTrue(robot.drive::tryRelocalize);
            headingResetRequest.whenBecomesTrue(robot.drive::resetHeadingToFieldForward);
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
                aimAtGoal.get()
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

    /**
     * Human-readable control summary for driver station telemetry.
     */
    public static String[] controlsSummary() {
        return new String[]{
                "Left stick: Drive (X/Y)",
                "Right stick X: Rotate",
                "Right bumper: Slow mode (hold)",
                "Right trigger: Fixed heading (265\u00b0 blue / 275\u00b0 red)",
                "Circle: Aim+drive (hold)",
                "Triangle: Fixed-angle aim (hold)",
                "Cross: Vision relocalize",
                "Square: Reset heading to face obelisk"
        };
    }
}
