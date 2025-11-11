package org.firstinspires.ftc.teamcode.bindings;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.AimAtGoalCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import dev.nextftc.bindings.Button;
import dev.nextftc.bindings.Range;
import dev.nextftc.core.commands.Command;
import dev.nextftc.ftc.GamepadEx;


/**
 * Driver-facing bindings that turn gamepad inputs into drive commands.
 * Only the scaling flags are captured here; higher level logic (auto heading,
 * pose hold, etc.) lives in the {@link org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem}.
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
    private final Button aim;

    public DriverBindings(GamepadEx driver, Robot robot) {
        fieldX = driver.leftStickX().deadZone(TRANSLATION_DEADBAND).negate();
        fieldY = driver.leftStickY().deadZone(TRANSLATION_DEADBAND);
        rotationCcw = driver.rightStickX().deadZone(ROTATION_DEADBAND);
        slowHold = driver.rightBumper();
        rampHold = driver.leftBumper();
        aim = driver.x();
        aimHold = driver.b();
        relocalizeRequest = driver.a();

        Command aimCommand = new AimAtGoalCommand(robot.drive, ()-> 0, ()-> 0, ()-> false,
                5,
                120,
                true,
                true,
                1000);

        aim.whenBecomesTrue(aimCommand);
    }

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

    public void onRelocalizeRequested(Runnable action) {
        if (action == null) {
            return;
        }
        relocalizeRequest.whenBecomesTrue(action);
    }

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
