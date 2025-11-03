package org.firstinspires.ftc.teamcode.bindings;

import dev.nextftc.bindings.Button;
import dev.nextftc.bindings.Range;
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


    public DriverBindings(GamepadEx driver) {
        fieldX = driver.leftStickX().deadZone(TRANSLATION_DEADBAND);
        fieldY = driver.leftStickY().deadZone(TRANSLATION_DEADBAND).negate();
        rotationCcw = driver.rightStickX().deadZone(ROTATION_DEADBAND);
        slowHold = driver.rightBumper();
    }

    public DriveRequest sampleDriveRequest() {
        return new DriveRequest(fieldX.get(), fieldY.get(), rotationCcw.get(), slowHold.get());
    }

    public static final class DriveRequest {
        public final double fieldX;
        public final double fieldY;
        public final double rotation;
        public final boolean slowMode;

        private DriveRequest(double fieldX, double fieldY, double rotation, boolean slowMode) {
            this.fieldX = fieldX;
            this.fieldY = fieldY;
            this.rotation = rotation;
            this.slowMode = slowMode;
        }
    }
}
