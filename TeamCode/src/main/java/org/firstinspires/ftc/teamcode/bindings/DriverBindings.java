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
    private final Button rampHold;
    private final Button headingHold;
    private final Button aimHold;
    private final Button relocalizeRequest;


    public DriverBindings(GamepadEx driver) {
        fieldX = driver.leftStickX().deadZone(TRANSLATION_DEADBAND);
        fieldY = driver.leftStickY().deadZone(TRANSLATION_DEADBAND).negate();
        rotationCcw = driver.rightStickX().deadZone(ROTATION_DEADBAND);
        slowHold = driver.rightBumper();
        rampHold = driver.leftBumper();
        headingHold = driver.x();
        aimHold = driver.b();
        relocalizeRequest = driver.a();
    }

    public DriveRequest sampleDriveRequest() {
        return new DriveRequest(
                fieldX.get(),
                fieldY.get(),
                rotationCcw.get(),
                slowHold.get(),
                rampHold.get(),
                headingHold.get(),
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
        public final boolean headingHold;
        public final boolean aimMode;

        private DriveRequest(double fieldX,
                             double fieldY,
                             double rotation,
                             boolean slowMode,
                             boolean rampMode,
                             boolean headingHold,
                             boolean aimMode) {
            this.fieldX = fieldX;
            this.fieldY = fieldY;
            this.rotation = rotation;
            this.slowMode = slowMode;
            this.rampMode = rampMode;
            this.headingHold = headingHold;
            this.aimMode = aimMode;
        }
    }
}
