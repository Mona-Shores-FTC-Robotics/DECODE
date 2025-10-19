package org.firstinspires.ftc.teamcode.bindings;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.bindings.Range;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.config.Tuning;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;

/**
 * Encapsulates all driver-facing NextFTC bindings for the field-centric TeleOp.
 */
public class TeleopBindings {

    private static final double TRANSLATION_DEADBAND = 0.05;
    private static final double ROTATION_DEADBAND = 0.05;

    private final Range fieldX;
    private final Range fieldY;
    private final Range rotationCcw;
    private final Button precisionHold;

    public TeleopBindings(GamepadEx driver, FlywheelSubsystem flywheel) {
        fieldX = driver.leftStickX().deadZone(TRANSLATION_DEADBAND);
        fieldY = driver.leftStickY().deadZone(TRANSLATION_DEADBAND).negate();
        rotationCcw = driver.rightStickX().deadZone(ROTATION_DEADBAND).negate();
        precisionHold = driver.rightBumper();

        driver.a().whenBecomesTrue(flywheel::stop);
        driver.b().whenBecomesTrue(() -> flywheel.setTargetRpm(Tuning.FLYWHEEL_RPM_LOW));
        driver.x().whenBecomesTrue(() -> flywheel.setTargetRpm(Tuning.FLYWHEEL_RPM_HIGH));
    }

    public DriveRequest sampleDriveRequest() {
        return new DriveRequest(fieldX.get(), fieldY.get(), rotationCcw.get(), precisionHold.get());
    }

    public void reset() {
        BindingManager.reset();
    }

    public static final class DriveRequest {
        public final double fieldX;
        public final double fieldY;
        public final double rotation;
        public final boolean precisionMode;

        private DriveRequest(double fieldX, double fieldY, double rotation, boolean precisionMode) {
            this.fieldX = fieldX;
            this.fieldY = fieldY;
            this.rotation = rotation;
            this.precisionMode = precisionMode;
        }
    }
}
