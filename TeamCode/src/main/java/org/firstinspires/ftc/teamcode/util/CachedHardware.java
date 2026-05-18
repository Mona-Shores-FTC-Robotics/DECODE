package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

/**
 * Helpers that wrap {@link HardwareMap} lookups in Dairy CachingHardware wrappers
 * so redundant {@code setPower} / {@code setPosition} calls within tolerance of
 * the cached value are dropped before reaching the I2C bus.
 *
 * <p>Caching tolerance defaults come from V3 (BeepBot99/CodeBloodedDecodeV3):
 * 0.005 (0.5%) of full scale for both motors and servos. Tunable via Panels.
 *
 * <p>The {@code try*} variants return {@code null} on lookup failure (the existing
 * tryGetMotor/tryGetServo pattern used by every subsystem); the non-try variants
 * propagate the underlying {@link IllegalArgumentException}.
 *
 * <p>This caches OUTGOING actuator writes. It is unrelated to
 * {@link org.firstinspires.ftc.teamcode.subsystems.intake.FastColorSensor}, which
 * caches INCOMING I2C reads from the REV V3 color/distance sensor.
 */
@Configurable
public final class CachedHardware {

    @Configurable
    public static class CacheConfig {
        /** Minimum power delta (0.0–1.0) before forwarding setPower to a motor. */
        public static double motorTolerance = 0.005;
        /** Minimum position delta (0.0–1.0) before forwarding setPosition to a servo. */
        public static double servoTolerance = 0.005;
    }

    private CachedHardware() {}

    /**
     * Look up a motor by name and wrap it in {@link CachingDcMotorEx}.
     * Throws if the motor is not found in the hardware config.
     */
    public static DcMotorEx motor(HardwareMap hardwareMap, String name) {
        return new CachingDcMotorEx(
                hardwareMap.get(DcMotorEx.class, name),
                CacheConfig.motorTolerance);
    }

    /**
     * Look up a motor by name and wrap it in {@link CachingDcMotorEx}. Returns
     * {@code null} if the name is blank or the motor isn't in the hardware config,
     * matching the existing {@code tryGetMotor} pattern in subsystems.
     */
    public static DcMotorEx tryMotor(HardwareMap hardwareMap, String name) {
        if (name == null || name.isEmpty()) return null;
        try {
            return motor(hardwareMap, name);
        } catch (IllegalArgumentException ignored) {
            return null;
        }
    }

    /**
     * Look up a servo by name and wrap it in {@link CachingServo}.
     * Throws if the servo is not found in the hardware config.
     */
    public static Servo servo(HardwareMap hardwareMap, String name) {
        return new CachingServo(
                hardwareMap.get(Servo.class, name),
                CacheConfig.servoTolerance);
    }

    /**
     * Look up a servo by name and wrap it in {@link CachingServo}. Returns
     * {@code null} if the name is blank or the servo isn't in the hardware config.
     */
    public static Servo tryServo(HardwareMap hardwareMap, String name) {
        if (name == null || name.isEmpty()) return null;
        try {
            return servo(hardwareMap, name);
        } catch (IllegalArgumentException ignored) {
            return null;
        }
    }
}
