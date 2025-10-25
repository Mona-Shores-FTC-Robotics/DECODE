package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.auto.Alliance;

/**
 * goBILDA RGB Indicator Light helper.
 * Wire the light to a REV servo port (JST-PH) and configure it as a standard Servo.
 * Then call AllianceLight.applyAlliance(...) during INIT of your OpMode.
 */
@Configurable
public class AllianceLight {

    // --- Dashboard-tunable servo "positions" for colors (0.0..1.0 maps ~500..2500 µs) ---
    public static double GREEN_POS  = 0.500;   // adjust if your unit’s hue doesn’t look pure red
    public static double PURPLE_POS = 0.722;   // adjust if needed
    public static double RED_POS  = 0.281;   // adjust if your unit’s hue doesn’t look pure red
    public static double BLUE_POS = 0.611;   // adjust if needed

    private final Servo led;

    private AllianceLight(Servo led) {
        this.led = led;
    }

    public static AllianceLight onServo(HardwareMap hw, String servoName) {
        return new AllianceLight(hw.get(Servo.class, servoName));
    }

    /** Sets light to a specific servo position (0..1). */
    public void setRaw(double position) {
        led.setPosition(clamp01(position));
    }

    /** Convenience: set red or blue based on selected alliance. */
    public void applyAlliance(Alliance alliance) {
        if (alliance == Alliance.RED) {
            setRaw(RED_POS);
        } else if (alliance == Alliance.BLUE) {
            setRaw(BLUE_POS);
        }
    }

    private static double clamp01(double x) { return Math.max(0.0, Math.min(1.0, x)); }
}
