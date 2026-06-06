package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Drop-in replacement for the REV Color Sensor V3 driver that performs a single 14-byte bulk read
 * of the APDS-9151 data registers per call instead of the multiple short reads REV's stock driver
 * issues. Combined with a REPEAT read window, the Control Hub caches the data in the background so
 * lookups avoid blocking on I2C traffic.
 *
 * Implements the same {@link NormalizedColorSensor}, {@link DistanceSensor}, and
 * {@link SwitchableLight} surface as {@link com.qualcomm.hardware.rev.RevColorSensorV3} so it
 * drops into existing code with no call-site changes once the Driver Station hardware config has
 * been switched to xmlTag="FastColorSensor".
 *
 * Distance calibration constants are copied verbatim from RevColorSensorV3 and assume the same
 * init sequence (LED 125mA @ 60kHz, 32 pulses, 100ms PS rate, 11-bit PS resolution).
 */
@I2cDeviceType
@DeviceProperties(name = "Fast Color Sensor", description = "Bulk-read driver for REV V3 (APDS-9151)", xmlTag = "FastColorSensor")
public class FastColorSensor extends I2cDeviceSynchDevice<I2cDeviceSynch>
        implements NormalizedColorSensor, DistanceSensor, SwitchableLight {

    // ---------- APDS-9151 register map ----------
    private static final int REG_MAIN_CTRL    = 0x00;
    private static final int REG_PS_LED       = 0x01;
    private static final int REG_PS_PULSES    = 0x02;
    private static final int REG_PS_MEAS_RATE = 0x03;
    private static final int REG_LS_MEAS_RATE = 0x04;
    private static final int REG_LS_GAIN      = 0x05;
    private static final int REG_DATA_START   = 0x08;
    private static final int BULK_READ_LENGTH = 14; // PS(2) + IR(3) + G(3) + B(3) + R(3)

    private static final I2cAddr I2C_ADDRESS = I2cAddr.create7bit(0x52);

    // Init values matched to REV's BroadcomColorSensorImpl defaults so distance calibration holds.
    private static final byte MAIN_CTRL_ENABLE = 0x07;     // PS_EN | LS_EN | RGB_MODE
    private static final byte PS_LED_CONFIG    = 0x37;     // 60kHz pulse modulation, 125mA drive current
    private static final byte PS_PULSES_VALUE  = 0x20;     // 32 pulses
    private static final byte PS_MEAS_RATE_CFG = 0x1D;     // 11-bit resolution, 100ms rate
    private static final byte LS_MEAS_RATE_CFG = 0x42;     // 16-bit resolution, 100ms rate
    private static final byte LS_GAIN_DEFAULT  = 0x01;     // GAIN_3

    // Distance fit (RawOptical = a * distance^b + c) — copied from RevColorSensorV3.
    private static final double DIST_A      = 325.961;
    private static final double DIST_B_INV  = -0.75934;
    private static final double DIST_C      = 26.980;
    private static final double DIST_MAX_IN = 6.0;

    private static final double COLOR_SATURATION = 65535.0;
    private static final double BLUE_GAIN  = 1.55; // REV white-balance correction
    private static final double RED_GAIN   = 1.07;

    private final NormalizedRGBA colors = new NormalizedRGBA();
    private float softwareGain = 1f;

    private int rawProximity = 0;
    private int rawRed = 0;
    private int rawGreen = 0;
    private int rawBlue = 0;

    public FastColorSensor(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(I2C_ADDRESS);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        deviceClient.write8(REG_LS_GAIN,      LS_GAIN_DEFAULT);
        deviceClient.write8(REG_PS_LED,       PS_LED_CONFIG);
        deviceClient.write8(REG_PS_PULSES,    PS_PULSES_VALUE);
        deviceClient.write8(REG_PS_MEAS_RATE, PS_MEAS_RATE_CFG);
        deviceClient.write8(REG_LS_MEAS_RATE, LS_MEAS_RATE_CFG);
        deviceClient.write8(REG_MAIN_CTRL,    MAIN_CTRL_ENABLE);

        deviceClient.setReadWindow(new I2cDeviceSynch.ReadWindow(
                REG_DATA_START, BULK_READ_LENGTH, I2cDeviceSynch.ReadMode.REPEAT));
        return true;
    }

    /** Performs a single 14-byte bulk read and caches PS + RGB values. */
    private synchronized void refresh() {
        byte[] data;
        try {
            data = deviceClient.read(REG_DATA_START, BULK_READ_LENGTH);
        } catch (Exception e) {
            // I2C failure — keep stale cached values; sensor recovers on the next REPEAT poll.
            return;
        }
        if (data == null || data.length < BULK_READ_LENGTH) {
            return;
        }
        rawProximity = ((data[0] & 0xFF) | ((data[1] & 0xFF) << 8)) & 0x7FF; // 11-bit valid
        // Bytes 2-4 are IR; we skip them — only RGB and proximity are used downstream.
        rawGreen = (data[5]  & 0xFF) | ((data[6]  & 0xFF) << 8);
        rawBlue  = (data[8]  & 0xFF) | ((data[9]  & 0xFF) << 8);
        rawRed   = (data[11] & 0xFF) | ((data[12] & 0xFF) << 8);
        rawBlue  = Range.clip((int) (BLUE_GAIN * rawBlue), 0, 65535);
        rawRed   = Range.clip((int) (RED_GAIN  * rawRed),  0, 65535);
    }

    @Override
    public synchronized NormalizedRGBA getNormalizedColors() {
        refresh();
        colors.red   = Range.clip((rawRed   * softwareGain) / (float) COLOR_SATURATION, 0f, 1f);
        colors.green = Range.clip((rawGreen * softwareGain) / (float) COLOR_SATURATION, 0f, 1f);
        colors.blue  = Range.clip((rawBlue  * softwareGain) / (float) COLOR_SATURATION, 0f, 1f);
        float avg = (rawRed + rawGreen + rawBlue) / 3f;
        colors.alpha = (float) (1.0 - 65535.0 / (avg * avg + 65535.0));
        return colors;
    }

    @Override
    public float getGain() {
        return softwareGain;
    }

    @Override
    public void setGain(float newGain) {
        this.softwareGain = newGain;
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        refresh();
        double inches;
        if (rawProximity <= DIST_C) {
            inches = DIST_MAX_IN;
        } else {
            inches = Math.min(Math.pow((rawProximity - DIST_C) / DIST_A, DIST_B_INV), DIST_MAX_IN);
        }
        return unit.fromUnit(DistanceUnit.INCH, inches);
    }

    /** Raw 11-bit proximity register value (higher = closer). Exposed for diagnostics. */
    public int getRawProximity() {
        refresh();
        return rawProximity;
    }

    // The APDS-9151 LED on the REV V3 board is hard-wired on; matches RevColorSensorV3 behavior.
    @Override public void enableLight(boolean enable) { }
    @Override public boolean isLightOn() { return true; }

    @Override
    public Manufacturer getManufacturer() {
        return HardwareDevice.Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Fast Color Sensor (APDS-9151)";
    }
}
