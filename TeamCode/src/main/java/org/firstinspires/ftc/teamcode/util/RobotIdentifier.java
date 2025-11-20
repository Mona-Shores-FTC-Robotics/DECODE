package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * Utility for identifying which physical robot is running the code.
 * Useful when you have multiple robots and need to distinguish between them.
 */
public class RobotIdentifier {

    private final String configName;
    private final String controlHubSerial;
    private final String expansionHubSerial;

    /**
     * Creates a RobotIdentifier from the current hardware map.
     *
     * @param hardwareMap The hardware map from an OpMode
     */
    public RobotIdentifier(HardwareMap hardwareMap) {
        // Get the active configuration name
        this.configName = hardwareMap.getActiveConfigurationName();

        // Get Control Hub and Expansion Hub serial numbers
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        String controlSerial = "UNKNOWN";
        String expansionSerial = "UNKNOWN";

        for (LynxModule module : allHubs) {
            if (module.isParent()) {
                // This is the Control Hub
                controlSerial = module.getSerialNumber().toString();
            } else {
                // This is an Expansion Hub
                expansionSerial = module.getSerialNumber().toString();
            }
        }

        this.controlHubSerial = controlSerial;
        this.expansionHubSerial = expansionSerial;
    }

    /**
     * Gets the active robot configuration name.
     * This is the easiest way to identify robots - just use different config files.
     *
     * @return Configuration name (e.g., "DECODE_Robot_Config" or "DECODE_Robot_Config19429")
     */
    public String getConfigName() {
        return configName;
    }

    /**
     * Gets the Control Hub serial number.
     * This is unique to each Control Hub and never changes.
     *
     * @return Control Hub serial number
     */
    public String getControlHubSerial() {
        return controlHubSerial;
    }

    /**
     * Gets the Expansion Hub serial number.
     *
     * @return Expansion Hub serial number
     */
    public String getExpansionHubSerial() {
        return expansionHubSerial;
    }

    /**
     * Checks if this is the robot with config name containing the given substring.
     * Useful for quick checks like: isRobot("19429")
     *
     * @param configSubstring Substring to search for in config name
     * @return true if config name contains the substring (case-insensitive)
     */
    public boolean isRobot(String configSubstring) {
        return configName.toLowerCase().contains(configSubstring.toLowerCase());
    }

    /**
     * Gets a human-readable summary of this robot's identity.
     *
     * @return Multi-line string with robot identification info
     */
    public String getSummary() {
        return String.format("Config: %s\nControl Hub: %s\nExpansion Hub: %s",
                configName, controlHubSerial, expansionHubSerial);
    }

    /**
     * Gets a short identifier for this robot (just the config name).
     *
     * @return Short identifier string
     */
    @Override
    public String toString() {
        return configName;
    }
}
