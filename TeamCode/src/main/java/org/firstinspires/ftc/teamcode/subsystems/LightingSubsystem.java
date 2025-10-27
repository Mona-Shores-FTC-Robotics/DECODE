package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.auto.Alliance;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

import java.util.Arrays;
import java.util.EnumMap;

/**
 * Lighting controller that keeps the alliance indicator and three launcher-lane lights in sync
 * with the intake sensors and autonomous decode pattern.
 */
@Configurable
public class LightingSubsystem implements Subsystem, IntakeSubsystem.LaneColorListener {

    private enum LightingMode {
        DISABLED,
        ALLIANCE_INIT,
        LANE_STATUS,
        INTAKE_BUSY,
        DECODE_PATTERN
    }

    @Configurable
    public static class AllianceIndicatorConfig {
        public static String servoName = "indicator";
        public static double offPosition = 0.50;
        public static double blueAlliancePosition = 0.61;
        public static double redAlliancePosition = 0.28;
        public static double unknownAlliancePosition = 0.50;
        public static double busyPosition = 0.72;
    }

    @Configurable
    public static class LaneHardwareConfig {
        public static String leftServo = "lane_left_led";
        public static String centerServo = "lane_center_led";
        public static String rightServo = "lane_right_led";
    }

    @Configurable
    public static class LanePaletteConfig {
        public static double offPosition = 0.05;
        public static double artifactGreenPosition = 0.50;
        public static double artifactPurplePosition = 0.72;
        public static double busyPosition = 0.16;
        public static double allianceBluePosition = 0.61;
        public static double allianceRedPosition = 0.28;
    }

    private final AllianceIndicator allianceIndicator;
    private final EnumMap<LauncherLane, LaneLight> laneLights = new EnumMap<>(LauncherLane.class);
    private final EnumMap<LauncherLane, ArtifactColor> laneColors = new EnumMap<>(LauncherLane.class);

    private LightingMode mode = LightingMode.DISABLED;
    private Alliance alliance = Alliance.UNKNOWN;
    private ArtifactColor[] decodePattern = new ArtifactColor[0];

    public LightingSubsystem(HardwareMap hardwareMap) {
        allianceIndicator = AllianceIndicator.fromHardware(hardwareMap);
        laneLights.put(LauncherLane.LEFT, LaneLight.fromHardware(hardwareMap, LaneHardwareConfig.leftServo));
        laneLights.put(LauncherLane.CENTER, LaneLight.fromHardware(hardwareMap, LaneHardwareConfig.centerServo));
        laneLights.put(LauncherLane.RIGHT, LaneLight.fromHardware(hardwareMap, LaneHardwareConfig.rightServo));
        for (LauncherLane lane : LauncherLane.values()) {
            laneColors.put(lane, ArtifactColor.NONE);
        }
    }

    @Override
    public void initialize() {
        decodePattern = new ArtifactColor[0];
        indicateAllianceInit();
    }

    @Override
    public void periodic() {
        // Reserved for future animations (blink, chase, etc).
    }

    public void disable() {
        mode = LightingMode.DISABLED;
        decodePattern = new ArtifactColor[0];
        allianceIndicator.off();
        for (LaneLight light : laneLights.values()) {
            light.off();
        }
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance == null ? Alliance.UNKNOWN : alliance;
        applyIndicatorForCurrentMode();
    }

    public void indicateAllianceInit() {
        mode = LightingMode.ALLIANCE_INIT;
        decodePattern = new ArtifactColor[0];
        applyIndicatorForCurrentMode();
        refreshLaneDisplays();
    }

    public void indicateIdle() {
        mode = LightingMode.LANE_STATUS;
        decodePattern = new ArtifactColor[0];
        applyIndicatorForCurrentMode();
        refreshLaneDisplays();
    }

    public void indicateBusy() {
        mode = LightingMode.INTAKE_BUSY;
        decodePattern = new ArtifactColor[0];
        applyIndicatorForCurrentMode();
        for (LaneLight light : laneLights.values()) {
            light.busy();
        }
    }

    public void showDecodePattern(ArtifactColor[] pattern) {
        if (pattern == null || pattern.length == 0) {
            clearPattern();
            return;
        }
        decodePattern = Arrays.copyOf(pattern, LauncherLane.values().length);
        mode = LightingMode.DECODE_PATTERN;
        applyIndicatorForCurrentMode();
        refreshLaneDisplays();
    }

    public void clearPattern() {
        decodePattern = new ArtifactColor[0];
        indicateIdle();
    }

    public ArtifactColor getLaneColor(LauncherLane lane) {
        return lane == null ? ArtifactColor.NONE : laneColors.getOrDefault(lane, ArtifactColor.NONE);
    }

    public void setLaneColor(LauncherLane lane, ArtifactColor color) {
        if (lane == null) {
            return;
        }
        ArtifactColor resolved = color == null ? ArtifactColor.NONE : color;
        laneColors.put(lane, resolved);
        if (mode == LightingMode.LANE_STATUS) {
            applyLaneDisplay(lane);
        }
    }

    @Override
    public void onLaneColorChanged(LauncherLane lane, ArtifactColor color) {
        setLaneColor(lane, color);
    }

    private void refreshLaneDisplays() {
        for (LauncherLane lane : LauncherLane.values()) {
            applyLaneDisplay(lane);
        }
    }

    private void applyLaneDisplay(LauncherLane lane) {
        LaneLight light = laneLights.get(lane);
        if (light == null) {
            return;
        }

        if (mode == LightingMode.ALLIANCE_INIT) {
            if (alliance == Alliance.BLUE) {
                light.allianceBlue();
            } else if (alliance == Alliance.RED) {
                light.allianceRed();
            } else {
                light.off();
            }
            return;
        }

        if (mode == LightingMode.INTAKE_BUSY) {
            light.busy();
            return;
        }

        ArtifactColor displayColour = laneColors.getOrDefault(lane, ArtifactColor.NONE);

        if (mode == LightingMode.DECODE_PATTERN && lane.ordinal() < decodePattern.length) {
            ArtifactColor patternColour = decodePattern[lane.ordinal()];
            if (patternColour != null && patternColour != ArtifactColor.NONE) {
                displayColour = patternColour;
            }
        }

        switch (displayColour) {
            case GREEN:
                light.green();
                break;
            case PURPLE:
                light.purple();
                break;
            case UNKNOWN:
            case NONE:
            default:
                light.off();
                break;
        }
    }

    private void applyIndicatorForCurrentMode() {
        if (mode == LightingMode.INTAKE_BUSY) {
            allianceIndicator.busy();
            return;
        }
        allianceIndicator.showAlliance(alliance);
    }

    private static double clamp01(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }

    private static final class AllianceIndicator {
        private final Servo servo;

        static AllianceIndicator fromHardware(HardwareMap hardwareMap) {
            Servo servo = null;
            try {
                servo = hardwareMap.get(Servo.class, AllianceIndicatorConfig.servoName);
            } catch (IllegalArgumentException ignored) {
                // Optional hardware.
            }
            return new AllianceIndicator(servo);
        }

        AllianceIndicator(Servo servo) {
            this.servo = servo;
        }

        void showAlliance(Alliance alliance) {
            if (servo == null) {
                return;
            }
            double position;
            switch (alliance) {
                case RED:
                    position = AllianceIndicatorConfig.redAlliancePosition;
                    break;
                case BLUE:
                    position = AllianceIndicatorConfig.blueAlliancePosition;
                    break;
                case UNKNOWN:
                default:
                    position = AllianceIndicatorConfig.unknownAlliancePosition;
                    break;
            }
            servo.setPosition(clamp01(position));
        }

        void busy() {
            if (servo == null) {
                return;
            }
            servo.setPosition(clamp01(AllianceIndicatorConfig.busyPosition));
        }

        void off() {
            if (servo == null) {
                return;
            }
            servo.setPosition(clamp01(AllianceIndicatorConfig.offPosition));
        }
    }

    private static final class LaneLight {
        private final Servo servo;

        static LaneLight fromHardware(HardwareMap hardwareMap, String servoName) {
            Servo servo = null;
            if (servoName != null && !servoName.isEmpty()) {
                try {
                    servo = hardwareMap.get(Servo.class, servoName);
                } catch (IllegalArgumentException ignored) {
                    // Optional hardware.
                }
            }
            return new LaneLight(servo);
        }

        LaneLight(Servo servo) {
            this.servo = servo;
        }

        void green() {
            setPosition(LanePaletteConfig.artifactGreenPosition);
        }

        void purple() {
            setPosition(LanePaletteConfig.artifactPurplePosition);
        }

        void allianceBlue() {
            setPosition(LanePaletteConfig.allianceBluePosition);
        }

        void allianceRed() {
            setPosition(LanePaletteConfig.allianceRedPosition);
        }

        void busy() {
            setPosition(LanePaletteConfig.busyPosition);
        }

        void off() {
            setPosition(LanePaletteConfig.offPosition);
        }

        private void setPosition(double position) {
            if (servo == null) {
                return;
            }
            servo.setPosition(clamp01(position));
        }
    }
}
