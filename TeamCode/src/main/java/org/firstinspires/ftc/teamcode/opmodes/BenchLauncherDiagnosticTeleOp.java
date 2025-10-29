package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.PanelsBridge;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.LauncherLane;

/**
 * Diagnostic driver-control OpMode for exercising the bench launcher servos and paired lane lights.
 * Right bumper commands the configured "first" limit, left bumper commands the "second" limit.
 */
@Config
@TeleOp(name = "Bench Launcher Diagnostic", group = "Diagnostics")
public class BenchLauncherDiagnosticTeleOp extends LinearOpMode {

    @Config
    public static class BenchConfig {
        public static String leftServoName = ShooterSubsystem.LeftFeederConfig.servoName;
        public static String centerServoName = ShooterSubsystem.CenterFeederConfig.servoName;
        public static String rightServoName = ShooterSubsystem.RightFeederConfig.servoName;
        public static double firstLimitPosition = 0.25;
        public static double secondLimitPosition = 0.75;
    }

    private enum TargetState {
        FIRST_LIMIT,
        SECOND_LIMIT;

        String displayName() {
            switch (this) {
                case FIRST_LIMIT:
                    return "First limit";
                case SECOND_LIMIT:
                default:
                    return "Second limit";
            }
        }
    }

    private TelemetryManager panelsTelemetry;
    private LaneActuator leftActuator;
    private LaneActuator centerActuator;
    private LaneActuator rightActuator;
    private LaneLight leftLight;
    private LaneLight centerLight;
    private LaneLight rightLight;
    private TargetState currentState = TargetState.FIRST_LIMIT;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsBridge.preparePanels();

        leftActuator = new LaneActuator(LauncherLane.LEFT, BenchConfig.leftServoName, hardwareMap);
        centerActuator = new LaneActuator(LauncherLane.CENTER, BenchConfig.centerServoName, hardwareMap);
        rightActuator = new LaneActuator(LauncherLane.RIGHT, BenchConfig.rightServoName, hardwareMap);

        leftLight = new LaneLight(LauncherLane.LEFT, resolveLightName(LightingSubsystem.leftServoName), hardwareMap);
        centerLight = new LaneLight(LauncherLane.CENTER, resolveLightName(LightingSubsystem.centerServoName), hardwareMap);
        rightLight = new LaneLight(LauncherLane.RIGHT, resolveLightName(LightingSubsystem.rightServoName), hardwareMap);

        telemetry.setAutoClear(false);
        telemetry.clearAll();
        applyCurrentState();
        pushTelemetry();

        telemetry.addLine("Ready: RB -> first limit, LB -> second limit");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            disableLights();
            return;
        }

        telemetry.clearAll();

        while (opModeIsActive()) {
            boolean rb = gamepad1.right_bumper;
            boolean lb = gamepad1.left_bumper;

            if (rb && !lastRightBumper) {
                currentState = TargetState.FIRST_LIMIT;
                applyCurrentState();
            } else if (lb && !lastLeftBumper) {
                currentState = TargetState.SECOND_LIMIT;
                applyCurrentState();
            } else {
                refreshCurrentState(); // pick up dashboard edits
            }

            lastRightBumper = rb;
            lastLeftBumper = lb;

            pushTelemetry();
            idle();
        }

        disableLights();
    }

    private void applyCurrentState() {
        double position = resolveTargetPosition();
        ArtifactColor color = resolveTargetColor();

        leftActuator.apply(position);
        centerActuator.apply(position);
        rightActuator.apply(position);

        leftLight.apply(color);
        centerLight.apply(color);
        rightLight.apply(color);
    }

    private void refreshCurrentState() {
        double position = resolveTargetPosition();
        leftActuator.apply(position);
        centerActuator.apply(position);
        rightActuator.apply(position);
    }

    private double resolveTargetPosition() {
        double first = Range.clip(BenchConfig.firstLimitPosition, 0.0, 1.0);
        double second = Range.clip(BenchConfig.secondLimitPosition, 0.0, 1.0);
        return currentState == TargetState.FIRST_LIMIT ? first : second;
    }

    private ArtifactColor resolveTargetColor() {
        return currentState == TargetState.FIRST_LIMIT ? ArtifactColor.PURPLE : ArtifactColor.GREEN;
    }

    private void pushTelemetry() {
        telemetry.clearAll();
        double first = Range.clip(BenchConfig.firstLimitPosition, 0.0, 1.0);
        double second = Range.clip(BenchConfig.secondLimitPosition, 0.0, 1.0);

        telemetry.addData("State", currentState.displayName());
        telemetry.addData("First limit", "%.2f", first);
        telemetry.addData("Second limit", "%.2f", second);
        telemetry.addLine(formatLaneStatus(leftActuator, leftLight));
        telemetry.addLine(formatLaneStatus(centerActuator, centerLight));
        telemetry.addLine(formatLaneStatus(rightActuator, rightLight));
        telemetry.update();

        if (panelsTelemetry != null) {
            panelsTelemetry.debug("Bench state", currentState.displayName());
            panelsTelemetry.debug("First limit", String.format("%.2f", first));
            panelsTelemetry.debug("Second limit", String.format("%.2f", second));
            panelsTelemetry.update(telemetry);
        }
    }

    private static String formatLaneStatus(LaneActuator actuator, LaneLight light) {
        String servoStatus;
        if (!actuator.isPresent()) {
            servoStatus = "servo missing (" + actuator.getName() + ")";
        } else {
            servoStatus = actuator.getServoPositionSummary() + " (" + actuator.getName() + ")";
        }

        String lightStatus;
        if (!light.isPresent()) {
            lightStatus = "light missing (" + light.getName() + ")";
        } else {
            lightStatus = light.getPositionSummary();
        }

        return actuator.getLane().name() + ": " + servoStatus + "; " + lightStatus;
    }

    private static String resolveLightName(String preferred) {
        if (preferred != null && !preferred.isEmpty()) {
            return preferred;
        }
        return LightingSubsystem.sharedServoName;
    }

    private void disableLights() {
        leftLight.apply(ArtifactColor.NONE);
        centerLight.apply(ArtifactColor.NONE);
        rightLight.apply(ArtifactColor.NONE);
    }

    private static final class LaneActuator {
        private final LauncherLane lane;
        private final String name;
        private final Servo servo;

        LaneActuator(LauncherLane lane, String servoName, HardwareMap hardwareMap) {
            this.lane = lane;
            this.name = servoName == null ? "" : servoName;
            Servo resolved = null;
            if (servoName != null && !servoName.isEmpty()) {
                try {
                    resolved = hardwareMap.get(Servo.class, servoName);
                } catch (IllegalArgumentException ignored) {
                    resolved = null;
                }
            }
            this.servo = resolved;
        }

        void apply(double position) {
            if (servo == null) {
                return;
            }
            servo.setPosition(Range.clip(position, 0.0, 1.0));
        }

        boolean isPresent() {
            return servo != null;
        }

        LauncherLane getLane() {
            return lane;
        }

        String getName() {
            return name.isEmpty() ? "(unset)" : name;
        }

        String getServoPositionSummary() {
            if (servo == null) {
                return "pos=n/a";
            }
            return String.format("pos=%.2f", servo.getPosition());
        }
    }

    private static final class LaneLight {
        private final LauncherLane lane;
        private final String name;
        private final Servo servo;

        LaneLight(LauncherLane lane, String servoName, HardwareMap hardwareMap) {
            this.lane = lane;
            this.name = servoName == null ? "" : servoName;
            this.servo = resolveServo(servoName, hardwareMap);
        }

        void apply(ArtifactColor color) {
            if (servo == null) {
                return;
            }
            servo.setPosition(resolvePosition(color));
        }

        boolean isPresent() {
            return servo != null;
        }

        LauncherLane getLane() {
            return lane;
        }

        String getName() {
            return name.isEmpty() ? "(unset)" : name;
        }

        String getPositionSummary() {
            if (servo == null) {
                return "light pos=n/a";
            }
            return String.format("light pos=%.3f", servo.getPosition());
        }

        private static Servo resolveServo(String servoName, HardwareMap hardwareMap) {
            if (servoName != null && !servoName.isEmpty()) {
                try {
                    return hardwareMap.get(Servo.class, servoName);
                } catch (IllegalArgumentException ignored) {
                    // fall through to shared lookup
                }
            }
            if (LightingSubsystem.sharedServoName != null && !LightingSubsystem.sharedServoName.isEmpty()) {
                try {
                    return hardwareMap.get(Servo.class, LightingSubsystem.sharedServoName);
                } catch (IllegalArgumentException ignored) {
                    return null;
                }
            }
            return null;
        }

        private static double resolvePosition(ArtifactColor color) {
            switch (color) {
                case GREEN:
                    return LightingSubsystem.GREEN_POS;
                case PURPLE:
                    return LightingSubsystem.PURPLE_POS;
                case NONE:
                case UNKNOWN:
                default:
                    return LightingSubsystem.OFF_POS;
            }
        }
    }
}
