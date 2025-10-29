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

    private LightingSubsystem lighting;
    private TelemetryManager panelsTelemetry;
    private LaneActuator leftActuator;
    private LaneActuator centerActuator;
    private LaneActuator rightActuator;
    private TargetState currentState = TargetState.FIRST_LIMIT;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    @Override
    public void runOpMode() {
        lighting = new LightingSubsystem(hardwareMap);
        panelsTelemetry = PanelsBridge.preparePanels();

        leftActuator = new LaneActuator(LauncherLane.LEFT, BenchConfig.leftServoName, hardwareMap);
        centerActuator = new LaneActuator(LauncherLane.CENTER, BenchConfig.centerServoName, hardwareMap);
        rightActuator = new LaneActuator(LauncherLane.RIGHT, BenchConfig.rightServoName, hardwareMap);

        lighting.initialize();
        lighting.indicateIdle();
        applyCurrentState();
        pushTelemetry();

        telemetry.addLine("Ready: RB -> first limit, LB -> second limit");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            lighting.disable();
            return;
        }

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

            lighting.periodic();
            pushTelemetry();
            idle();
        }

        lighting.disable();
    }

    private void applyCurrentState() {
        double position = resolveTargetPosition();
        ArtifactColor color = resolveTargetColor();

        leftActuator.apply(position);
        centerActuator.apply(position);
        rightActuator.apply(position);

        lighting.setLaneColor(LauncherLane.LEFT, color);
        lighting.setLaneColor(LauncherLane.CENTER, color);
        lighting.setLaneColor(LauncherLane.RIGHT, color);
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
        double first = Range.clip(BenchConfig.firstLimitPosition, 0.0, 1.0);
        double second = Range.clip(BenchConfig.secondLimitPosition, 0.0, 1.0);

        telemetry.addData("State", currentState.displayName());
        telemetry.addData("First limit", "%.2f", first);
        telemetry.addData("Second limit", "%.2f", second);
        telemetry.addLine(formatActuatorStatus(leftActuator));
        telemetry.addLine(formatActuatorStatus(centerActuator));
        telemetry.addLine(formatActuatorStatus(rightActuator));
        telemetry.update();

        if (panelsTelemetry != null) {
            panelsTelemetry.debug("Bench state", currentState.displayName());
            panelsTelemetry.debug("First limit", String.format("%.2f", first));
            panelsTelemetry.debug("Second limit", String.format("%.2f", second));
            panelsTelemetry.update(telemetry);
        }
    }

    private static String formatActuatorStatus(LaneActuator actuator) {
        if (!actuator.isPresent()) {
            return actuator.getLane().name() + ": missing (" + actuator.getName() + ")";
        }
        return actuator.getLane().name() + ": "
                + actuator.getServoPositionSummary()
                + " (" + actuator.getName() + ")";
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
                return "n/a";
            }
            return String.format("pos=%.2f", servo.getPosition());
        }
    }
}
