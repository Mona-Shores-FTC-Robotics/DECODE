package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceSelector;
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
        public static double firstLimitPosition = 0.25;
        public static double secondLimitPosition = 0.75;
    }

    private enum TargetState {
        FIRST_LIMIT,
        SECOND_LIMIT;

        String displayName() {
            return this == FIRST_LIMIT ? "First limit" : "Second limit";
        }
    }

    private ShooterSubsystem shooter;
    private LightingSubsystem lighting;
    private AllianceSelector allianceSelector;
    private Alliance selectedAlliance = Alliance.BLUE;

    private Servo leftFeeder;
    private Servo centerFeeder;
    private Servo rightFeeder;
    private Servo leftIndicator;
    private Servo centerIndicator;
    private Servo rightIndicator;

    private TargetState currentState = TargetState.FIRST_LIMIT;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    @Override
    public void runOpMode() {
        shooter = new ShooterSubsystem(hardwareMap);
        lighting = new LightingSubsystem(hardwareMap);
        bindHardware(hardwareMap);

        shooter.initialize();
        lighting.initialize();
        lighting.clearLaneColors();

        BindingManager.reset();
        GamepadEx driverPad = new GamepadEx(() -> gamepad1);
        allianceSelector = new AllianceSelector(driverPad, Alliance.BLUE);
        allianceSelector.applySelection(null, lighting);
        selectedAlliance = allianceSelector.getSelectedAlliance();
        lighting.indicateAllianceInit();

        telemetry.setAutoClear(false);
        telemetry.clearAll();

        while (!isStarted() && !isStopRequested()) {
            BindingManager.update();
            allianceSelector.updateFromVision(null);

            Alliance currentAlliance = allianceSelector.getSelectedAlliance();
            if (currentAlliance != selectedAlliance) {
                selectedAlliance = currentAlliance;
                allianceSelector.applySelection(null, lighting);
                lighting.indicateAllianceInit();
            }

            telemetry.clearAll();
            telemetry.addData("Alliance", selectedAlliance);
            telemetry.addLine("D-pad Left = Blue, Right = Red, Down = Auto");
            telemetry.addLine("Press START to begin diagnostic");
            telemetry.update();

            idle();
        }

        BindingManager.reset();

        waitForStart();

        if (isStopRequested()) {
            lighting.disable();
            return;
        }

        allianceSelector.lockSelection();
        allianceSelector.applySelection(null, lighting);
        lighting.indicateIdle();

        telemetry.clearAll();
        telemetry.addLine("Ready: RB -> first limit, LB -> second limit");
        telemetry.update();

        applyCurrentState();
        pushTelemetry();

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

    private void bindHardware(HardwareMap hardwareMap) {
        leftFeeder = tryGetServo(hardwareMap, ShooterSubsystem.LeftFeederConfig.servoName);
        centerFeeder = tryGetServo(hardwareMap, ShooterSubsystem.CenterFeederConfig.servoName);
        rightFeeder = tryGetServo(hardwareMap, ShooterSubsystem.RightFeederConfig.servoName);

        leftIndicator = tryGetServo(hardwareMap, LightingSubsystem.leftServoName);
        centerIndicator = tryGetServo(hardwareMap, LightingSubsystem.centerServoName);
        rightIndicator = tryGetServo(hardwareMap, LightingSubsystem.rightServoName);

        forceStandardMode(leftFeeder);
        forceStandardMode(centerFeeder);
        forceStandardMode(rightFeeder);
        forceStandardMode(leftIndicator);
        forceStandardMode(centerIndicator);
        forceStandardMode(rightIndicator);
    }

    private void applyCurrentState() {
        double position = resolveTargetPosition();
        ArtifactColor color = resolveTargetColor();

        setFeederPosition(leftFeeder, position);
        setFeederPosition(centerFeeder, position);
        setFeederPosition(rightFeeder, position);

        lighting.setLaneColor(LauncherLane.LEFT, color);
        lighting.setLaneColor(LauncherLane.CENTER, color);
        lighting.setLaneColor(LauncherLane.RIGHT, color);
        applyIndicatorColor(leftIndicator, color);
        applyIndicatorColor(centerIndicator, color);
        applyIndicatorColor(rightIndicator, color);
    }

    private void refreshCurrentState() {
        double position = resolveTargetPosition();
        setFeederPosition(leftFeeder, position);
        setFeederPosition(centerFeeder, position);
        setFeederPosition(rightFeeder, position);
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
        telemetry.addLine(formatLaneStatus("LEFT", ShooterSubsystem.LeftFeederConfig.servoName,
                leftFeeder, LightingSubsystem.leftServoName, leftIndicator));
        telemetry.addLine(formatLaneStatus("CENTER", ShooterSubsystem.CenterFeederConfig.servoName,
                centerFeeder, LightingSubsystem.centerServoName, centerIndicator));
        telemetry.addLine(formatLaneStatus("RIGHT", ShooterSubsystem.RightFeederConfig.servoName,
                rightFeeder, LightingSubsystem.rightServoName, rightIndicator));
        telemetry.update();

    }

    private static Servo tryGetServo(HardwareMap hardwareMap, String name) {
        if (name == null || name.isEmpty()) {
            return null;
        }
        try {
            return hardwareMap.get(Servo.class, name);
        } catch (IllegalArgumentException ignored) {
            return null;
        }
    }

    private static void setFeederPosition(Servo servo, double position) {
        if (servo == null) {
            return;
        }
        servo.setPosition(Range.clip(position, 0.0, 1.0));
    }

    private void applyIndicatorColor(Servo servo, ArtifactColor color) {
        if (servo == null) {
            return;
        }
        double target;
        switch (color) {
            case GREEN:
                target = LightingSubsystem.GREEN_POS;
                break;
            case PURPLE:
                target = LightingSubsystem.PURPLE_POS;
                break;
            case NONE:
            case UNKNOWN:
            default:
                target = LightingSubsystem.OFF_POS;
                break;
        }
        servo.setPosition(Range.clip(target, 0.0, 1.0));
    }

    private static String formatLaneStatus(String label, String feederName, Servo feeder,
                                           String indicatorName, Servo indicator) {
        String feederStatus = feeder == null
                ? "feeder missing (" + safeName(feederName) + ")"
                : String.format("feeder %.3f (%s)", feeder.getPosition(), safeName(feederName));
        String lightStatus = indicator == null
                ? "light missing (" + safeName(indicatorName) + ")"
                : String.format("light %.3f (%s)", indicator.getPosition(), safeName(indicatorName));
        return label + ": " + feederStatus + "; " + lightStatus;
    }

    private static String safeName(String name) {
        return (name == null || name.isEmpty()) ? "unset" : name;
    }

    private static void forceStandardMode(Servo servo) {
        if (servo == null) {
            return;
        }
        ServoController controller = servo.getController();
        if (controller instanceof ServoControllerEx) {
            ServoControllerEx controllerEx = (ServoControllerEx) controller;
            controllerEx.setServoType(
                    servo.getPortNumber(),
                    ServoConfigurationType.getStandardServoType()
            );
        } else {
            try {
                ServoConfigurationType standard = ServoConfigurationType.getStandardServoType();
                java.lang.reflect.Method method = controller.getClass().getMethod(
                        "setServoType", int.class, ServoConfigurationType.class);
                method.invoke(controller, servo.getPortNumber(), standard);
            } catch (ReflectiveOperationException ignored) {
                // No supported API – leave mode unchanged.
            }
        }
    }
}
