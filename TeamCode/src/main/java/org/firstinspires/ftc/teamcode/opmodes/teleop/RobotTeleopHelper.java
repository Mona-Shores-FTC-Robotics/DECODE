package org.firstinspires.ftc.teamcode.opmodes.teleop;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Provides optional on-stick help overlays for the robot-centric TeleOp.
 * All button wiring is centralised here so the OpMode loop stays focused on behaviour.
 */
public class RobotTeleopHelper {

    private static final class HelpEntry {
        final String line;

        HelpEntry(String line) {
            this.line = line;
        }
    }

    private final List<HelpEntry> driverHelp = new ArrayList<>();
    private final List<HelpEntry> operatorHelp = new ArrayList<>();
    private boolean showDriverHelp = false;
    private boolean showOperatorHelp = false;

    public RobotTeleopHelper(GamepadEx driver, GamepadEx operator) {
        driverHelp.add(new HelpEntry("Driver: Left stick = robot X/Y, Right stick = rotation"));
        driverHelp.add(new HelpEntry("Driver RB: Slow mode, LB: Normal mode"));
        driverHelp.add(new HelpEntry("Driver B: Flywheel low, X: Flywheel high, A: Stop"));

        operatorHelp.add(new HelpEntry("Operator: customise bindings in RobotTeleopHelper"));

        Button driverHelpToggle = driver.options().toggleOnBecomesTrue();
        driverHelpToggle.whenBecomesTrue(() -> showDriverHelp = true);
        driverHelpToggle.whenBecomesFalse(() -> showDriverHelp = false);

        Button operatorHelpToggle = operator.options().toggleOnBecomesTrue();
        operatorHelpToggle.whenBecomesTrue(() -> showOperatorHelp = true);
        operatorHelpToggle.whenBecomesFalse(() -> showOperatorHelp = false);
    }

    public void publishHelp(Telemetry telemetry) {
        if (showDriverHelp) {
            driverHelp.forEach(entry -> telemetry.addLine(entry.line));
        }
        if (showOperatorHelp) {
            operatorHelp.forEach(entry -> telemetry.addLine(entry.line));
        }
    }

    public void reset() {
        showDriverHelp = false;
        showOperatorHelp = false;
    }
}
