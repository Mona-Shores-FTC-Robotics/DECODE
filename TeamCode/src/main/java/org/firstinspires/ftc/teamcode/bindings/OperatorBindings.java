package org.firstinspires.ftc.teamcode.bindings;

import com.qualcomm.robotcore.hardware.Gamepad;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.DistanceBasedSpinCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.LauncherIdleCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.ModeAwareLaunchCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.PresetRangeSpinCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.GamepadBindings;
import org.firstinspires.ftc.teamcode.util.IntakeMode;
import org.firstinspires.ftc.teamcode.util.LauncherMode;
import org.firstinspires.ftc.teamcode.util.LauncherRange;
import org.firstinspires.ftc.teamcode.util.MotifPattern;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.function.Supplier;

/**
 * Operator gamepad bindings. Uses {@link GamepadBindings} on top of raw FTC gamepad
 * polling.
 */
public class OperatorBindings {

    /** Stick deflection required to commit a motif-tail change (avoids accidental flicks). */
    private static final double MOTIF_TAIL_STICK_THRESHOLD = 0.6;
    /** A trigger is "pressed" once it crosses this fraction of full pull (0.0–1.0). */
    private static final double TRIGGER_THRESHOLD = 0.2;

    private final GamepadBindings bindings = new GamepadBindings();
    private final Supplier<Gamepad> gamepadSupplier;

    private Robot robot;

    public OperatorBindings(Supplier<Gamepad> operatorGamepadSupplier) {
        this.gamepadSupplier = operatorGamepadSupplier;
    }

    private Gamepad gp() { return gamepadSupplier.get(); }

    /**
     * Wire up bindings. Must be called after the Ivy Scheduler reset and before
     * {@link #update()} is called. The raw operator gamepad is forwarded into
     * the launcher Commands for haptic feedback inside the spin-up commands.
     */
    public void configureTeleopBindings(Robot robot, Gamepad rawOperatorGamepad) {
        this.robot = robot;

        configureGroundIntakeBindings(robot, rawOperatorGamepad);
        configureHumanIntakeBindings(robot);
        configureEjectBindings(robot);
        bindings.when(() -> true).whileTrue(LauncherIdleCommand.create(robot.launcher, robot.intake));
        configureDistanceBasedLaunchBindings(robot, rawOperatorGamepad);
        configurePresetRangeLaunchBindings(robot, rawOperatorGamepad);

        // Motif tail quick-set on left stick (left=0, up=1, right=2)
        bindings.when(this::motifTailLeftActive).onTrue(instantRunnable(() -> setMotifTailWithFeedback(robot, 0)));
        bindings.when(this::motifTailUpActive).onTrue(instantRunnable(() -> setMotifTailWithFeedback(robot, 1)));
        bindings.when(this::motifTailRightActive).onTrue(instantRunnable(() -> setMotifTailWithFeedback(robot, 2)));

        // Mode toggle (Back button)
        bindings.when(() -> gp().back).onTrue(instantRunnable(this::toggleLauncherMode));

        // Detect motif from vision (dpad right)
        bindings.when(() -> gp().dpad_right).onTrue(instantRunnable(this::tryDetectMotif));
    }

    /** Poll bindings each loop. Schedules commands on rising / falling edges. */
    public void update() {
        bindings.update();
    }

    private void configureGroundIntakeBindings(Robot robot, Gamepad rawOperatorGamepad) {
        // Smart intake on right bumper / right trigger. whileTrue (not onTrue)
        // because smartIntakeCmd is an infinite Command — without an explicit
        // cancel on release it keeps running in parallel with the onFalse
        // PASSIVE_REVERSE and stomps the mode back to ACTIVE_FORWARD on its
        // next tick (state machine sees isFull() == false → INTAKING).
        // whileTrue handles the cancel automatically (see GamepadBindings.java).
        // Each press still gets a fresh state holder via the lazy wrapper.
        bindings.when(() -> gp().right_bumper)
                .whileTrue(com.pedropathing.ivy.commands.Commands.lazy(() -> robot.intake.smartIntakeCmd(rawOperatorGamepad)))
                .onFalse(robot.intake.setIntakeModeCmd(IntakeMode.PASSIVE_REVERSE));

        bindings.when(() -> gp().right_trigger > TRIGGER_THRESHOLD)
                .whileTrue(com.pedropathing.ivy.commands.Commands.lazy(() -> robot.intake.smartIntakeCmd(rawOperatorGamepad)))
                .onFalse(robot.intake.setIntakeModeCmd(IntakeMode.PASSIVE_REVERSE));

        // Regular (non-smart) intake on left trigger
        bindings.when(() -> gp().left_trigger > TRIGGER_THRESHOLD)
                .onTrue(robot.intake.setIntakeModeCmd(IntakeMode.ACTIVE_FORWARD))
                .onFalse(robot.intake.setIntakeModeCmd(IntakeMode.PASSIVE_REVERSE));
    }

    private void configureHumanIntakeBindings(Robot robot) {
        // Human Loading (triangle held)
        // NOTE: Hood retraction is speed-gated per-lane in LauncherSubsystem.periodic().
        bindings.when(() -> gp().triangle)
                .onTrue(instantRunnable(() -> {
                    robot.launcher.runReverseFlywheelForHumanLoading();
                    robot.intake.setGateReverseConfig();
                    robot.intake.startForward();
                    robot.intake.deactivateRoller();
                }))
                .onFalse(instantRunnable(() -> {
                    robot.intake.startReverseIntakeMotor();
                    robot.launcher.stopReverseFlywheelForHumanLoading();
                    robot.intake.setGatePreventArtifact();
                    robot.launcher.setAllHoodsExtended();
                    robot.intake.forwardRoller();
                }));
    }

    private void configureEjectBindings(Robot robot) {
        // Eject button (square held): reverse intake + reverse roller; restore on release.
        bindings.when(() -> gp().square)
                .onTrue(instantRunnable(() -> {
                    robot.intake.startEject();
                    robot.intake.reverseRoller();
                }))
                .onFalse(instantRunnable(() -> {
                    robot.intake.startReverseIntakeMotor();
                    robot.intake.forwardRoller();
                }));
    }

    private void configureDistanceBasedLaunchBindings(Robot robot, Gamepad rawOperatorGamepad) {
        // Cross button: hold to distance-based spin-up; release to fire all and relocalize.
        bindings.when(() -> gp().cross)
                .whileTrue(DistanceBasedSpinCommand.create(
                        robot.launcher, robot.vision, robot.drive, robot.lighting, rawOperatorGamepad))
                .onFalse(ModeAwareLaunchCommand.create(robot.launcher, robot.intake, true)
                        .then(instantRunnable(robot.drive::tryRelocalizeForShot)));
    }

    private void configurePresetRangeLaunchBindings(Robot robot, Gamepad rawOperatorGamepad) {
        // Preset range buttons. Each holds to spin up, releases to fire and relocalize.
        bindings.when(() -> gp().dpad_down)
                .whileTrue(PresetRangeSpinCommand.create(
                        robot.launcher, LauncherRange.SHORT, false,
                        robot.drive, robot.lighting, rawOperatorGamepad))
                .onFalse(ModeAwareLaunchCommand.create(robot.launcher, robot.intake, true)
                        .then(instantRunnable(robot.drive::tryRelocalizeForShot)));

        bindings.when(() -> gp().dpad_left)
                .whileTrue(PresetRangeSpinCommand.create(
                        robot.launcher, LauncherRange.MID, false,
                        robot.drive, robot.lighting, rawOperatorGamepad))
                .onFalse(ModeAwareLaunchCommand.create(robot.launcher, robot.intake, true)
                        .then(instantRunnable(robot.drive::tryRelocalizeForShot)));

        bindings.when(() -> gp().dpad_up)
                .whileTrue(PresetRangeSpinCommand.create(
                        robot.launcher, LauncherRange.LONG, false,
                        robot.drive, robot.lighting, rawOperatorGamepad))
                .onFalse(ModeAwareLaunchCommand.create(robot.launcher, robot.intake, true)
                        .then(instantRunnable(robot.drive::tryRelocalizeForShot)));
    }

    private boolean motifTailLeftActive() {
        return -gp().left_stick_x > MOTIF_TAIL_STICK_THRESHOLD;
    }
    private boolean motifTailRightActive() {
        return gp().left_stick_x > MOTIF_TAIL_STICK_THRESHOLD;
    }
    private boolean motifTailUpActive() {
        // FTC left_stick_y is inverted (up = negative)
        return -gp().left_stick_y > MOTIF_TAIL_STICK_THRESHOLD;
    }

    private static com.pedropathing.ivy.Command instantRunnable(Runnable r) {
        return com.pedropathing.ivy.commands.Commands.instant(r);
    }

    private void setMotifTailWithFeedback(Robot robot, int value) {
        RobotState.setMotifTail(value);
        if (robot != null && robot.lighting != null) {
            robot.lighting.showMotifTailFeedback(value);
        }
    }

    private void toggleLauncherMode() {
        LauncherMode current = RobotState.getLauncherMode();
        LauncherMode newMode = (current == LauncherMode.THROUGHPUT)
                ? LauncherMode.DECODE
                : LauncherMode.THROUGHPUT;
        RobotState.setLauncherMode(newMode);
    }

    private void tryDetectMotif() {
        if (robot == null || robot.vision == null) {
            return;
        }

        Integer motifTag = robot.vision.findMotifTagId();
        if (motifTag != null) {
            MotifPattern pattern = MotifPattern.fromTagId(motifTag);
            RobotState.setMotif(pattern);

            if (robot.lighting != null) {
                robot.lighting.showMotifPattern(pattern.getLaneColors());
            }

            Gamepad raw = gp();
            if (raw != null) {
                raw.rumble(200);
            }
        }
    }

    /**
     * Explains the launcher modes and motif concept for quick reference.
     * Show this alongside controlsSummary() to give operators the full picture.
     */
    public static String[] conceptsSummary() {
        return new String[]{
                "--- LAUNCHER MODES ---",
                "THROUGHPUT: fire all lanes fast, no color order (early match)",
                "DECODE: fire in motif color order for bonus points (endgame)",
                "Auto-switches to DECODE at ~50 s remaining; Back button toggles manually",
                "",
                "--- MOTIF (endgame pattern) ---",
                "Vision reads AprilTags 21-23 to find today's color pattern (GPP / PGP / PPG)",
                "D-pad Right: detect motif from vision",
                "Left stick: set motif tail (0=none, 1=one artifact in ramp, 2=two in ramp)",
                "Tail corrects for artifacts already sitting in the scoring ramp"
        };
    }

    /** Human-readable control summary for driver station telemetry. */
    public static String[] controlsSummary() {
        return new String[]{
                "Right bumper: Smart ground intake (hold)",
                "Right trigger: Smart ground intake (hold)",
                "Left trigger: Regular ground intake (hold)",
                "Triangle: Human loading (hold)",
                "Square: Eject (hold)",
                "Cross: Distance-based spin (hold), release to launch all",
                "D-pad Down: Preset SHORT spin (hold), release to launch all",
                "D-pad Left: Preset MID spin (hold), release to launch all",
                "D-pad Up: Preset FAR spin (hold), release to launch all",
                "D-pad Right: Detect motif from vision (press)",
                "Left stick: Motif tail quick set (left=0, up=1, right=2)",
                "Back: Toggle launcher mode"
        };
    }
}
