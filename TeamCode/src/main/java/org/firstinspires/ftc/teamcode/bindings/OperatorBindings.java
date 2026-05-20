package org.firstinspires.ftc.teamcode.bindings;

import com.qualcomm.robotcore.hardware.Gamepad;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.DistanceBasedSpinCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.ModeAwareLaunchCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommands.PresetRangeSpinCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.IvyBindings;
import org.firstinspires.ftc.teamcode.util.LauncherMode;
import org.firstinspires.ftc.teamcode.util.LauncherRange;
import org.firstinspires.ftc.teamcode.util.MotifPattern;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * Operator gamepad bindings. Uses {@link IvyBindings} on top of raw FTC gamepad
 * polling.
 */
public class OperatorBindings {

    private static final double MOTIF_TAIL_STICK_THRESHOLD = 0.6;
    private static final double TRIGGER_THRESHOLD = 0.2;

    private final IvyBindings bindings = new IvyBindings();
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
        // Smart intake on right bumper: schedule new instance on press, reverse on release.
        // Each press creates a fresh Ivy Command with its own state holder so the
        // debounce timer starts clean.
        bindings.when(() -> gp().right_bumper)
                .onTrue(com.pedropathing.ivy.commands.Commands.lazy(() -> robot.intake.smartIntakeCmd(rawOperatorGamepad)))
                .onFalse(robot.intake.setIntakeModeCmd(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE));

        // Smart intake on right trigger
        bindings.when(() -> gp().right_trigger > TRIGGER_THRESHOLD)
                .onTrue(com.pedropathing.ivy.commands.Commands.lazy(() -> robot.intake.smartIntakeCmd(rawOperatorGamepad)))
                .onFalse(robot.intake.setIntakeModeCmd(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE));

        // Regular (non-smart) intake on left trigger
        bindings.when(() -> gp().left_trigger > TRIGGER_THRESHOLD)
                .onTrue(robot.intake.setIntakeModeCmd(IntakeSubsystem.IntakeMode.ACTIVE_FORWARD))
                .onFalse(robot.intake.setIntakeModeCmd(IntakeSubsystem.IntakeMode.PASSIVE_REVERSE));
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

        Optional<Integer> motifTag = robot.vision.findMotifTagId();
        if (motifTag.isPresent()) {
            MotifPattern pattern = MotifPattern.fromTagId(motifTag.get());
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
