package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
import org.firstinspires.ftc.teamcode.util.LauncherRange;

/**
 * Thin convenience factory for launcher Commands. Each method just forwards to the
 * underlying command's static .create(...) factory. Kept primarily for backward
 * compatibility with callers (autos, Robot.launcherCommands) that pre-date the
 * Ivy port.
 *
 * After PR 5 finishes, callers can either invoke the static .create() methods
 * directly or this thin class can be deleted entirely.
 */
@SuppressWarnings("UnusedReturnValue")
public class LauncherCommands {

    public static final double DEFAULT_BURST_SPACING_MS = 300;

    private final LauncherSubsystem launcher;
    private final IntakeSubsystem intake;
    private final DriveSubsystem drive;
    private final LightingSubsystem lighting;
    private Gamepad operatorGamepad;

    public LauncherCommands(LauncherSubsystem launcher,
                            IntakeSubsystem intake,
                            DriveSubsystem drive,
                            LightingSubsystem lighting) {
        this.launcher = launcher;
        this.intake = intake;
        this.drive = drive;
        this.lighting = lighting;
    }

    public Command presetRangeSpinUp(LauncherRange range, boolean finishWhenReady) {
        return PresetRangeSpinCommand.create(launcher, range, finishWhenReady, drive, lighting, operatorGamepad);
    }

    public Command distanceBasedSpinUp(VisionSubsystemLimelight vision,
                                       DriveSubsystem drive,
                                       LightingSubsystem lighting,
                                       Gamepad gamepad) {
        return DistanceBasedSpinCommand.create(launcher, vision, drive, lighting, gamepad);
    }

    public Command launchAll(boolean spinDownAfterShot) {
        return LaunchAllCommand.create(launcher, intake, spinDownAfterShot);
    }

    public Command launchAccordingToMode(boolean spinDownAfterShot) {
        return ModeAwareLaunchCommand.create(launcher, intake, spinDownAfterShot);
    }

    public Command launchInSequence(boolean spinDownAfterShot) {
        return LaunchInSequenceCommand.create(launcher, intake, spinDownAfterShot);
    }

    public void setOperatorGamepad(Gamepad operatorGamepad) {
        this.operatorGamepad = operatorGamepad;
    }
}
