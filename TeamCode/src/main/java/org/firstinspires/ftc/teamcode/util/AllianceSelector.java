package org.firstinspires.ftc.teamcode.util;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotState;

import java.util.Optional;

/**
 * Coordinates alliance selection by combining AprilTag detections with manual overrides bound
 * through NextFTC. Keeps {@link RobotState}, lighting, and the robot container in sync.
 */
public final class AllianceSelector {

    private final Alliance defaultAlliance;
    private final Button blueOverrideButton;
    private final Button redOverrideButton;
    private final Button autoOverrideButton;

    private boolean manualOverride;
    private boolean selectionLocked;
    private Alliance manualAlliance = Alliance.UNKNOWN;

    private Alliance detectedAlliance = Alliance.UNKNOWN;
    private Alliance selectedAlliance;
    private int detectedTagId = -1;
    private double detectedRange = Double.NaN;
    private double detectedYaw = Double.NaN;
    private VisionSubsystem.TagSnapshot lastSnapshot;

    public AllianceSelector(GamepadEx driver, Alliance defaultAlliance) {
        this.defaultAlliance = defaultAlliance;
        this.selectedAlliance = defaultAlliance == null ? Alliance.UNKNOWN : defaultAlliance;

        blueOverrideButton = driver.dpadLeft();
        redOverrideButton = driver.dpadRight();
        autoOverrideButton = driver.dpadDown();

        blueOverrideButton.whenBecomesTrue(() -> selectAllianceManually(Alliance.BLUE));
        redOverrideButton.whenBecomesTrue(() -> selectAllianceManually(Alliance.RED));
        autoOverrideButton.whenBecomesTrue(this::clearManualOverride);
    }

    /**
     * Polls the vision subsystem for an AprilTag snapshot and updates the selected alliance.
     *
     * @return snapshot data when a valid detection was found.
     */
    public Optional<VisionSubsystem.TagSnapshot> updateFromVision(VisionSubsystem vision) {
        Alliance requiredAlliance = manualOverride ? manualAlliance : null;
        Optional<VisionSubsystem.TagSnapshot> snapshotOpt =
                vision == null ? Optional.empty() : vision.findAllianceSnapshot(requiredAlliance);

        if (!snapshotOpt.isPresent()) {
            detectedAlliance = Alliance.UNKNOWN;
            detectedTagId = -1;
            detectedRange = Double.NaN;
            detectedYaw = Double.NaN;
            lastSnapshot = null;
        } else {
            VisionSubsystem.TagSnapshot snapshot = snapshotOpt.get();
            detectedAlliance = snapshot.getAlliance();
            detectedTagId = snapshot.getTagId();
            detectedRange = snapshot.getFtcRange();
            detectedYaw = snapshot.getFtcBearing();
            lastSnapshot = snapshot;
        }

        refreshSelectedAlliance();
        return snapshotOpt;
    }

    /**
     * Applies the current selected alliance to the robot containers and lighting.
     */
    public void applySelection(Robot robot, LightingSubsystem lighting) {
        Alliance allianceToApply = selectedAlliance != Alliance.UNKNOWN ? selectedAlliance : defaultAlliance;
        if (allianceToApply == null) {
            allianceToApply = Alliance.UNKNOWN;
        }
        if (robot != null) {
            robot.setAlliance(allianceToApply);
        } else {
            RobotState.setAlliance(allianceToApply);
        }
        if (lighting != null) {
            lighting.setAlliance(allianceToApply);
        }
    }

    public void selectAllianceManually(Alliance alliance) {
        if (selectionLocked) {
            return;
        }
        manualOverride = true;
        manualAlliance = alliance == null ? Alliance.UNKNOWN : alliance;
        selectedAlliance = manualAlliance;
    }

    public void clearManualOverride() {
        if (selectionLocked) {
            return;
        }
        manualOverride = false;
        manualAlliance = Alliance.UNKNOWN;
        refreshSelectedAlliance();
    }

    private void refreshSelectedAlliance() {
        if (selectionLocked) {
            return;
        }
        if (!manualOverride) {
            if (detectedAlliance != Alliance.UNKNOWN) {
                selectedAlliance = detectedAlliance;
            } else {
                selectedAlliance = defaultAlliance;
            }
        }
        if (selectedAlliance == null) {
            selectedAlliance = Alliance.UNKNOWN;
        }
    }

    public boolean isManualOverrideActive() {
        return manualOverride;
    }

    public Alliance getManualAlliance() {
        return manualAlliance;
    }

    public Alliance getDetectedAlliance() {
        return detectedAlliance;
    }

    public Alliance getSelectedAlliance() {
        return selectedAlliance;
    }

    public Optional<VisionSubsystem.TagSnapshot> getLastSnapshot() {
        return Optional.ofNullable(lastSnapshot);
    }

    public int getDetectedTagId() {
        return detectedTagId;
    }

    public double getDetectedRange() {
        return detectedRange;
    }

    public double getDetectedYaw() {
        return detectedYaw;
    }

    public void lockSelection() {
        selectionLocked = true;
    }

    public void unlockSelection() {
        selectionLocked = false;
        refreshSelectedAlliance();
    }
}
