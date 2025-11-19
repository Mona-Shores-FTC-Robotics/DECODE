package org.firstinspires.ftc.teamcode.util;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.LightingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystemLimelight;
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
    private final Button defaultButton;

    private boolean manualOverride;
    private boolean visionPreferred;
    private boolean selectionLocked;
    private Alliance manualAlliance = Alliance.UNKNOWN;

    private Alliance detectedAlliance = Alliance.UNKNOWN;
    private Alliance selectedAlliance;
    private int detectedTagId = -1;
    private double detectedRange = Double.NaN;
    private double detectedYaw = Double.NaN;
    private VisionSubsystemLimelight.TagSnapshot lastSnapshot;

    public AllianceSelector(GamepadEx driver, Alliance defaultAlliance) {
        this.defaultAlliance = defaultAlliance;
        this.selectedAlliance = defaultAlliance == null ? Alliance.UNKNOWN : defaultAlliance;
        this.visionPreferred = (defaultAlliance == null || defaultAlliance == Alliance.UNKNOWN);

        blueOverrideButton = driver.dpadLeft();
        redOverrideButton = driver.dpadRight();
        autoOverrideButton = driver.dpadDown();
        defaultButton = driver.dpadUp();

        blueOverrideButton.whenBecomesTrue(() -> selectAllianceManually(Alliance.BLUE));
        redOverrideButton.whenBecomesTrue(() -> selectAllianceManually(Alliance.RED));
        autoOverrideButton.whenBecomesTrue(this::enableVisionOverride);
        defaultButton.whenBecomesTrue(this::clearManualOverride);
    }

    /**
     * Polls the vision subsystem for an AprilTag snapshot and updates the selected alliance.
     *
     * @return snapshot data when a valid detection was found.
     */
    public Optional<VisionSubsystemLimelight.TagSnapshot> updateFromVision(VisionSubsystemLimelight vision) {
        if (vision == null) {
            clearDetection();
            return Optional.empty();
        }

        Optional<VisionSubsystemLimelight.TagSnapshot> snapshotOpt = vision.findAllianceSnapshot(null);
        if (!snapshotOpt.isPresent()) {
            clearDetection();
        } else {
            VisionSubsystemLimelight.TagSnapshot snapshot = snapshotOpt.get();
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
        visionPreferred = false;
        manualAlliance = alliance == null ? Alliance.UNKNOWN : alliance;
        selectedAlliance = manualAlliance;
    }

    public void clearManualOverride() {
        if (selectionLocked) {
            return;
        }
        manualOverride = false;
        visionPreferred = false;
        manualAlliance = Alliance.UNKNOWN;
        refreshSelectedAlliance();
    }

    public void enableVisionOverride() {
        if (selectionLocked) {
            return;
        }
        manualOverride = false;
        visionPreferred = true;
        refreshSelectedAlliance();
    }

    private void refreshSelectedAlliance() {
        if (selectionLocked) {
            return;
        }
        if (manualOverride) {
            selectedAlliance = manualAlliance;
        } else if (visionPreferred && detectedAlliance != Alliance.UNKNOWN) {
            selectedAlliance = detectedAlliance;
        } else {
            selectedAlliance = defaultAlliance;
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

    public Optional<VisionSubsystemLimelight.TagSnapshot> getLastSnapshot() {
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

    public boolean isSelectionLocked() {
        return selectionLocked;
    }

    public void unlockSelection() {
        selectionLocked = false;
        refreshSelectedAlliance();
    }

    private void clearDetection() {
        detectedAlliance = Alliance.UNKNOWN;
        detectedTagId = -1;
        detectedRange = Double.NaN;
        detectedYaw = Double.NaN;
        lastSnapshot = null;
    }

    public boolean isVisionPreferred() {
        return visionPreferred;
    }
}
