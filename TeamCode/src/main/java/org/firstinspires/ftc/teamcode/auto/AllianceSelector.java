package org.firstinspires.ftc.teamcode.auto;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.GamepadEx;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * Handles alliance selection by blending AprilTag detections with manual overrides that are bound
 * to the driver's D-pad through NextFTC.
 */
public final class AllianceSelector {

    private final Alliance defaultAlliance;
    private final int blueTagId;
    private final int redTagId;

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

    public AllianceSelector(GamepadEx driver, Alliance defaultAlliance, int blueTagId, int redTagId) {
        this.defaultAlliance = defaultAlliance;
        this.blueTagId = blueTagId;
        this.redTagId = redTagId;
        this.selectedAlliance = defaultAlliance;

        blueOverrideButton = driver.dpadLeft();
        redOverrideButton = driver.dpadRight();
        autoOverrideButton = driver.dpadDown();

        blueOverrideButton.whenBecomesTrue(() -> selectAllianceManually(Alliance.BLUE));
        redOverrideButton.whenBecomesTrue(() -> selectAllianceManually(Alliance.RED));
        autoOverrideButton.whenBecomesTrue(this::clearManualOverride);
    }

    /**
     * Updates the detection snapshot from the AprilTag processor and recomputes the active
     * alliance choice. The first detection with the highest decision margin wins.
     *
     * @param detections AprilTag detections for the current frame.
     * @return {@code true} if the selected alliance changed.
     */
    public boolean updateFromDetections(List<AprilTagDetection> detections) {
        Alliance previousSelection = selectedAlliance;

        detectedAlliance = Alliance.UNKNOWN;
        detectedTagId = -1;
        detectedRange = Double.NaN;
        detectedYaw = Double.NaN;

        double bestDecisionMargin = Double.NEGATIVE_INFINITY;

        if (detections != null) {
            for (AprilTagDetection detection : detections) {
                Alliance candidate = mapTagToAlliance(detection.id);
                if (candidate == Alliance.UNKNOWN) {
                    continue;
                }

                if (detection.decisionMargin > bestDecisionMargin) {
                    bestDecisionMargin = detection.decisionMargin;
                    detectedAlliance = candidate;
                    detectedTagId = detection.id;

                    if (detection.ftcPose != null) {
                        detectedRange = detection.ftcPose.range;
                        detectedYaw = detection.ftcPose.bearing;
                    }
                }
            }
        }

        refreshSelectedAlliance();
        return previousSelection != selectedAlliance;
    }

    /**
     * Forces the alliance to the provided value and marks the selector as overridden until
     * {@link #clearManualOverride()} is called.
     */
    public void selectAllianceManually(Alliance alliance) {
        if (selectionLocked) {
            return;
        }

        manualOverride = true;
        manualAlliance = alliance;
        selectedAlliance = alliance;
    }

    /**
     * Clears the manual override and falls back to the latest detection or default alliance.
     */
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
    }

    private Alliance mapTagToAlliance(int tagId) {
        if (tagId == blueTagId) {
            return Alliance.BLUE;
        }
        if (tagId == redTagId) {
            return Alliance.RED;
        }
        return Alliance.UNKNOWN;
    }

    public Alliance getDetectedAlliance() {
        return detectedAlliance;
    }

    public Alliance getSelectedAlliance() {
        return selectedAlliance;
    }

    public boolean isManualOverrideActive() {
        return manualOverride;
    }

    public Alliance getManualAlliance() {
        return manualAlliance;
    }

    public Alliance getDefaultAlliance() {
        return defaultAlliance;
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

    public boolean isSelectionLocked() {
        return selectionLocked;
    }

    public void lockSelection() {
        selectionLocked = true;
    }

    public void unlockSelection() {
        if (!selectionLocked) {
            return;
        }

        selectionLocked = false;
        refreshSelectedAlliance();
    }
}