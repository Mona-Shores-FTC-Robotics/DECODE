package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

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
        // Once selection is locked (after Start), do not change alliance via vision
        if (selectionLocked) {
            return getLastSnapshot();
        }

        if (vision == null) {
            clearDetection();
            refreshSelectedAlliance();
            return Optional.empty();
        }

        Optional<VisionSubsystemLimelight.TagSnapshot> snapshotOpt = vision.findAllianceSnapshot(null);
        if (!snapshotOpt.isPresent()) {
            clearDetection();
            refreshSelectedAlliance();
            return Optional.empty();
        }

        VisionSubsystemLimelight.TagSnapshot snapshot = snapshotOpt.get();
        lastSnapshot = snapshot;

        // Use pose based inference only
        Alliance poseAlliance = snapshot.inferAllianceFromPose();
        detectedAlliance = poseAlliance;

        // Merge with manual override and default alliance
        refreshSelectedAlliance();
        return snapshotOpt;
    }


    /**
     * Applies the current selected alliance to the robot containers and lighting.
     * Never applies UNKNOWN - defaults to BLUE if no valid alliance is selected.
     */
    public void applySelection(Robot robot, LightingSubsystem lighting) {
        Alliance allianceToApply = selectedAlliance != Alliance.UNKNOWN ? selectedAlliance : defaultAlliance;
        // Never apply UNKNOWN - default to BLUE as a safety net
        if (allianceToApply == null || allianceToApply == Alliance.UNKNOWN) {
            allianceToApply = Alliance.BLUE;
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

    /**
     * Convenience method for updating alliance selection during OpMode init (waitForStart loop).
     * Combines vision updates and selection application into a single call.
     *
     * <p>This is the drop-in method for OpModes. Call this in your waitForStart loop to:
     * <ul>
     *   <li>Poll vision for AprilTag alliance detection</li>
     *   <li>Process manual dpad overrides (requires BindingManager.update() to be called first)</li>
     *   <li>Apply the selection to robot state and lighting</li>
     * </ul>
     *
     * <p>Usage example in waitForStart:
     * <pre>{@code
     * BindingManager.update(); // Process button presses
     * Alliance currentAlliance = allianceSelector.updateDuringInit(robot.vision, robot, robot.lighting);
     * telemetry.addData("Alliance", currentAlliance.displayName());
     * }</pre>
     *
     * @param vision Vision subsystem for AprilTag detection (can be null)
     * @param robot Robot container to apply alliance selection (can be null)
     * @param lighting Lighting subsystem to apply alliance colors (can be null)
     * @return The currently selected alliance
     */
    public Alliance updateDuringInit(VisionSubsystemLimelight vision, Robot robot, LightingSubsystem lighting) {
        updateFromVision(vision);
        applySelection(robot, lighting);
        return selectedAlliance;
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
