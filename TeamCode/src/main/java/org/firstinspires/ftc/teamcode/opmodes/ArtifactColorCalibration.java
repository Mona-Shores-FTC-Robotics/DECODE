package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.LaneSample;
import org.firstinspires.ftc.teamcode.subsystems.LauncherLane;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Calibration OpMode for tuning artifact color detection thresholds.
 *
 * Usage:
 * 1. Place a GREEN artifact in the desired lane (use D-pad to select)
 * 2. Press A to capture multiple GREEN samples
 * 3. Place a PURPLE artifact in the lane
 * 4. Press B to capture multiple PURPLE samples
 * 5. Review recommended threshold values on telemetry
 * 6. Update values in FTC Dashboard Config → IntakeSubsystem → LaneSensorConfig
 *
 * Controls:
 * - D-pad Up/Down: Cycle through lanes (LEFT/CENTER/RIGHT)
 * - A: Capture GREEN artifact sample
 * - B: Capture PURPLE artifact sample
 * - X: Clear all samples and start over
 * - Y: Toggle continuous sampling mode
 */
@TeleOp(name = "Artifact Color Calibration", group = "Tuning")
public class ArtifactColorCalibration extends LinearOpMode {

    private IntakeSubsystem intake;
    private LauncherLane selectedLane = LauncherLane.CENTER;
    private final List<ColorSample> greenSamples = new ArrayList<>();
    private final List<ColorSample> purpleSamples = new ArrayList<>();
    private boolean continuousSampling = false;
    private final ElapsedTime sampleTimer = new ElapsedTime();
    private static final double SAMPLE_INTERVAL_MS = 200.0; // Match IntakeSubsystem polling rate

    private static class ColorSample {
        final float hue;
        final float saturation;
        final float value;
        final float redRatio;
        final float greenRatio;
        final float blueRatio;
        final double distanceCm;

        ColorSample(LaneSample sample) {
            this.hue = sample.hue;
            this.saturation = sample.saturation;
            this.value = sample.value;

            // Calculate normalized RGB ratios
            float total = sample.scaledRed + sample.scaledGreen + sample.scaledBlue;
            if (total > 0.01f) {
                this.redRatio = sample.scaledRed / total;
                this.greenRatio = sample.scaledGreen / total;
                this.blueRatio = sample.scaledBlue / total;
            } else {
                this.redRatio = 0;
                this.greenRatio = 0;
                this.blueRatio = 0;
            }
            this.distanceCm = sample.distanceCm;
        }
    }

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Artifact Color Calibration...");
        telemetry.update();

        // Initialize intake subsystem (no telemetry service needed for calibration)
        intake = new IntakeSubsystem(hardwareMap, null);

        telemetry.addLine("✓ Ready!");
        telemetry.addLine();
        telemetry.addLine("Place GREEN artifact in lane, then press A");
        telemetry.addLine("Place PURPLE artifact in lane, then press B");
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();
        sampleTimer.reset();

        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        boolean lastA = false;
        boolean lastB = false;
        boolean lastX = false;
        boolean lastY = false;

        while (opModeIsActive()) {
            // Lane selection (D-pad)
            if (gamepad1.dpad_up && !lastDpadUp) {
                selectedLane = nextLane(selectedLane, 1);
            }
            if (gamepad1.dpad_down && !lastDpadDown) {
                selectedLane = nextLane(selectedLane, -1);
            }
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            // Continuous sampling toggle
            if (gamepad1.y && !lastY) {
                continuousSampling = !continuousSampling;
                sampleTimer.reset();
            }
            lastY = gamepad1.y;

            // Manual capture or continuous sampling
            boolean shouldCapture = false;
            ArtifactColor captureColor = null;

            if (gamepad1.a && !lastA) {
                shouldCapture = true;
                captureColor = ArtifactColor.GREEN;
            } else if (gamepad1.b && !lastB) {
                shouldCapture = true;
                captureColor = ArtifactColor.PURPLE;
            } else if (continuousSampling && sampleTimer.milliseconds() >= SAMPLE_INTERVAL_MS) {
                // Auto-capture based on detected color
                LaneSample sample = intake.getLaneSample(selectedLane);
                if (sample.color == ArtifactColor.GREEN || sample.color == ArtifactColor.PURPLE) {
                    shouldCapture = true;
                    captureColor = sample.color;
                }
                sampleTimer.reset();
            }
            lastA = gamepad1.a;
            lastB = gamepad1.b;

            // Capture sample
            if (shouldCapture && captureColor != null) {
                intake.refreshLaneSensors(); // Force fresh read
                LaneSample sample = intake.getLaneSample(selectedLane);

                if (sample.sensorPresent) {
                    ColorSample colorSample = new ColorSample(sample);
                    if (captureColor == ArtifactColor.GREEN) {
                        greenSamples.add(colorSample);
                    } else {
                        purpleSamples.add(colorSample);
                    }
                }
            }

            // Clear samples
            if (gamepad1.x && !lastX) {
                greenSamples.clear();
                purpleSamples.clear();
            }
            lastX = gamepad1.x;

            // Display telemetry
            displayTelemetry();
        }
    }

    private LauncherLane nextLane(LauncherLane current, int direction) {
        LauncherLane[] lanes = LauncherLane.values();
        int index = current.ordinal();
        index = (index + direction + lanes.length) % lanes.length;
        return lanes[index];
    }

    private void displayTelemetry() {
        telemetry.addLine("=== ARTIFACT COLOR CALIBRATION ===");
        telemetry.addLine();

        // Current lane info
        telemetry.addData("Selected Lane", "%s (D-pad ↑↓)", selectedLane);
        LaneSample current = intake.getLaneSample(selectedLane);

        if (current.sensorPresent) {
            telemetry.addLine();
            telemetry.addLine("--- Current Reading ---");
            telemetry.addData("Distance", "%.1f cm", current.distanceCm);
            telemetry.addData("Detected", "%s", current.color);
            telemetry.addData("HSV", "H:%.0f° S:%.2f V:%.2f",
                current.hue, current.saturation, current.value);

            float total = current.scaledRed + current.scaledGreen + current.scaledBlue;
            if (total > 0.01f) {
                telemetry.addData("RGB Ratios", "R:%.2f G:%.2f B:%.2f",
                    current.scaledRed / total,
                    current.scaledGreen / total,
                    current.scaledBlue / total);
            }
            telemetry.addData("RGB Raw", "R:%d G:%d B:%d",
                current.scaledRed, current.scaledGreen, current.scaledBlue);
        } else {
            telemetry.addData("Sensor", "NOT PRESENT in %s lane", selectedLane);
        }

        // Sample counts
        telemetry.addLine();
        telemetry.addData("GREEN Samples", "%d (Press A)", greenSamples.size());
        telemetry.addData("PURPLE Samples", "%d (Press B)", purpleSamples.size());
        telemetry.addData("Continuous Mode", "%s (Press Y)", continuousSampling ? "ON" : "OFF");
        telemetry.addData("Clear All", "Press X");

        // Analysis and recommendations
        if (!greenSamples.isEmpty() || !purpleSamples.isEmpty()) {
            telemetry.addLine();
            telemetry.addLine("=== RECOMMENDED THRESHOLDS ===");

            if (!greenSamples.isEmpty()) {
                displayRecommendations("GREEN", greenSamples);
            }

            if (!purpleSamples.isEmpty()) {
                displayRecommendations("PURPLE", purpleSamples);
            }

            // Cross-analysis for separation
            if (!greenSamples.isEmpty() && !purpleSamples.isEmpty()) {
                telemetry.addLine();
                telemetry.addLine("--- Separation Analysis ---");

                float greenHueAvg = average(greenSamples, s -> s.hue);
                float purpleHueAvg = average(purpleSamples, s -> s.hue);
                float greenGRatioAvg = average(greenSamples, s -> s.greenRatio);
                float purpleGRatioAvg = average(purpleSamples, s -> s.greenRatio);

                telemetry.addData("Hue Separation", "%.0f°", Math.abs(greenHueAvg - purpleHueAvg));
                telemetry.addData("Green Ratio Sep", "%.2f", Math.abs(greenGRatioAvg - purpleGRatioAvg));

                // Confidence assessment
                if (Math.abs(greenGRatioAvg - purpleGRatioAvg) > 0.2) {
                    telemetry.addLine("✓ Good separation - RGB ratios reliable");
                } else if (Math.abs(greenHueAvg - purpleHueAvg) > 60) {
                    telemetry.addLine("✓ Good separation - HSV hue reliable");
                } else {
                    telemetry.addLine("⚠ Poor separation - may need better lighting");
                }
            }
        }

        telemetry.update();
    }

    private void displayRecommendations(String colorName, List<ColorSample> samples) {
        telemetry.addLine(String.format("--- %s (n=%d) ---", colorName, samples.size()));

        // HSV stats
        float hueAvg = average(samples, s -> s.hue);
        float hueMin = min(samples, s -> s.hue);
        float hueMax = max(samples, s -> s.hue);
        float satAvg = average(samples, s -> s.saturation);
        float satMin = min(samples, s -> s.saturation);
        float valAvg = average(samples, s -> s.value);
        float valMin = min(samples, s -> s.value);

        telemetry.addData("  Hue Range", "%.0f° - %.0f° (avg: %.0f°)", hueMin, hueMax, hueAvg);
        telemetry.addData("  Saturation", "%.2f - %.2f (avg: %.2f)", satMin, 1.0f, satAvg);
        telemetry.addData("  Value", "%.2f min (avg: %.2f)", valMin, valAvg);

        // RGB ratio stats
        float rRatioAvg = average(samples, s -> s.redRatio);
        float gRatioAvg = average(samples, s -> s.greenRatio);
        float bRatioAvg = average(samples, s -> s.blueRatio);
        float gRatioMin = min(samples, s -> s.greenRatio);
        float gRatioMax = max(samples, s -> s.greenRatio);

        telemetry.addData("  RGB Ratios", "R:%.2f G:%.2f B:%.2f", rRatioAvg, gRatioAvg, bRatioAvg);
        telemetry.addData("  Green Ratio", "%.2f - %.2f", gRatioMin, gRatioMax);

        // Distance stats
        double distAvg = average(samples, s -> (float) s.distanceCm);
        double distMax = max(samples, s -> (float) s.distanceCm);
        telemetry.addData("  Distance", "%.1f cm avg (max: %.1f cm)", distAvg, distMax);

        // Specific recommendations
        if (colorName.equals("GREEN")) {
            float hueMargin = 10.0f;
            telemetry.addData("  → greenHueMin", "%.0f", Math.max(0, hueMin - hueMargin));
            telemetry.addData("  → greenHueMax", "%.0f", Math.min(180, hueMax + hueMargin));
            telemetry.addData("  → greenRatioMin", "%.2f", gRatioMin - 0.05f);
        } else {
            float hueMargin = 15.0f;
            // Handle purple wrap-around (purple is typically 270-330 or 0-30)
            if (hueAvg > 180) {
                telemetry.addData("  → purpleHueMin", "%.0f", Math.max(0, hueMin - hueMargin));
                telemetry.addData("  → purpleHueMax", "%.0f", Math.min(360, hueMax + hueMargin));
            } else {
                telemetry.addData("  → purpleHueWrapMax", "%.0f", Math.min(60, hueMax + hueMargin));
            }
            telemetry.addData("  → purpleGreenMax", "%.2f", gRatioMax + 0.05f);
        }
        telemetry.addData("  → minValue", "%.2f", Math.max(0.01f, valMin - 0.02f));
        telemetry.addData("  → minSaturation", "%.2f", Math.max(0.05f, satMin - 0.05f));
    }

    private float average(List<ColorSample> samples, SampleExtractor extractor) {
        if (samples.isEmpty()) return 0;
        float sum = 0;
        for (ColorSample s : samples) {
            sum += extractor.extract(s);
        }
        return sum / samples.size();
    }

    private float min(List<ColorSample> samples, SampleExtractor extractor) {
        if (samples.isEmpty()) return 0;
        float min = Float.MAX_VALUE;
        for (ColorSample s : samples) {
            float val = extractor.extract(s);
            if (val < min) min = val;
        }
        return min;
    }

    private float max(List<ColorSample> samples, SampleExtractor extractor) {
        if (samples.isEmpty()) return 0;
        float max = Float.MIN_VALUE;
        for (ColorSample s : samples) {
            float val = extractor.extract(s);
            if (val > max) max = val;
        }
        return max;
    }

    @FunctionalInterface
    private interface SampleExtractor {
        float extract(ColorSample sample);
    }
}
