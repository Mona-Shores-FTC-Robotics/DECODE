package org.firstinspires.ftc.teamcode.commands.LauncherCommands.config;

/**
 * Tunable RPM and hood positions, grouped so the Panels tree reads as a tidy
 * hierarchy instead of one long flat list:
 *
 * <pre>
 *   commandRangeConfig
 *    ├─ teleop ▸ shortRange / midRange / longRange
 *    ├─ auto   ▸ shortRange / midRange / farRange
 *    └─ timeoutSeconds
 * </pre>
 *
 * <p>Each range groups its three lanes ({@code left/center/right}) plus the
 * {@code hood} position together — that's the unit you actually tune ("change
 * the mid shot"). The lanes are usually set to the same value but can differ.
 *
 * <p>The data SHAPE lives here; per-robot VALUES live in
 * {@code util/RobotProfile.java} under {@code rangeConfig19429()} /
 * {@code rangeConfig20245()}. Access the live config via
 * {@code RobotProfile.forCurrent().commandRange}.
 */
public class CommandRangeConfig {

    /** One launch range: per-lane RPM target plus the (shared) hood position. */
    public static class RangeSettings {
        /** Left lane RPM. */
        public double left;
        /** Center lane RPM. */
        public double center;
        /** Right lane RPM. */
        public double right;
        /** Hood position, applied uniformly to all lanes in this range. */
        public double hood;

        /** Sets all three lanes to the same RPM and the hood position. Returns
         *  this so callers can set per-lane overrides afterward if needed. */
        public RangeSettings set(double allLanesRpm, double hood) {
            this.left = allLanesRpm;
            this.center = allLanesRpm;
            this.right = allLanesRpm;
            this.hood = hood;
            return this;
        }
    }

    /** LONG range. Distance-based shots interpolate min → max across the LONG
     *  zone; the LONG preset button uses the per-lane midpoint ((min+max)/2). */
    public static class LongRangeSettings {
        /** Left lane RPM at the near edge of the LONG zone. */
        public double minLeft;
        /** Center lane RPM at the near edge of the LONG zone. */
        public double minCenter;
        /** Right lane RPM at the near edge of the LONG zone. */
        public double minRight;
        /** Left lane RPM at the far edge of the LONG zone. */
        public double maxLeft;
        /** Center lane RPM at the far edge of the LONG zone. */
        public double maxCenter;
        /** Right lane RPM at the far edge of the LONG zone. */
        public double maxRight;
        /** Hood position, applied uniformly to all lanes. */
        public double hood;

        /** Sets all three lanes' min and max to single shared values plus hood. */
        public LongRangeSettings set(double allLanesMinRpm, double allLanesMaxRpm, double hood) {
            this.minLeft = allLanesMinRpm;
            this.minCenter = allLanesMinRpm;
            this.minRight = allLanesMinRpm;
            this.maxLeft = allLanesMaxRpm;
            this.maxCenter = allLanesMaxRpm;
            this.maxRight = allLanesMaxRpm;
            this.hood = hood;
            return this;
        }
    }

    /** Driver-selectable teleop ranges. */
    public static class TeleopRanges {
        public RangeSettings shortRange = new RangeSettings();
        public RangeSettings midRange = new RangeSettings();
        public LongRangeSettings longRange = new LongRangeSettings();
    }

    /** Autonomous ranges, tuned separately from their teleop counterparts. */
    public static class AutoRanges {
        public RangeSettings shortRange = new RangeSettings();
        public RangeSettings midRange = new RangeSettings();
        public RangeSettings farRange = new RangeSettings();
    }

    public TeleopRanges teleop = new TeleopRanges();
    public AutoRanges auto = new AutoRanges();

    /** Timeout in seconds before giving up on spin-up. */
    public double timeoutSeconds;
}
