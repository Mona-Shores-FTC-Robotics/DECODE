package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;

/**
 * Dashboard-backed configuration for the ExampleGateAuto. The init loop menu mutates these fields
 * directly so students can tweak plans without redeploying code.
 */
@Config
public final class AutoConfig {

    public enum Alliance {
        RED,
        BLUE
    }

    public enum GateMode {
        AUTO,
        MANUAL_AFTER_N,
        NEVER
    }

    public static Alliance alliance = Alliance.RED;
    public static int partnerPreloads = 3;
    public static boolean weOwnGate = true;
    public static int expectedTotalShots = 15;
    public static int plannedCycles = 2;
    public static GateMode gateMode = GateMode.AUTO;
    public static int manualOpenAfterShots = 6;

    public static int mustLeaveLast = 9;
    public static int ourPreloadCount = 3;

    private AutoConfig() { }

    public static void clamp() {
        partnerPreloads = clampToSet(partnerPreloads, 0, 3);
        expectedTotalShots = Math.max(0, expectedTotalShots);
        plannedCycles = Math.max(0, plannedCycles);
        manualOpenAfterShots = Math.max(0, manualOpenAfterShots);
        ourPreloadCount = clampToRange(ourPreloadCount, 0, 3);
    }

    private static int clampToSet(int value, int... allowed) {
        for (int option : allowed) {
            if (value == option) {
                return option;
            }
        }
        return allowed.length > 0 ? allowed[0] : value;
    }

    private static int clampToRange(int value, int min, int max) {
        if (value < min) {
            return min;
        }
        if (value > max) {
            return max;
        }
        return value;
    }

    public static class Snapshot {
        public final Alliance alliance;
        public final int partnerPreloads;
        public final boolean weOwnGate;
        public final int expectedTotalShots;
        public final int plannedCycles;
        public final GateMode gateMode;
        public final int manualOpenAfterShots;
        public final int ourPreloadCount;

        public Snapshot(Alliance alliance,
                        int partnerPreloads,
                        boolean weOwnGate,
                        int expectedTotalShots,
                        int plannedCycles,
                        GateMode gateMode,
                        int manualOpenAfterShots,
                        int ourPreloadCount) {
            this.alliance = alliance;
            this.partnerPreloads = partnerPreloads;
            this.weOwnGate = weOwnGate;
            this.expectedTotalShots = expectedTotalShots;
            this.plannedCycles = plannedCycles;
            this.gateMode = gateMode;
            this.manualOpenAfterShots = manualOpenAfterShots;
            this.ourPreloadCount = ourPreloadCount;
        }
    }
}
