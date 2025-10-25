package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.OptionalInt;

/**
 * Pure gate-opening policy. Inputs are intentionally primitive so the logic can be unit tested
 * easily and reused in both the init preview and runtime decision loop.
 */
public final class GatePolicy {

    private static final String TAG = "GatePolicy";

    private GatePolicy() { }

    /**
     * Computes the earliest shot index (1-based) where opening the gate ensures that {@code mustLeaveLast}
     * artifacts remain in the ramp at the end of autonomous. The shot index tracks every ball scored by
     * both robots. When {@code gateOpened} is already true the policy declines to open again.
     */
    public static OptionalInt computeOpenShotIndex(int expectedTotalShots,
                                                   int mustLeaveLast,
                                                   int partnerEarlyShots,
                                                   int alreadyShot,
                                                   boolean gateOpened) {
        if (gateOpened) {
            return OptionalInt.empty();
        }
        if (expectedTotalShots <= mustLeaveLast) {
            return OptionalInt.empty();
        }

        int flushThreshold = expectedTotalShots - mustLeaveLast;
        if (flushThreshold <= 0) {
            return OptionalInt.empty();
        }

        int partnerFloor = Math.max(0, partnerEarlyShots);
        int minimumIndex = Math.max(flushThreshold, partnerFloor);
        int openIndex = Math.max(minimumIndex, alreadyShot + 1);

        return OptionalInt.of(openIndex);
    }

    /**
     * Simple helper used inside the OpMode. Treats {@code alreadyShot} as the total number of shots
     * that have either landed already or are about to leave the robot once {@link #shouldOpenNow} is
     * evaluated.
     */
    public static boolean shouldOpenNow(int alreadyShot,
                                        int expectedTotalShots,
                                        int mustLeaveLast,
                                        boolean gateOpened) {
        if (gateOpened) {
            return false;
        }
        if (expectedTotalShots <= mustLeaveLast) {
            return false;
        }
        int flushThreshold = expectedTotalShots - mustLeaveLast;
        return alreadyShot >= flushThreshold;
    }

    public static Preview preview(int expectedTotalShots,
                                  int mustLeaveLast,
                                  int partnerEarlyShots) {
        OptionalInt openIndex = computeOpenShotIndex(
                expectedTotalShots,
                mustLeaveLast,
                partnerEarlyShots,
                0,
                false
        );
        int remainingIfOpened;
        if (openIndex.isPresent()) {
            remainingIfOpened = expectedTotalShots - openIndex.getAsInt();
        } else {
            remainingIfOpened = expectedTotalShots;
        }
        if (remainingIfOpened < 0) {
            remainingIfOpened = 0;
        }
        return new Preview(expectedTotalShots, mustLeaveLast, openIndex, remainingIfOpened);
    }

    private static void runSelfTest() {
        try {
            expect(computeOpenShotIndex(9, 9, 0, 0, false).isEmpty(), "Exactly must-leave should not open");
            expect(computeOpenShotIndex(8, 9, 0, 0, false).isEmpty(), "Below capacity should not open");
            OptionalInt idx = computeOpenShotIndex(15, 9, 0, 0, false);
            expect(idx.isPresent() && idx.getAsInt() == 6, "15 total should open at shot 6");
            expect(!shouldOpenNow(5, 15, 9, false), "Before threshold should not open");
            expect(shouldOpenNow(6, 15, 9, false), "At threshold should open");
            expect(!shouldOpenNow(10, 8, 9, false), "Must leave greater than total should never open");
        } catch (IllegalStateException ex) {
            RobotLog.ee(TAG, ex, "Gate policy self-test failed");
        }
    }

    private static void expect(boolean condition, String message) {
        if (!condition) {
            throw new IllegalStateException(message);
        }
    }

    static {
        runSelfTest();
    }

    public static final class Preview {
        public final int expectedTotalShots;
        public final int mustLeaveLast;
        public final OptionalInt openAtShotIndex;
        public final int remainingShotsIfOpen;

        public Preview(int expectedTotalShots,
                       int mustLeaveLast,
                       OptionalInt openAtShotIndex,
                       int remainingShotsIfOpen) {
            this.expectedTotalShots = expectedTotalShots;
            this.mustLeaveLast = mustLeaveLast;
            this.openAtShotIndex = openAtShotIndex;
            this.remainingShotsIfOpen = remainingShotsIfOpen;
        }
    }
}
