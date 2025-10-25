package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.AutoConfig;
import org.firstinspires.ftc.teamcode.autonomous.GatePolicy;

/**
 * Handles the init-loop configuration menu for ExampleGateAuto. Students can tweak expectations with
 * gamepad inputs and mirror the same controls through the dashboard.
 */
public class InitMenu {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private AutoConfig.Snapshot snapshot = buildSnapshot();

    private boolean prevX;
    private boolean prevY;
    private boolean prevA;
    private boolean prevB;
    private boolean prevLB;
    private boolean prevRB;
    private boolean prevDpadLeft;
    private boolean prevDpadRight;
    private boolean prevDpadUp;
    private boolean prevDpadDown;

    private GatePolicy.Preview preview = GatePolicy.preview(
            AutoConfig.expectedTotalShots,
            AutoConfig.mustLeaveLast,
            AutoConfig.partnerPreloads
    );

    public AutoConfig.Snapshot update(Gamepad gamepad, Telemetry telemetry) {
        if (gamepad == null) {
            return snapshot;
        }

        handleInputs(gamepad);
        AutoConfig.clamp();
        AutoConfig.ourPreloadCount = computeOurPreloads();
        updateSnapshot();
        preview = GatePolicy.preview(
                snapshot.expectedTotalShots,
                AutoConfig.mustLeaveLast,
                snapshot.partnerPreloads
        );

        writeTelemetry(telemetry);
        pushDashboard();

        return snapshot;
    }

    public GatePolicy.Preview getPreview() {
        return preview;
    }

    private void handleInputs(Gamepad gamepad) {
        if (rising(gamepad.x, prevX)) {
            AutoConfig.alliance = (AutoConfig.alliance == AutoConfig.Alliance.RED)
                    ? AutoConfig.Alliance.BLUE
                    : AutoConfig.Alliance.RED;
        }
        if (rising(gamepad.y, prevY)) {
            AutoConfig.partnerPreloads = (AutoConfig.partnerPreloads == 0) ? 3 : 0;
        }
        if (rising(gamepad.a, prevA)) {
            AutoConfig.weOwnGate = !AutoConfig.weOwnGate;
        }
        if (rising(gamepad.left_bumper, prevLB)) {
            AutoConfig.expectedTotalShots = Math.max(0, AutoConfig.expectedTotalShots - 1);
        }
        if (rising(gamepad.right_bumper, prevRB)) {
            AutoConfig.expectedTotalShots += 1;
        }
        if (rising(gamepad.dpad_left, prevDpadLeft)) {
            AutoConfig.plannedCycles = Math.max(0, AutoConfig.plannedCycles - 1);
        }
        if (rising(gamepad.dpad_right, prevDpadRight)) {
            AutoConfig.plannedCycles += 1;
        }
        if (rising(gamepad.b, prevB)) {
            AutoConfig.gateMode = nextGateMode(AutoConfig.gateMode);
        }
        if (AutoConfig.gateMode == AutoConfig.GateMode.MANUAL_AFTER_N) {
            if (rising(gamepad.dpad_up, prevDpadUp)) {
                AutoConfig.manualOpenAfterShots += 1;
            }
            if (rising(gamepad.dpad_down, prevDpadDown)) {
                AutoConfig.manualOpenAfterShots = Math.max(0, AutoConfig.manualOpenAfterShots - 1);
            }
        }

        prevX = gamepad.x;
        prevY = gamepad.y;
        prevA = gamepad.a;
        prevB = gamepad.b;
        prevLB = gamepad.left_bumper;
        prevRB = gamepad.right_bumper;
        prevDpadLeft = gamepad.dpad_left;
        prevDpadRight = gamepad.dpad_right;
        prevDpadUp = gamepad.dpad_up;
        prevDpadDown = gamepad.dpad_down;
    }

    private void writeTelemetry(Telemetry telemetry) {
        if (telemetry == null) {
            return;
        }

        telemetry.addData("Alliance (X)", snapshot.alliance);
        telemetry.addData("Partner preloads (Y)", snapshot.partnerPreloads);
        telemetry.addData("We own gate (A)", snapshot.weOwnGate ? "YES" : "NO");
        telemetry.addData("Expected shots (LB/RB)", snapshot.expectedTotalShots);
        telemetry.addData("Cycles (DPad L/R)", snapshot.plannedCycles);
        telemetry.addData("Gate mode (B)", snapshot.gateMode);
        if (snapshot.gateMode == AutoConfig.GateMode.MANUAL_AFTER_N) {
            telemetry.addData("Manual open after (DPad Up/Down)", snapshot.manualOpenAfterShots);
        }
        telemetry.addData("Our preloads (auto)", snapshot.ourPreloadCount);
        telemetry.addData("Preview total shots", preview.expectedTotalShots);
        telemetry.addData("Preview open at", preview.openAtShotIndex.isPresent()
                ? preview.openAtShotIndex.getAsInt()
                : "No gate open needed");
        telemetry.addData("Preview remaining (should be 9)", preview.remainingShotsIfOpen);
        telemetry.update();
    }

    private void pushDashboard() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Alliance", snapshot.alliance.toString());
        packet.put("PartnerPreloads", snapshot.partnerPreloads);
        packet.put("WeOwnGate", snapshot.weOwnGate);
        packet.put("ExpectedShots", snapshot.expectedTotalShots);
        packet.put("PlannedCycles", snapshot.plannedCycles);
        packet.put("GateMode", snapshot.gateMode.toString());
        packet.put("ManualAfterShots", snapshot.manualOpenAfterShots);
        packet.put("OurPreloads", snapshot.ourPreloadCount);
        packet.put("OpenAtShot", preview.openAtShotIndex.isPresent()
                ? preview.openAtShotIndex.getAsInt()
                : -1);
        packet.put("MustLeaveLast", AutoConfig.mustLeaveLast);
        packet.put("RemainingIfOpen", preview.remainingShotsIfOpen);
        dashboard.sendTelemetryPacket(packet);
    }

    private void updateSnapshot() {
        snapshot = buildSnapshot();
    }

    private static boolean rising(boolean current, boolean previous) {
        return current && !previous;
    }

    private static AutoConfig.GateMode nextGateMode(AutoConfig.GateMode current) {
        AutoConfig.GateMode[] values = AutoConfig.GateMode.values();
        int index = current.ordinal();
        index = (index + 1) % values.length;
        return values[index];
    }

    private static int computeOurPreloads() {
        int remainder = AutoConfig.expectedTotalShots - AutoConfig.partnerPreloads - (AutoConfig.plannedCycles * 3);
        if (remainder < 0) {
            remainder = 0;
        }
        if (remainder > 3) {
            remainder = 3;
        }
        return remainder;
    }

    private static AutoConfig.Snapshot buildSnapshot() {
        return new AutoConfig.Snapshot(
                AutoConfig.alliance,
                AutoConfig.partnerPreloads,
                AutoConfig.weOwnGate,
                AutoConfig.expectedTotalShots,
                AutoConfig.plannedCycles,
                AutoConfig.gateMode,
                AutoConfig.manualOpenAfterShots,
                AutoConfig.ourPreloadCount
        );
    }
}
