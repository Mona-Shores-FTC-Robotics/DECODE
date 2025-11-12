package org.firstinspires.ftc.teamcode.commands.LauncherCommands;

/**
 * Coordination hook so multiple manual launcher commands can keep the spin mode locked
 * until every manual interaction has finished.
 */
public interface ManualSpinController {

    ManualSpinController NO_OP = new ManualSpinController() {
        @Override
        public void enterManualSpin() {
            // no-op when no coordinator is present
        }

        @Override
        public void exitManualSpin() {
            // no-op when no coordinator is present
        }
    };

    void enterManualSpin();
    void exitManualSpin();
}
