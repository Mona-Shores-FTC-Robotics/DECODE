package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ivy.Command;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * Declarative gamepad-button-to-Command bindings.
 *
 * <p>Pedro Pathing Ivy 1.0.0 ships no bindings module, so this shim polls a
 * list of {@link BooleanSupplier} triggers each loop and fires
 * {@code Command.schedule()} / {@code Command.cancel()} on rising / falling
 * edges.
 *
 * <p>Usage:
 * <pre>{@code
 *   bindings.when(() -> gamepad1.a).onTrue(launcher.fire());
 *   bindings.when(() -> gamepad1.right_trigger > 0.5).whileTrue(intakeCommand);
 *   bindings.when(gamepad1::aWasPressed).onTrue(toggleAimCommand);
 * }</pre>
 *
 * <p>Call {@link #update()} once per OpMode loop, before {@code Scheduler.execute()}.
 */
public final class GamepadBindings {
    private final List<Trigger> triggers = new ArrayList<>();

    public Trigger when(BooleanSupplier condition) {
        Trigger t = new Trigger(condition);
        triggers.add(t);
        return t;
    }

    public void update() {
        for (Trigger t : triggers) t.poll();
    }

    /** Removes every binding. Useful between OpModes or in test setup. */
    public void clear() {
        triggers.clear();
    }

    public static final class Trigger {
        private final BooleanSupplier condition;
        private boolean last = false;
        private final List<Runnable> onTrue = new ArrayList<>();
        private final List<Runnable> onFalse = new ArrayList<>();
        private final List<Runnable> whileTrue = new ArrayList<>();

        Trigger(BooleanSupplier condition) { this.condition = condition; }

        public Trigger onTrue(Command cmd)  { onTrue.add(cmd::schedule);  return this; }
        public Trigger onFalse(Command cmd) { onFalse.add(cmd::schedule); return this; }

        public Trigger whileTrue(Command cmd) {
            whileTrue.add(() -> { if (!cmd.isScheduled()) cmd.schedule(); });
            onFalse.add(cmd::cancel);
            return this;
        }

        void poll() {
            boolean now = condition.getAsBoolean();
            if (now && !last)  for (Runnable r : onTrue)    r.run();
            if (!now && last)  for (Runnable r : onFalse)   r.run();
            if (now)           for (Runnable r : whileTrue) r.run();
            last = now;
        }
    }
}
