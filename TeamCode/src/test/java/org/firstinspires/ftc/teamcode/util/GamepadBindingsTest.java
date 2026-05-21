package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;

import org.junit.Before;
import org.junit.Test;

import java.util.Collections;
import java.util.Set;

/**
 * JUnit 4 tests for {@link GamepadBindings}. Uses a hand-rolled CountingCommand
 * that overrides schedule/cancel/isScheduled so the real Ivy Scheduler is
 * never touched (it has process-global static state we don't want in tests).
 */
public class GamepadBindingsTest {

    private GamepadBindings bindings;
    private boolean triggerState;
    private CountingCommand cmd;

    @Before
    public void setUp() {
        bindings = new GamepadBindings();
        triggerState = false;
        cmd = new CountingCommand();
    }

    @Test
    public void onTrue_firesOnRisingEdge() {
        bindings.when(() -> triggerState).onTrue(cmd);

        bindings.update();
        assertEquals("no fire when starting false", 0, cmd.scheduleCalls);

        triggerState = true;
        bindings.update();
        assertEquals("fires on false->true", 1, cmd.scheduleCalls);
    }

    @Test
    public void onTrue_doesNotRefireWhileStaysTrue() {
        bindings.when(() -> triggerState).onTrue(cmd);

        triggerState = true;
        bindings.update();
        bindings.update();
        bindings.update();
        assertEquals("only one fire across three true polls", 1, cmd.scheduleCalls);
    }

    @Test
    public void onTrue_refiresAfterReturningFalse() {
        bindings.when(() -> triggerState).onTrue(cmd);

        triggerState = true;
        bindings.update();
        triggerState = false;
        bindings.update();
        triggerState = true;
        bindings.update();
        assertEquals("fires once per rising edge", 2, cmd.scheduleCalls);
    }

    @Test
    public void onFalse_firesOnFallingEdge() {
        bindings.when(() -> triggerState).onFalse(cmd);

        triggerState = true;
        bindings.update();
        assertEquals("no fire on rising edge", 0, cmd.scheduleCalls);

        triggerState = false;
        bindings.update();
        assertEquals("fires on true->false", 1, cmd.scheduleCalls);
    }

    @Test
    public void whileTrue_schedulesOnRisingEdge_cancelsOnFalling() {
        bindings.when(() -> triggerState).whileTrue(cmd);

        triggerState = true;
        bindings.update();
        assertEquals("scheduled on rising edge", 1, cmd.scheduleCalls);
        assertTrue(cmd.scheduled);

        triggerState = false;
        bindings.update();
        assertEquals("cancelled on falling edge", 1, cmd.cancelCalls);
        assertFalse(cmd.scheduled);
    }

    @Test
    public void whileTrue_doesNotRescheduleWhileAlreadyScheduled() {
        bindings.when(() -> triggerState).whileTrue(cmd);

        triggerState = true;
        bindings.update();
        bindings.update();
        bindings.update();
        assertEquals("only one schedule across multiple true polls", 1, cmd.scheduleCalls);
    }

    @Test
    public void whileTrue_reschedulesIfCommandFinishesEarly() {
        bindings.when(() -> triggerState).whileTrue(cmd);

        triggerState = true;
        bindings.update();
        cmd.scheduled = false;  // simulate the command finishing naturally mid-hold
        bindings.update();
        assertEquals("reschedules when not scheduled and trigger still true", 2, cmd.scheduleCalls);
    }

    @Test
    public void update_pollsAllRegisteredTriggers() {
        CountingCommand cmdA = new CountingCommand();
        CountingCommand cmdB = new CountingCommand();
        bindings.when(() -> triggerState).onTrue(cmdA);
        bindings.when(() -> triggerState).onTrue(cmdB);

        triggerState = true;
        bindings.update();

        assertEquals(1, cmdA.scheduleCalls);
        assertEquals(1, cmdB.scheduleCalls);
    }

    @Test
    public void clear_removesAllTriggers() {
        bindings.when(() -> triggerState).onTrue(cmd);
        bindings.clear();

        triggerState = true;
        bindings.update();
        assertEquals(0, cmd.scheduleCalls);
    }

    /** Fake Command that records schedule/cancel calls without touching Scheduler. */
    private static final class CountingCommand implements Command {
        int scheduleCalls = 0;
        int cancelCalls = 0;
        boolean scheduled = false;

        @Override public void schedule()  { scheduleCalls++; scheduled = true; }
        @Override public void cancel()    { cancelCalls++; scheduled = false; }
        @Override public boolean isScheduled() { return scheduled; }

        @Override public Set<Object> requirements() { return Collections.emptySet(); }
        @Override public int priority() { return 0; }
        @Override public InterruptedBehavior interruptedBehavior() { return InterruptedBehavior.END; }
        @Override public ConflictBehavior conflictBehavior() { return ConflictBehavior.OVERRIDE; }
        @Override public BlockedBehavior blockedBehavior() { return BlockedBehavior.CANCEL; }
        @Override public void start() {}
        @Override public boolean done() { return false; }
        @Override public void execute() {}
        @Override public void end(EndCondition endCondition) {}
    }
}
