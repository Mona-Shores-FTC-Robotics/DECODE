# Ivy Migration Plan: NextFTC → Pedro Pathing Ivy

Status: **Proposal, not started.** Generated 2026-05-17 from a live read of `com.pedropathing:ivy:1.0.0` source in External Libraries and inspection of two reference repos already running Ivy.

## Prerequisite: merge fusion branch first

A parallel branch `claude/evaluate-rewrite-sensor-fusion-LI9x2` adds Kalman filter sensor fusion via a new `pedroPathing/FusionLocalizer.java` and deletes `util/PoseFusion.java`. That branch also heavily refactors `DriveSubsystem.java`.

**This Ivy migration must start from a TuesdayTest that already has fusion merged.** Reasons:

1. The Ivy migration rewrites `DriveSubsystem.java`. So does the fusion branch. Doing them as parallel branches guarantees a merge conflict in that file with no clean way to untangle which change caused any robot-side regression.
2. `FusionLocalizer.java` is framework-agnostic (pure Pedro — `Localizer`, `Pose`, `Matrix`, `Vector`) and survives the Ivy migration unchanged. The Ivy migration's `DriveSubsystem` rewrite simply uses the already-installed fusion localizer.
3. Fusion is the smaller, more validatable change. Land it first.

**Recommended sequencing:**

```
1. Robot-test claude/evaluate-rewrite-sensor-fusion-LI9x2 (one session).
2. If it works, merge to TuesdayTest with `git merge --no-ff` so the
   merge commit exists for a clean one-command revert.
3. Rebase ivy-migration onto the new TuesdayTest.
4. Begin the Ivy migration on the fusion-aware base.
```

**Rollback safety net for fusion** (in case it misbehaves after Ivy stacks on top):

- **Runtime toggle:** `DriveSubsystem.FusionConfig.minTargetAreaPercent` — set high via Panels to reject all Limelight measurements; falls back to dead-reckoning through Kalman.
- **One-line code disable:** `Constants.activeFusionLocalizer` is wired into the follower via a single `.setLocalizer(activeFusionLocalizer)` call. Swap for the original Pinpoint localizer to fully disable fusion without touching DriveSubsystem.
- **Branch-level revert:** if fusion's merge commit is preserved (`--no-ff`), `git revert -m 1 <merge>` undoes it cleanly. Note: only works before Ivy migration's DriveSubsystem rewrite depends on the fusion-shape Drive.

## TL;DR

- **Migrate offseason, not mid-season.** Ivy 1.0.0 was released 2026-05-11 (six days before this doc); the codebase has ~32 files / ~100 NextFTC reference lines to touch.
- **Ivy 1.0.0 + Sloth + Pedro Pathing is a known-working stack** (proved by `MOEbo-Sapiens/MOEbo-Sapiens-Decode`).
- **Ivy ships no bindings module.** We will write a small shim (`util/IvyBindings.java`) on top of the SDK 11.1 gamepad edge-detection primitives. Full source is in this doc.
- **Plan in 5 PRs.** PR 1 lands the dep + shim + tests with zero production-code changes. Each subsequent PR keeps the codebase buildable and runnable on the robot.

## Why now (this doc), not now (the migration)

- NextFTC works. Our code runs. The cost of *not* migrating mid-season is zero.
- Ivy is fresh. APIs may shift in 1.0.x / 1.1.x patches. Waiting de-risks.
- This document is the discovery work that's expensive to redo. Capturing it now means offseason execution is mechanical.

## Confirmed compatibility evidence

Both observed at `raw.githubusercontent.com` 2026-05-17:

- **`MOEbo-Sapiens/MOEbo-Sapiens-Decode`** runs `com.pedropathing:ivy:0.0.1-SNAPSHOT` + `dev.frozenmilk.sinister:Sloth:0.2.4` + `com.acmerobotics.slothboard:dashboard:0.2.4+0.5.1` + `com.bylazar.sloth:fullpanels:0.2.4+1.0.12` together. No NextFTC.
- **`BaronClaps/22131-Decode`** (Pedro author's repo) runs `com.pedropathing:ivy:0.0.1-LOCAL` + Sloth + slothboard. No NextFTC. Uses `com.acmerobotics.dashboard.config.Config` annotations against the slothboard fork without modification.

Caveat: both reference repos are on **pre-1.0.0 Ivy snapshots**. At least one rename is already confirmed between snapshot and 1.0.0 (`PedroCommands.turn` → `PedroCommands.turnTo`). Treat reference-repo code as structural guidance, not as a copy-paste source.

## Ivy 1.0.0 API reference

Verified from the `com.pedropathing:ivy:1.0.0` AAR source.

### Package tree

```
com.pedropathing.ivy
├── Command            (interface)
├── CommandBuilder     (class, implements Command)
├── Scheduler          (final class, all-static)
├── behaviors/
│   ├── BlockedBehavior     (enum: QUEUE, CANCEL)
│   ├── ConflictBehavior    (enum: OVERRIDE, QUEUE, CANCEL)
│   ├── EndCondition        (enum: NATURALLY, INTERRUPTED, SUSPENDED)
│   └── InterruptedBehavior (enum: END, SUSPEND)
├── commands/
│   ├── Branch, Commands, Conditional, Lazy, Match
├── groups/
│   ├── Deadline, Groups, Loop, Parallel, Race, Repeat, Sequential
└── pedro/
    ├── Follow, Hold, PedroCommands, Turn
```

**Notable absence:** no `bindings/`, `triggers/`, `input/`, or `gamepad/` package.

### Scheduler (global static)

```java
Scheduler.schedule(Command...);   // schedule one or many
Scheduler.execute();              // tick once per loop
Scheduler.cancel(Command);
Scheduler.reset();                // call in OpMode init
Scheduler.isRunning(Command);
Scheduler.isScheduled(Command);
```

Priority arbitration is the scheduler's job. When a new command's requirements conflict:

- Active conflicting command has **higher priority** → new command is *blocked* (queued or cancelled per its `blockedBehavior`).
- Active conflicting command has **equal priority** → resolved per new command's `conflictBehavior` (OVERRIDE / QUEUE / CANCEL).
- Active conflicting command has **lower priority** → it is interrupted (END or SUSPEND per its `interruptedBehavior`); new command starts.

### Command lifecycle

`start()` → repeated `execute()` until `done()` returns true → `end(EndCondition)`. Default `interruptedBehavior` is `END`; SUSPEND moves a command to a suspended deque and it resumes when its requirements free up.

### CommandBuilder (fluent)

```java
new CommandBuilder()
    .setStart(Runnable)
    .setExecute(Runnable)
    .setDone(BooleanSupplier)
    .setEnd(Consumer<EndCondition>)
    .requiring(Object... requirements)   // note: requiring, NOT requires
    .setPriority(int)
    .setInterruptedBehavior(InterruptedBehavior.END | SUSPEND)
    .setBlockedBehavior(BlockedBehavior.QUEUE | CANCEL)
    .setConflictBehavior(ConflictBehavior.OVERRIDE | QUEUE | CANCEL)
```

Defaults: `priority=0`, `interruptedBehavior=END`, `blockedBehavior=CANCEL`, `conflictBehavior=OVERRIDE`. CommandBuilder *is* a Command — no `.build()` call needed.

### Command interface defaults (chaining)

```java
cmd.schedule();
cmd.cancel();
cmd.isScheduled();
cmd.then(cmd2, ...)        // sequential
cmd.with(cmd2, ...)        // parallel
cmd.raceWith(cmd2, ...)    // race
cmd.until(condition)       // race with waitUntil
cmd.unless(condition)      // NOOP if condition true, else cmd
cmd.proxy()                // schedule-and-await wrapper
```

### Pedro commands (verified signatures)

```java
PedroCommands.follow(Follower, PathChain)
PedroCommands.follow(Follower, PathChain, double maxPower)
PedroCommands.follow(Follower, PathChain, boolean holdEnd)
PedroCommands.follow(Follower, PathChain, boolean holdEnd, double maxPower)
PedroCommands.hold(Follower)
PedroCommands.hold(Follower, Pose)
PedroCommands.hold(Follower, Pose, PathConstraints)
PedroCommands.turnTo(Follower, double radians)
PedroCommands.turnTo(Follower, double radians, PathConstraints)
```

All return `CommandBuilder`, so `follow(...).then(score).requiring(arm)` composes naturally.

### Factories (verified from 1.0.0 source)

All confirmed against `com.pedropathing.ivy.commands.Commands` and `com.pedropathing.ivy.groups.Groups` source in the `com.pedropathing:ivy:1.0.0` AAR.

`Commands.*`:

```java
Commands.instant(Runnable)                  // run once, done immediately
Commands.infinite(Runnable)                 // run each tick forever (done=false)
Commands.waitMs(double milliseconds)        // delay; NOTE: name is waitMs not wait
Commands.waitUntil(BooleanSupplier)         // done when supplier returns true
Commands.conditional(BooleanSupplier decider, Command ifTrue, Command ifFalse)
Commands.branch(LinkedHashMap<BooleanSupplier, Command>)   // first match wins
Commands.lazy(Supplier<Command>)            // defer command creation to start time
Commands.match(Supplier<T extends Enum<T>>, EnumMap<T, Command>)   // enum switch
Commands.onInterrupt(Runnable callback)     // forever-running; callback on interrupt only
```

`Groups.*`:

```java
Groups.sequential(Command...)
Groups.parallel(Command...)
Groups.race(Command...)                     // ends when first finishes
Groups.deadline(Command deadline, Command... others)   // ends when deadline finishes
Groups.repeat(Command, int iterations)
Groups.repeat(Command, IntSupplier iterationsSupplier)  // dynamic count at start time
Groups.loop(Command)                        // re-run command forever until interrupted
```

All factories return `CommandBuilder` so chaining (`.then`, `.with`, `.until`, `.unless`, `.proxy`) works.

`Groups.loop(Command)` is distinct from `Commands.infinite(Runnable)`: `loop` re-runs an entire command lifecycle (start → execute → done → end → start → ...); `infinite` calls a `Runnable` each tick without any lifecycle.

`Commands.match` is the cleanest replacement for our existing `switch (robotState)` / `switch (alliance)` code in places like `Robot.initializeForAuto/TeleOp` and lighting state dispatch.

## NextFTC → Ivy translation table

| NextFTC | Ivy 1.0.0 |
|---|---|
| `extends NextFTCOpMode` | `extends LinearOpMode`; manual `Scheduler.execute()` loop |
| `BindingsComponent` / `SubsystemComponent` / `BulkReadComponent` | None — manual `bindings.update()`, no subsystem registration, manual `LynxModule.clearBulkCache()` |
| `implements Subsystem` + `periodic()` | Plain class; periodic logic becomes a priority-0 `infinite` command scheduled in init |
| `extends Command` (override start/execute/isFinished/end) | `new CommandBuilder().setStart(...).setExecute(...).setDone(...).setEnd(...)` |
| `new SequentialGroup(a, b, c)` | `Groups.sequential(a, b, c)` or `a.then(b).then(c)` |
| `new ParallelDeadlineGroup(deadline, others...)` | `Groups.deadline(deadline, others...)` |
| `InstantCommand(r)` | `Commands.instant(r)` |
| `Delay(ms)` | `Commands.waitMs(ms)` |
| (no equivalent) | `Commands.match(stateSupplier, enumMap)` — for replacing `switch (robotState)` blocks |
| (no equivalent) | `Commands.lazy(() -> buildCmd(currentPose))` — for commands that need state captured at start, not schedule |
| `BindingManager` / `Button` / `Range` | `IvyBindings` shim (this doc) |
| `dev.nextftc.extensions.pedro.FollowPath` | `PedroCommands.follow(follower, pathChain)` |
| `dev.nextftc.extensions.pedro.PedroComponent` | Drop — `Robot` holds the `Follower` instance directly |
| `GamepadEx` | Raw `gamepad1` / `gamepad2` + SDK 11.1 `wasJustPressed()` family |
| Default command (per subsystem) | Priority-0 `infinite` command scheduled in init; higher-priority commands preempt; set default's `interruptedBehavior=SUSPEND` so it resumes |

## The bindings shim

New file: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/IvyBindings.java`.

Polls a list of `BooleanSupplier` triggers each loop, fires `Command.schedule()` / `Command.cancel()` on edges. Reusable across teleop OpModes.

```java
package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ivy.Command;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public final class IvyBindings {
    private final List<Trigger> triggers = new ArrayList<>();

    public Trigger when(BooleanSupplier condition) {
        Trigger t = new Trigger(condition);
        triggers.add(t);
        return t;
    }

    public void update() {
        for (Trigger t : triggers) t.poll();
    }

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
```

### Usage example (post-migration `DriverBindings`)

```java
bindings.when(() -> gamepad1.b).onTrue(launcherCommands.fire());
bindings.when(() -> gamepad1.right_trigger > 0.5).whileTrue(intakeCommand);
bindings.when(gamepad1::aWasPressed).onTrue(toggleAimCommand);   // SDK 11.1 edge prim
```

### OpMode loop shape

```java
while (opModeIsActive()) {
    for (LynxModule hub : hubs) hub.clearBulkCache();
    bindings.update();
    Scheduler.execute();
    telemetry.update();
}
```

## Per-file migration checklist

Files counted from current `claude/setup-repo-navigation-PQzdy` branch (2026-05-17). Subject to drift; recount when starting.

### Subsystems (5 files) — mechanical

- `subsystems/DriveSubsystem.java`
- `subsystems/VisionSubsystemLimelight.java`
- `subsystems/LauncherSubsystem.java`
- `subsystems/IntakeSubsystem.java`
- `subsystems/LightingSubsystem.java`

Per file:
1. Remove `implements dev.nextftc.core.subsystems.Subsystem`.
2. Move `periodic()` body to a private method, then schedule a priority-0 `infinite` command in `Robot.init()` that calls it with `interruptedBehavior=SUSPEND`.
3. The subsystem instance itself becomes the requirement key (`.requiring(driveSubsystem)`).

### Commands (~18 files) — mechanical

- `commands/IntakeCommands/IntakeCommand.java`
- `commands/LauncherCommands/*` (5 files)
- `commands/DriveCommands/*` (6 files including `AimAndDrive*`, `DefaultDriveCommand`, `TryRelocalizeForShotCommand`)
- `opmodes/Autos/Commands/*` (5 files)

Per file:
1. Replace `extends dev.nextftc.core.commands.Command` with a method returning `Command` (built via `CommandBuilder`).
2. `start/execute/isFinished/end` → `setStart/setExecute/setDone/setEnd`.
3. Add `.requiring(subsystem).setPriority(N)` as appropriate.
4. `SequentialGroup` / `ParallelDeadlineGroup` / `InstantCommand` / `Delay` swap per translation table.

### Command factories — light edits

- `commands/IntakeCommands/IntakeCommands.java` — swap group constructors for `Groups.*` calls.
- `commands/LauncherCommands/LauncherCommands.java` — same.
- `util/FollowPathBuilder.java` — replace `new FollowPath(...)` with `PedroCommands.follow(...)`. Confirm whether any call sites pass a bare `Path`; if so, wrap in a single-element `PathChain`.

### OpModes — most thinking required

- `opmodes/DecodeTeleOp.java` — change base class to `LinearOpMode`; add bulk-cache / bindings.update / Scheduler.execute loop; rewrite component registration as plain construction.
- `opmodes/Autos/DecodeAutonomousClose.java`, `DecodeAutonomousFar.java`, `DecodeAutonomousCloseThreeAtOnce.java`, `DecodeAutonomousCloseTogether.java`, `DecodeAutonomousFarThreeAtOnce.java`, `DecodeAutonomousFarTogether.java` — `extends LinearOpMode`; build the auto command tree; `Scheduler.schedule(autoCmd)` after `waitForStart()`; loop `Scheduler.execute()`.
- `opmodes/Calibration/DiagnoseMegaTag2.java` — light port.

### Bindings (2 files) — rewrite against shim

- `bindings/DriverBindings.java`
- `bindings/OperatorBindings.java`

Replace `BindingManager`/`Button`/`Range` calls with `IvyBindings.when(...).onTrue/onFalse/whileTrue(...)`. SDK 11.1 `wasJustPressed()` helpers are useful for one-shot bindings.

### Untouched

- `pedroPathing/Constants.java`
- `pedroPathing/Tuning.java` (still uses `SelectableOpMode` from `com.pedropathing:telemetry:1.0.0`; Ivy doesn't affect it)
- `util/PoseTransforms.java`, `AprilTagPoseUtil.java`, `FieldConstants.java`, `RobotState.java`, `Alliance.java`, `RobotMode.java`, `ArtifactColor.java` (note: `util/PoseFusion.java` deleted by fusion branch; `pedroPathing/FusionLocalizer.java` added there, framework-agnostic, survives migration unchanged)
- `telemetry/*`

### Dependency changes (final state, TeamCode/build.gradle)

```groovy
dependencies {
    implementation project(':FtcRobotController')

    // Pedro Pathing + Ivy
    implementation('com.pedropathing:ftc:2.1.2')
    implementation('com.pedropathing:telemetry:1.0.0')   // for Tuning.java
    implementation('com.pedropathing:ivy:1.0.0')

    // Panels
    implementation('com.bylazar:fullpanels:1.0.12')

    // FTC Dashboard
    implementation('com.acmerobotics.dashboard:dashboard:0.5.1') {
        exclude group: 'org.firstinspires.ftc'
    }

    // REMOVED:
    // implementation 'dev.nextftc:ftc:1.1.0'
    // implementation 'dev.nextftc:hardware:1.1.0'
    // implementation 'dev.nextftc:bindings:1.0.1'
    // implementation 'dev.nextftc.extensions:pedro:1.0.0'
}
```

If adding Sloth at the same time: replace `com.acmerobotics.dashboard:dashboard:0.5.1` with `com.acmerobotics.slothboard:dashboard:0.2.4+0.5.1`, replace `com.bylazar:fullpanels:1.0.12` with `com.bylazar.sloth:fullpanels:0.2.4+1.0.12`, add Sloth Load plugin per `Dairy-Foundation/Sloth` README.

## Known gotchas

1. **`PedroCommands.turn` was renamed to `turnTo` in 1.0.0.** Reference-repo code (on snapshot Ivy) uses `turn(...)`; that signature does not exist on the public artifact. Verified against the 1.0.0 AAR.
2. **`Commands.wait(double)` does not exist in 1.0.0.** The 1.0.0 name is `Commands.waitMs(double milliseconds)`. Baron's `22131-Decode` uses `Commands.wait(...)` because it builds against `ivy:0.0.1-LOCAL` — a personal local build, not the public artifact. MOEbo's snapshot and the 1.0.0 release both use `waitMs`.
3. **No default-command concept.** Use priority-0 `Commands.infinite(...)` with `interruptedBehavior=SUSPEND`; higher-priority commands preempt and the default resumes when they finish.
4. **`PedroCommands.follow` only takes `PathChain`, not `Path`.** Wrap any bare `Path` in a single-element `PathChain` at the call site.
5. **`requiring(...)` not `requires(...)`.** Easy typo to make in mechanical porting.
6. **`Scheduler` is process-global and static.** Call `Scheduler.reset()` in OpMode init or state from a prior OpMode (or test) will leak.
7. **`BulkReadComponent` has no Ivy equivalent.** Manually iterate `hardwareMap.getAll(LynxModule.class)` and call `clearBulkCache()` each loop (Baron's pattern in `Robot.clearCaches()`).
8. **0.0.1-SNAPSHOT, 0.0.1-LOCAL, and 1.0.0 differ.** Don't assume reference-repo code compiles unchanged against 1.0.0. Confirmed renames so far: `turn` → `turnTo` (snapshot → 1.0.0). Confirmed non-renames: `waitMs` exists in both. Baron's `-LOCAL` uses some method names (`wait`) that exist in neither the snapshot nor 1.0.0 — treat his code as structural reference only.

## PR sequencing

Each PR ends with the codebase still buildable and runnable on the robot. Any PR after the first can be safely reverted without breaking the ones before it.

### PR 1: dependency + shim + tests (no production changes)

**Adds:**
- `implementation 'com.pedropathing:ivy:1.0.0'` to `TeamCode/build.gradle`.
- `util/IvyBindings.java` (full source above).
- `TeamCode/src/test/java/.../IvyBindingsTest.java` — JUnit 4 tests for `onTrue` / `onFalse` / `whileTrue` edge logic using a mutable boolean and a counter-incrementing `Command`.

**Touches no production code. Existing NextFTC code continues to compile and run.**

Acceptance:
- `./gradlew :TeamCode:assembleDebug` succeeds.
- `./gradlew :TeamCode:test --tests IvyBindingsTest` passes.
- No behavior change on the robot.

### PR 2: port one subsystem end-to-end (proof of concept)

**Recommended target: `LightingSubsystem`** (smallest, lowest stakes, no Pedro coupling).

**Changes:**
- `LightingSubsystem.java` — drop `implements Subsystem`.
- Any lighting-related commands — rebuilt against `CommandBuilder`.
- `Robot.initializeForTeleOp()` (and Auto variant) — schedule lighting's periodic as an Ivy command.
- `DecodeTeleOp.java` — partial: lighting handling only; everything else still NextFTC.

Acceptance:
- TeleOp runs on the robot.
- Lighting patterns respond to commands correctly.
- All other subsystems unaffected.

**Decision gate:** if PR 2 reveals API friction or scheduler edge cases we didn't anticipate, this is the cheapest place to abandon. Total cost: one shim file + one ported subsystem.

### PR 3: port remaining subsystems + commands

- All other subsystems (`Intake`, `Launcher`, `Vision`, `Drive`).
- All commands in `commands/**`.
- Command factories (`IntakeCommands.java`, `LauncherCommands.java`).
- `util/FollowPathBuilder.java`.

OpModes and bindings still use NextFTC bridging code (temporary adapters) so the codebase stays runnable. This PR is the largest in line count but mechanical.

Acceptance:
- TeleOp runs.
- At least one auto runs end-to-end.

### PR 4: port TeleOp + bindings

- `bindings/DriverBindings.java`, `bindings/OperatorBindings.java` rewritten against `IvyBindings`.
- `DecodeTeleOp.java` switched to `LinearOpMode` base.
- Remove temporary bridging adapters from PR 3.

Acceptance:
- Full match-length TeleOp on the robot.
- Loop times within the budget noted in `docs/loop-timing-analysis.md`.

### PR 5: port autos, remove NextFTC

- All auto OpModes ported.
- Remove `dev.nextftc:*` lines from `TeamCode/build.gradle`.
- Delete now-unused NextFTC bridge code.
- Update `CLAUDE.md` references from NextFTC to Ivy.

Acceptance:
- All autos run on the robot.
- `./gradlew :TeamCode:dependencies` shows no `dev.nextftc:*` artifacts.

## Rollback

- Each PR is revertable in isolation.
- `master` retains the NextFTC implementation until PR 5 merges.
- If post-PR-5 issues surface in competition, revert PRs 5 → 4 → 3 → 2 → 1 in order, or branch from the last NextFTC commit.

## Multi-robot configuration (DECODE_19429 + DECODE_20245)

We have two physical robots sharing this codebase. The control hub identifies itself at runtime via `util/ControlHubIdentifierUtil.java`, and `util/RobotConfigs.java` dispatches to per-robot tuning constants via 11 `if ("DECODE_19429".equals(robotName))` getters. Configs that differ between robots: Pinpoint offsets, Mecanum PIDs, hood positions, feeder timing, aim-assist gains, lane sensor thresholds, distance→RPM curves, gate config.

The migration must preserve this. It's also a good opportunity to clean it up.

### Problems with the current pattern

- 11 nearly-identical getter methods doing string-equals comparisons.
- Adding a third robot would mean editing 11 getters plus all the config classes (each holds a `xxx19429` and `xxx20245` static field).
- Subsystems implicitly depend on `RobotState.getRobotName()` having been set before they read any config — temporal coupling that's only enforced by comments.
- The literal `"DECODE_19429"` appears 11+ times in one file. Rename risk.

### Proposed: `RobotProfile` value object

Replace `RobotConfigs` with a single immutable bundle, selected once at boot:

```java
public final class RobotProfile {
    public final LauncherFeederConfig feeder;
    public final LauncherHoodConfig hood;
    public final LauncherFlywheelConfig flywheel;
    public final LauncherTimingConfig timing;
    public final PinpointConstants pinpoint;
    public final FollowerConstants follower;
    public final MecanumConstants drive;
    public final DriveAimAssistConfig aimAssist;
    public final DriveFixedAngleAimConfig fixedAngleAim;
    public final DriveRightTriggerFixedAngleConfig rightTriggerFixedAngle;
    public final IntakeGateConfig gate;
    public final IntakeLaneSensorConfig.LanePresenceConfig lanePresence;
    public final CommandRangeConfig commandRange;

    private RobotProfile(/* all-args constructor */) { /* ... */ }

    public static RobotProfile forCurrent() {
        String name = RobotState.getRobotName();
        if ("DECODE_19429".equals(name)) return ROBOT_19429;
        return ROBOT_20245;  // default
    }

    public static final RobotProfile ROBOT_19429 = new RobotProfile(
        LauncherFeederConfig.feederConfig19429,
        LauncherHoodConfig.hoodConfig19429,
        /* ... 11 more args ... */);

    public static final RobotProfile ROBOT_20245 = new RobotProfile(/* ... */);
}
```

Subsystems take what they need explicitly at construction (Traffic Cones idiom):

```java
public LauncherSubsystem(HardwareMap hm,
                         LauncherFlywheelConfig flywheel,
                         LauncherHoodConfig hood,
                         LauncherFeederConfig feeder,
                         LauncherTimingConfig timing) { /* ... */ }
```

`Robot` wires it once:

```java
public Robot(HardwareMap hm) {
    ControlHubIdentifierUtil.setRobotName(hm, null);
    RobotProfile p = RobotProfile.forCurrent();
    launcher = new LauncherSubsystem(hm, p.flywheel, p.hood, p.feeder, p.timing);
    drive = new DriveSubsystem(hm, vision, p.drive, p.pinpoint, p.follower, p.aimAssist);
    intake = new IntakeSubsystem(hm, p.gate, p.lanePresence);
    /* ... */
}
```

### Wins

- Adding a third robot is **1 file edit** (one new `ROBOT_NNNNN` constant) instead of 11.
- Type-safe — no string compares scattered through config classes.
- Subsystems declare config dependencies explicitly. No hidden `RobotConfigs.getXxx()` calls inside subsystem code.
- Testable — pass any `RobotProfile` in tests.
- Plays nice with Ivy's plain-subsystem-class style; no framework coupling at all.

### Note on `Commands.match`

`Commands.match` is for *behavior* dispatch (returns a `Command`), not *config* dispatch. For selecting `RobotProfile` at construction time, use a plain Java switch / `forCurrent()` method. Don't try to force Ivy's `match` into the wrong job.

### Where this fits in PR sequencing

The `RobotProfile` refactor is **orthogonal to the Ivy migration** — it could happen before, during, or after. Two options:

- **Do it as PR 0** (before Ivy): clean refactor while staying on NextFTC. Smaller, safer changes per PR. Subsystems get explicit config deps before they're also asked to change framework.
- **Do it inside PR 3** (the big subsystem port): port subsystem to Ivy *and* take its configs as constructor args in one shot. Fewer PRs but each PR is larger.

PR 0 is the more conservative path. Recommended.

## Open items to verify before starting

1. ~~`com.pedropathing.ivy.commands.Commands` — paste source, confirm factory names.~~ **Done 2026-05-17.** All factories verified.
2. ~~`com.pedropathing.ivy.groups.Groups` — paste source, confirm signatures.~~ **Done 2026-05-17.** All signatures verified including the bonus `repeat(int)`, `repeat(IntSupplier)`, `loop(Command)`.
3. ~~Whether NextFTC pulls FTC Dashboard or `com.bylazar:fullpanels` transitively at `implementation` scope.~~ **Done 2026-05-17.** Per `./gradlew :TeamCode:dependencies` on dev machine: NextFTC pulls only `org.jetbrains.kotlin:kotlin-stdlib` and intra-NextFTC modules. **No dashboard or panels transitives.** Sloth can be adopted without any `exclude group:` lines — just swap the direct `com.acmerobotics.dashboard:dashboard:0.5.1` for `com.acmerobotics.slothboard:dashboard:0.2.4+0.5.1` and `com.bylazar:fullpanels:1.0.12` for `com.bylazar.sloth:fullpanels:0.2.4+1.0.12`. Original "Sloth incompatibility" concern was unfounded.
4. ~~Whether `com.pedropathing:telemetry:1.0.0` can be dropped once `Tuning.java` is replaced with whatever the current Pedro 2.1.2 quickstart uses.~~ **Done 2026-05-17.** Pedro-Pathing/Quickstart's current `Tuning.java` (in the `Pedro-Pathing/Quickstart` repo) is identical to ours — same `@version 1.0, 6/26/2025` header, same `import com.pedropathing.telemetry.SelectableOpMode;`. Their `build.dependencies.gradle` still includes `implementation 'com.pedropathing:telemetry:1.0.0'`. Keep the artifact and the file. No change.
5. ~~Confirm `Commands.waitMs` argument unit.~~ **Done.** Javadoc says milliseconds explicitly.

## Style guide: 22131 Traffic Cones

`BaronClaps/22131-Decode` is the canonical Ivy reference codebase — Baron Henderson is both the Pedro Pathing author and an Ivy co-author, and 22131 is a top-1% global OPR team. This section captures patterns from their code worth adopting, plus a couple worth explicitly rejecting.

Confirmed by reading these files at 2026-05-17:
- `TeamCode/.../config/Robot.java`
- `TeamCode/.../config/command/CommandOpMode.java`
- `TeamCode/.../config/subsystem/Shooter.java`
- `TeamCode/.../opmode/Tele.java`
- `TeamCode/.../opmode/auto/Auto15.java`

### Package layout

Their tree:

```
config/
  Robot.java                   # service locator
  command/CommandOpMode.java   # 30-line OpMode base
  paths/Fast15.java, ...       # auto path classes
  pedro/Constants.java         # Pedro config (same as our pedroPathing/)
  subsystem/                   # Drivetrain, Flipper, Intake, Shooter, Turret
  util/Alliance.java
  vision/Limelight.java, ArtifactFetcher.java, opencv/
opmode/
  auto/Auto15.java, Blue15.java, Red15.java, Auto12.java, Blue12.java, Red12.java
  test/DexTest.java, TurretTest.java
  Tele.java, BlueTele.java, RedTele.java
```

Pattern: **everything robot-related lives under `config/`**. OpModes are the only top-level package. Subsystems are flat (`config/subsystem/Shooter.java`, not `subsystems/shooter/Shooter.java` + config subdirectory like we do).

Our `subsystems/launcher/` + `subsystems/launcher/config/` split is more elaborate than theirs. Reasonable to flatten during the Ivy migration if we want — but not required.

### Robot.java as a service locator

Their `Robot` holds every subsystem as a `public final` field with a **single-letter name**:

```java
public final Intake i;
public final Limelight l;
public final Shooter s;
public final Flipper g;
public final Turret t;
public final Follower f;
public Alliance a;
```

…and is accessed directly: `r.i.in()`, `r.s.atTarget()`, `r.f.getPose()`. No getters.

**Adopt:** the service-locator pattern. Single owning class holding all subsystems, passed everywhere. We already do this; just simpler.

**Don't adopt:** single-letter field names. They scale poorly for a team with multiple programmers. `r.intake`, `r.shooter`, etc. read better and survive grep. (Baron has the luxury of writing most of his own code; we don't.)

### Subsystems are plain classes

No `implements Subsystem`, no base class, no framework registration. Constructor takes `HardwareMap`. Each subsystem has a `periodic()` method that `Robot.periodic()` calls.

```java
public class Shooter {
    private DcMotorEx l, r;
    public Shooter(HardwareMap hardwareMap) { ... }
    public void periodic() { ... }   // called from Robot.periodic()
}
```

Matches the Ivy translation table above — drop `implements Subsystem`, no replacement needed.

### Subsystem methods return commands

This is the **most important pattern to adopt.** Subsystems own their command factories:

```java
public class Shooter {
    public CommandBuilder toggle() { return Commands.instant(this::shooterToggle); }
    public CommandBuilder near()   { return Commands.instant(this::shootNear); }
    public CommandBuilder far()    { return Commands.instant(this::shootFar); }
}
```

Bindings then read like English:

```java
bindings.when(gamepad1::aWasPressed).onTrue(r.s.toggle());
bindings.when(gamepad1::bWasPressed).onTrue(r.s.near());
```

Compare to our current `commands/LauncherCommands/LauncherCommands.java` factory class. The Traffic Cones pattern eliminates that file: the factory methods live on the subsystem itself.

**Adopt for the migration:** move our `commands/<Subsystem>Commands/*.java` factories into the subsystems themselves. The `commands/` package shrinks toward zero (only true cross-subsystem composite commands remain). Big code-organization win.

### Composite commands live on Robot

Cross-subsystem macros are methods on `Robot` returning a built command:

```java
public CommandBuilder shoot() {
    return sequential(
        g.down(),
        i.in(),
        Commands.waitUntil(s::atTarget),
        Commands.wait(200.0),
        g.up(),
        Commands.wait(200.0),
        g.down(),
        ...
    );
}

public CommandBuilder intake() {
    return sequential(g.down(), i.in(), Commands.wait(500.0));
}
```

Auto code then reads as a high-level sequence:

```java
PedroCommands.follow(r.f, p.next())
    .with(Commands.waitUntil(() -> r.f.getCurrentTValue() >= 0.5).then(r.shoot()))
```

**Adopt:** put composite/cross-subsystem commands (current `LauncherCoordinator`, `AimAndDrive*`, etc.) as methods on `Robot` returning `CommandBuilder`. Our `opmodes/Autos/Commands/*.java` would mostly become `Robot.threeAtOnce()`, `Robot.together()`, etc.

### CommandOpMode base (30 lines)

```java
public abstract class CommandOpMode extends OpMode {
    public void reset() { Scheduler.reset(); }
    public void schedule(Command... commands) { Scheduler.schedule(commands); }
    @Override public void init() {}
    @Override public void loop() { Scheduler.execute(); }
    public void stop() { reset(); }
}
```

That's it. The base extends iterative `OpMode` (not `LinearOpMode`), `init()` is overridable by subclasses to do command setup + `schedule(...)`, `loop()` just ticks the scheduler. No `runOpMode()` / `waitForStart()` ceremony.

**Adopt:** copy `CommandOpMode` verbatim into our codebase. Add `bindings.update()` to our copy's `loop()` if we want bindings to be auto-ticked. (Their codebase doesn't have bindings — they handle gamepad input imperatively per loop. See next section.)

### Iterative OpMode + imperative gamepad

Their `Tele.java` extends `CommandOpMode` and handles input directly in `loop()`:

```java
@Override public void loop() {
    r.periodic();
    if (gamepad1.bWasPressed()) shoot = !shoot;
    if (gamepad1.rightBumperWasPressed())
        intakeOn = (intakeOn == 1) ? 0 : 1;
    if (gamepad1.aWasPressed()) {
        if (manualFlip) { r.g.toggle(); ... }
        else autoFlipping = true;
    }
    // ... 100 more lines of conditionals ...
}
```

`Scheduler.execute()` is called automatically by `CommandOpMode.loop()` at the end (well, actually their `Tele.java` doesn't seem to call `super.loop()` — they may rely on the framework's iterative scheduler).

**Don't adopt as-is.** This works for a 2-person team where one person writes all the teleop code. With multiple programmers and existing declarative `DriverBindings.java` / `OperatorBindings.java` patterns, we want the `IvyBindings` shim. Our shim *uses* SDK 11.1 `wasJustPressed` under the hood but keeps the declarative front-end.

A hybrid is fine: use `IvyBindings` for stable button → command mappings; use raw `if (gamepad.xWasPressed())` blocks inside `loop()` for one-off, in-development behavior. Same as we do today.

### Path "feeder" pattern for autos

Instead of declaring every `PathChain` upfront with a meaningful name, Baron uses a stateful path class that yields paths in order:

```java
Fast15 p = new Fast15(r);
r.f.setStartingPose(p.start);
// ...
PedroCommands.follow(r.f, p.next())   // path 1
PedroCommands.follow(r.f, p.next())   // path 2
PedroCommands.follow(r.f, p.next())   // path 3
```

The `Fast15` class internally holds an index and a list, advancing on each `next()`.

**Mixed verdict:** terse, but path identity is lost — you can't tell what `p.next()` does without counting calls. Our `FollowPathBuilder` + named paths in autos is more verbose but more readable. **Don't adopt** unless we run into auto-OpMode files growing past a screen length.

### Telemetry as an infinite command

```java
schedule(
    Commands.infinite(r::periodic),
    Commands.infinite(() -> {
        telemetry.addData("Pose: ", r.f.getPose());
        telemetry.addData("Shooter At Target: ", r.s.atTarget());
        ...
        telemetry.update();
    }),
    sequential(/* the actual auto */)
);
```

`Robot.periodic()` and telemetry both ride priority-0 infinite commands that run forever in parallel with the auto sequence.

**Adopt:** this is a clean way to handle the "always-on" stuff. Our `RobotStatusLogger.logStatus(...)` call can become an Ivy `infinite` command instead of being called manually in the OpMode loop.

### `@Config` and FTC Dashboard

Their subsystems are `@Config`-annotated with tunable `public static` fields:

```java
@Config
public class Shooter {
    public static double kS = 0.08, kV = 0.00039, kP = 0.01;
    public static double near = 1200;
    public static double far = 1400;
    ...
}
```

Same pattern we use in our `<Subsystem>.<Subsystem>Config` nested classes. They use top-level `@Config` instead of nested; either works.

**No change needed.** Our nested-config pattern is fine; the migration doesn't affect it.

### What our codebase already does better

For honesty's sake:

- We have proper telemetry levels (`TelemetryLevel.MATCH/DEBUG` gated at compile time) — Baron just always-logs.
- We have `Constants.HardwareNames.*` for centralized hardware names — Baron hardcodes strings (`"sl"`, `"sr"` in `Shooter.java`).
- We have Kalman-filter `FusionLocalizer` + AprilTag relocalization tooling — they have a simpler Limelight setup.
- Our autos use Pedro Path Generator-compatible exports for visualization — they don't seem to.

The migration should preserve all of these.

### API names: Traffic Cones vs 1.0.0

Baron's `22131-Decode` builds against `com.pedropathing:ivy:0.0.1-LOCAL` — a personal local build, not the public 1.0.0 artifact. One difference is confirmed:

- Baron uses **`Commands.wait(double)`**; 1.0.0 has **`Commands.waitMs(double milliseconds)`**.

When porting Traffic Cones patterns, mechanically substitute `wait` → `waitMs`. Everything else in their code (`Commands.waitUntil`, `Commands.infinite`, `Commands.instant`, `PedroCommands.follow`, `.then(...)`, `.with(...)`, `Groups.sequential`) matches 1.0.0 verbatim.

Confirmed structural patterns (independent of method names):
- `cmd.with(other)` / `cmd.then(other)` chaining — used heavily in their autos.
- Subsystem methods return `CommandBuilder` — the central pattern to adopt.
- `Robot.shoot()` / `Robot.intake()` composite commands built via `Groups.sequential`.
- `Commands.infinite(robot::periodic)` to keep subsystem state machines ticking in parallel with mission commands.

## References

- Ivy installation: https://pedropathing.com/docs/ivy (gated to network; user paste required)
- Sloth README: `Dairy-Foundation/Sloth` on GitHub
- Reference Ivy projects:
  - `BaronClaps/22131-Decode` (Traffic Cones, Pedro/Ivy authors — canonical reference)
  - `MOEbo-Sapiens/MOEbo-Sapiens-Decode` (uses pre-release Ivy snapshot; some API names stale)
- FTC SDK 11.1 release notes: gamepad trigger edge detection added (see CLAUDE.md history)
