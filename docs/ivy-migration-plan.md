# Ivy Migration Plan: NextFTC → Pedro Pathing Ivy

Status: **Proposal, not started.** Generated 2026-05-17 from a live read of `com.pedropathing:ivy:1.0.0` source in External Libraries and inspection of two reference repos already running Ivy.

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

### Factories (names to verify before PR 2)

Inferred from reference repo usage; **verify against `Commands.kt` and `Groups.kt` source** before relying on them in production:

```
Commands.instant(Runnable)
Commands.infinite(Runnable)
Commands.waitMs(double)
Commands.waitUntil(BooleanSupplier)
Commands.conditional(BooleanSupplier, Command ifTrue, Command ifFalse)
Groups.sequential(Command...)
Groups.parallel(Command...)
Groups.race(Command...)
Groups.deadline(Command deadline, Command... others)   // unverified
```

## NextFTC → Ivy translation table

| NextFTC | Ivy 1.0.0 |
|---|---|
| `extends NextFTCOpMode` | `extends LinearOpMode`; manual `Scheduler.execute()` loop |
| `BindingsComponent` / `SubsystemComponent` / `BulkReadComponent` | None — manual `bindings.update()`, no subsystem registration, manual `LynxModule.clearBulkCache()` |
| `implements Subsystem` + `periodic()` | Plain class; periodic logic becomes a priority-0 `infinite` command scheduled in init |
| `extends Command` (override start/execute/isFinished/end) | `new CommandBuilder().setStart(...).setExecute(...).setDone(...).setEnd(...)` |
| `new SequentialGroup(a, b, c)` | `Groups.sequential(a, b, c)` or `a.then(b).then(c)` |
| `new ParallelDeadlineGroup(deadline, others...)` | `Groups.deadline(deadline, others...)` (verify) or `new Deadline(...)` |
| `InstantCommand(r)` | `Commands.instant(r)` |
| `Delay(ms)` | `Commands.waitMs(ms)` |
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
- `util/PoseFusion.java`, `PoseTransforms.java`, `AprilTagPoseUtil.java`, `FieldConstants.java`, `RobotState.java`, `Alliance.java`, `RobotMode.java`, `ArtifactColor.java`
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

1. **`PedroCommands.turn` was renamed to `turnTo` in 1.0.0.** Reference-repo code (on snapshot Ivy) uses `turn(...)`; that signature does not exist on the public artifact.
2. **No default-command concept.** Use priority-0 `Commands.infinite(...)` with `interruptedBehavior=SUSPEND`; higher-priority commands preempt and the default resumes when they finish.
3. **`PedroCommands.follow` only takes `PathChain`, not `Path`.** Wrap any bare `Path` in a single-element `PathChain` at the call site.
4. **`requiring(...)` not `requires(...)`.** Easy typo to make in mechanical porting.
5. **`Scheduler` is process-global and static.** Call `Scheduler.reset()` in OpMode init or state from a prior OpMode (or test) will leak.
6. **`BulkReadComponent` has no Ivy equivalent.** Manually iterate `hardwareMap.getAll(LynxModule.class)` and call `clearBulkCache()` each loop (Baron's pattern in `Robot.clearCaches()`).
7. **The 0.0.1 → 1.0.0 jump renamed at least one method.** Don't assume reference-repo code compiles unchanged against 1.0.0. Verify against the AAR source before porting any snippet.
8. **Two `Commands.*` / `Groups.*` factory names are unverified** at time of writing. Open the Kotlin source for `Commands` and `Groups` in External Libraries and confirm before PR 2.

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

## Open items to verify before starting

1. `com.pedropathing.ivy.commands.Commands` — paste source, confirm factory names (`instant`, `infinite`, `waitMs`, `waitUntil`, `conditional`).
2. `com.pedropathing.ivy.groups.Groups` — paste source, confirm `sequential`, `parallel`, `race`, `deadline`, `loop`, `repeat` signatures.
3. Whether NextFTC pulls FTC Dashboard or `com.bylazar:fullpanels` transitively at `implementation` scope (matters only if Sloth is added before NextFTC is removed). Run on dev machine: `./gradlew :TeamCode:dependencies --configuration debugRuntimeClasspath | grep -E "acmerobotics\.dashboard|com\.bylazar"`.
4. Whether `com.pedropathing:telemetry:1.0.0` can be dropped once `Tuning.java` is replaced with whatever the current Pedro 2.1.2 quickstart uses (orthogonal to Ivy migration but worth checking at the same time).

## References

- Ivy installation: https://pedropathing.com/docs/ivy (gated to network; user paste required)
- Sloth README: `Dairy-Foundation/Sloth` on GitHub
- Reference Ivy projects: `MOEbo-Sapiens/MOEbo-Sapiens-Decode`, `BaronClaps/22131-Decode`
- FTC SDK 11.1 release notes: gamepad trigger edge detection added (see CLAUDE.md history)
