# Saturday Practice Plan

Drafted from Dustin's 3:04 PM mentor chat outline. Intent: review with mentors Friday/Saturday and pick what to attack first.

Branch with all of today's pre-Saturday work: **`testCode`** (commit `e1b0be2`).

## TL;DR priority order

1. **Verify today's package upgrade + new color-sensor driver on hardware.** Gate everything else on this.
2. **Tune flywheel idle RPM up.** Highest-ROI item on the list. Pure dashboard config change.
3. **Profile the auto for between-shot dead time.** Several known suspects (below).
4. Pick from auto review / USB cable / lighting driver with remaining time.

---

## What's already done on `testCode` (untested on robot)

These commits were pushed today and need hardware verification before they can be trusted:

| Change | Files | Why |
|---|---|---|
| FTC SDK 11.0.0 → 11.1.0 | `build.dependencies.gradle` | Routine SDK upgrade |
| Pedro 2.0.4 → 2.1.2 | `TeamCode/build.gradle` | Available update |
| NextFTC ftc/hardware 1.0.1 → 1.1.0 | `TeamCode/build.gradle` | Available update |
| fullpanels 1.0.11 → 1.0.12 | `TeamCode/build.gradle` | Available update |
| appcompat 1.2.0 → 1.7.1 | `build.dependencies.gradle` | Available update |
| Added Gradle versions plugin | `build.gradle` | So we can run `./gradlew dependencyUpdates -Drevision=release` in the future |
| Fixed `getI2cDeviceSynch` → `getDeviceClient` | `IntakeSubsystem.java` | Required by 11.1.0 (and was actually broken in 11.0.0 too — the May 6 REPEAT-window commit never compiled cleanly) |
| **New `FastColorSensor` driver** | `subsystems/intake/FastColorSensor.java` | Bulk 14-byte I2C read of REV V3 instead of multiple short reads. Implements `NormalizedColorSensor` + `DistanceSensor` + `SwitchableLight` so the rest of the code doesn't change. |
| Robot XMLs switched lane sensors to `<FastColorSensor>` | `DECODE_Robot_Config19429.xml`, `DECODE_Robot_Config20245.xml` | Routes color sensors through the new driver |
| Removed dead `applyRepeatReadWindow` | `IntakeSubsystem.java` | Cleanup; FastColorSensor sets the REPEAT window itself |

## P1 — Verify on hardware (gate everything below this)

Pull on the practice computer if you haven't:
```
git fetch && git checkout testCode
```

For each robot you bring up:
```
./gradlew :FtcRobotController:installDebug
.\deploy_config19429.bat        (or deploy_config20245.bat)
```

Then in the Driver Station: **Configure Robot → Activate** the freshly-pushed config.

Smoke checks, in order:
1. OpMode init — no `IllegalArgumentException` from `hardwareMap.get(...)`. If you see one, the DS config didn't pick up `FastColorSensor`.
2. **Color sensors:** put a known-color artifact at each lane. Sensor values and detected color should match what they were before the upgrade. (Same calibration constants are used; values should be ≈ identical.)
3. **Distance:** `intake/sample/<lane>/distance_cm` in dashboard should respond to artifact proximity.
4. **Drive / vision / launcher / intake actuation:** basic OpMode run.

**If anything is broken:** the rollback is one XML edit per robot — change `<FastColorSensor>` back to `<RevColorSensorV3>` and re-deploy. The Java code keeps working either way because both classes implement the same interfaces. If something deeper than that is broken, `git checkout` an earlier commit.

## P2 — Shot timing (the competitive win)

Reviewed the fire path; here's where the time actually goes.

### Where the time goes today

`LaunchInSequenceCommand` (operator press → first artifact out) walks through these stages:

1. **`SPINNING_UP` — 100 ms hard wait** (`LaunchInSequenceCommand:114`). Fires every shot, even when wheels are already spun up. Three auto shots = 300 ms baseline that's hard to justify.
2. **`WAITING_FOR_READY` — until `isLaneReady()` is true for at least one lane.** Driven by `Flywheel.isAtLaunch()` in `LauncherSubsystem.java:1059-1094`:
   - Fast path: current RPM within `flywheelConfig().parameters.rpmTolerance = 50` of target → immediate.
   - Otherwise: `minimalSpinUpMs = 500 ms` elapsed AND `currentRpm <= 1.1 × launchRpm` → OK.
   - Fallback: `fallbackReadyMs = 500 ms` elapsed regardless → OK.
   - Hard cap: 3 s timeout (`sequenceConfig.timeoutSeconds`) — fires anyway.
3. **`SHOTS_QUEUED` — actual firing.** Spacing inside a sequence is controlled by `LaunchInSequenceConfig`:
   - Same-color groups (PPG, GPP): **`shotSpacingMs = 525`** between groups.
   - Alternating (PGP): **`alternatingSpacingMs = 265`** between shots.
4. **Post-fire:** `launchHoldAfterFireMs = 500` keeps the wheel hot, `recoveryMs = 150` is the lane lockout before it's eligible to fire again.

### Current flywheel state

`LauncherFlywheelConfig` — all three lanes:
- `idleRpm = 1500` (left/center/right)
- `rpmTolerance = 50`
- `kS / kV / kP` — already tuned per robot, see `flywheelConfig19429` / `flywheelConfig20245`

`launchRpm` is set by `presetRangeSpinUp(LauncherRange.SHORT_AUTO, ...)` based on `LauncherRange` / `CommandRangeConfig` — need to confirm the actual SHORT_AUTO target during the practice session.

### Saturday lever order (do them in this order, retest after each)

**Lever 1 — raise `idleRpm` (dashboard, fully revertible).**
The fast path in `Flywheel.isAtLaunch()` is "within 50 RPM of target." If idle is much further below target than 50 RPM, the wheel has to actually accelerate before that check passes. Raising idle closer to launch RPM shortcuts most of the wait.

Process:
- Open FTC Dashboard → `LauncherFlywheelConfig.flywheelLeft.idleRpm` (and `flywheelCenter`, `flywheelRight`).
- Step up incrementally: 1500 → 2000 → 2500 → 3000. Stop a few hundred RPM below your launch target.
- Measure trigger-to-launch latency at each step. AdvantageScope replay against the `RobotState.packet` timestamps is the cleanest read; stopwatch on three trials per setting also fine.
- Stop raising when latency stops dropping, or when battery/wear/noise gets unreasonable.

**Lever 2 — drop the 100 ms `SPINNING_UP` floor and the 500 ms `minimalSpinUpMs` / `fallbackReadyMs` floors.**
If Lever 1 means the wheel is essentially always at launch RPM, those floors are wasted time. Try setting `minimalSpinUpMs = 150`, `fallbackReadyMs = 250`. The 100 ms in `LaunchInSequenceCommand` is hardcoded — search-and-replace if we want it lower.

Risk: if encoders drop out, the fallback timers are what save the shot. Don't drop them so low that a momentary encoder dropout fires before the wheel is up to speed.

**Lever 3 — `shotSpacingMs` (525) and `alternatingSpacingMs` (265).**
These are the gaps between firings within a single sequence. Drop in 50 ms steps until you see double-feeds or jams. Tradeoff: too tight and the gate / feeder can't recycle between shots.

### How to know it's enough

Today, three-shot sequence is *somewhere around* (estimate, not measured): 100 + spin-up + 0 + 525 + recovery ≈ 1.5 s minimum. If you can get to ~700-800 ms for three shots after these levers, that's a big competitive deal.

## P3 — Auto delays between movements and shots

`CloseThreeAtOnceCommand.create()` builds `SequentialGroup(path1, launch1, path2, launch2, ...)`. The first path correctly overlaps with `presetRangeSpinUp` via `ParallelDeadlineGroup`. Subsequent paths do NOT overlap with anything — the launch command runs, then the next path runs.

Suspects, in rough order of likelihood:

1. **Pedro path "arrived" criteria.** `withLinearHeadingCompletion(0.7)` finishes heading interp at 70% of path, but the path itself still needs to converge on the endpoint before the follower declares done. Pedro 2.1.2 (just upgraded) may have changed convergence behavior. Check Pedro 2.1.2 changelog for any path-completion-tolerance changes.
2. **The 100 ms `SPINNING_UP` floor fires on every launch** — even though wheels are still spun up between shots in auto (`spinDownAfterShot=false`). 100 ms × ~4 launches = 400 ms easy to recover.
3. **`canQueueSequence` waits for `isLaneReady`** but `recoveryMs = 150` keeps a lane "not ready" for 150 ms after each fire. Probably fine across shots, but worth confirming on the trace.
4. **Could `launchAccordingToMode` start in parallel with the *next* path?** Right now they're sequential. If we know the wheels stay hot and the gate/intake handle the next pickup independently, we could overlap. Likely a real architectural change, not a Saturday task — discuss with mentors.

### Suggested approach Saturday

- Run a normal close auto with `TelemetryLevel.DEBUG` and AdvantageScope recording.
- Open the replay, look at the gaps between "path completed" events and "first shot fired" events.
- Classify each gap:
  - Mostly "flywheel still spinning up" → fixed by P2.
  - Mostly "nothing visible happening" → paranoid timeout, probably the 100 ms floor.
  - Mostly "path follower still settling" → look at Pedro path config.
  - Mostly "vision relocalization" → ask whether we actually need a fresh AprilTag fix before each launch.

## P4 — Lower-priority items from the mentor message

- **Auto review** — open-ended. If P1-P3 leave time, sit with the path waypoints and decide if the routes themselves are good. The `CloseThreeAtOnceCommand` waypoints look settled already.
- **USB cable hookup** — Dustin to clarify what this is (Limelight USB Ethernet? Webcam?). Probably a 30-min exploratory.
- **Lighting driver** — `LightingSubsystem` exists; what's "the lighting driver" specifically? Update from the bylazar Panels lights plugin? Should clarify scope before committing time.

## Open questions for mentor discussion

1. What `launchRpm` do we actually use for SHORT_AUTO right now? That sets the ceiling on how high we can push `idleRpm`.
2. Is there a reason `minimalSpinUpMs = 500` was chosen, or was it a safety guess? Drop it freely if it was just a guess.
3. Is the 100 ms `SPINNING_UP` stage protecting against something specific, or is it dead weight?
4. Are we OK with the FastColorSensor driver being community code (technically ours now, but originally from FEsmondeWhite)? It's small and inspectable — no licensing concerns we know of.

## Rollback plan

Any of today's work giving us trouble at the meet, the rollback path is short:

| Failure | Action |
|---|---|
| FastColorSensor reads garbage | Edit XML: `FastColorSensor` → `RevColorSensorV3`, run deploy script. No code change needed. |
| SDK 11.1.0 misbehaves | `git checkout cca5746` (the commit before today's SDK upgrade) and rebuild. |
| Pedro 2.1.2 misbehaves | Same as above. |
| Want everything before today | `git checkout master` and rebuild. |

---

*Last updated 2026-05-16, end of pre-Saturday prep session.*
