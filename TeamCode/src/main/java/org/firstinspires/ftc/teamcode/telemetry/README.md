# Telemetry Stack

This package centralises every class involved in logging/telemetry so OpModes only
need to depend on `TelemetryService` and `RobotLogger`.

* `TelemetryService` – main façade used by OpModes. Fans data out to Panels
  and the dashboard/AdvantageScope packet.
* `TelemetryPublisher` – helper that emits the detailed drive/launcher topics
  expected by FTControl Panels.
* `TelemetrySettings` – Configurable flags that enable/disable dashboard telemetry.
* `RobotLogger` – AdvantageScope Lite/AdvLogger wrapper; background worker that
  samples subsystem inputs and emits metrics/events.
* `AdvLogger` – reflective bridge into AdvantageScope Lite (falls back to
  console logging when the dependency is absent).

Offline logging is handled by KoalaLog which produces `.wpilog` files compatible
with AdvantageScope for full-featured post-match analysis.

All other code should import from this package instead of `util` so telemetry
remains easy to track.
