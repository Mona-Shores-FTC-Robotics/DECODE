# Telemetry Stack

This package centralises every class involved in logging/telemetry so OpModes only
need to depend on `TelemetryService` and `RobotLogger`.

* `TelemetryService` – main façade used by OpModes. Fans data out to Panels
  and FTC Dashboard packets (which AdvantageScope desktop connects to).
* `TelemetryPublisher` – helper that emits the detailed drive/launcher topics
  expected by FTControl Panels.
* `TelemetrySettings` – Configurable flags that enable/disable dashboard telemetry.
* `RobotLogger` – collects subsystem Inputs via reflection and stores topics in a Map.
  Topics are retrieved by TelemetryService and published to FTC Dashboard packets.

Offline logging is handled by KoalaLog which produces `.wpilog` files compatible
with AdvantageScope for full-featured post-match analysis.

All other code should import from this package instead of `util` so telemetry
remains easy to track.
