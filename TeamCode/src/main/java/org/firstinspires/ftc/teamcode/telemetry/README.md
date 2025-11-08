# Telemetry Stack

This package centralises every class involved in logging/telemetry so OpModes only
need to depend on `TelemetryService` and `RobotLogger`.

* `TelemetryService` – main façade used by OpModes. Fans data out to Panels,
  the dashboard/AdvantageScope packet, and optional PsiKit logs.
* `TelemetryPublisher` – helper that emits the detailed drive/shooter topics
  expected by FTControl Panels (and PsiKit).
* `TelemetrySettings` – Configurable flags that enable/disable dashboard or
  PsiKit logging from panels.
* `RobotLogger` – AdvantageScope Lite/AdvLogger wrapper; background worker that
  samples subsystem inputs and emits metrics/events.
* `AdvLogger` – reflective bridge into AdvantageScope Lite (falls back to
  console logging when the dependency is absent).
* `PsiKitAdapter` – lightweight CSV logger that mimics PsiKit if the real
  dependency is unavailable yet.

All other code should import from this package instead of `util` so telemetry
remains easy to track.
