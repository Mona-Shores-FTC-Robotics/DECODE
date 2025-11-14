# Logging & Telemetry Overview

## Live Telemetry Architecture
- `TelemetryService` is the main façade for publishing telemetry data
- Subsystems provide state data through their public methods
- `TelemetryService` publishes data to:
  - FTC Dashboard packets for live visualization
  - Driver station telemetry display
  - FullPanels for team-specific metrics
- AdvantageScope Lite connects to FTC Dashboard for real-time visualization
- Pedro tuning OpModes use FTControl Panels directly

## Telemetry Levels
The system supports three telemetry levels (configurable via FTC Dashboard):
- **MATCH Mode**: Minimal telemetry for competition (<10ms overhead)
- **PRACTICE Mode**: Moderate telemetry for tuning (<20ms overhead)
- **DEBUG Mode**: Full telemetry for development

## Configuration & Toggles
- Keep runtime parameters in `@Configurable` classes beside the subsystem for live tuning via FTC Dashboard
- Global telemetry toggles (e.g. `TelemetrySettings.enableDashboardTelemetry`) stay in `TelemetrySettings`
- Switch telemetry levels via FTC Dashboard Config tab without recompiling
