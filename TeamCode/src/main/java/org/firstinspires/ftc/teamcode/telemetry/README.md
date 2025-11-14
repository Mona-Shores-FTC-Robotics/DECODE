# Telemetry Stack

This package centralizes all telemetry-related classes for the robot.

* `TelemetryService` – main façade used by OpModes. Publishes data to FTC Dashboard,
  driver station, and FullPanels.
* `TelemetryPublisher` – helper that emits detailed drive/launcher topics
  expected by FTControl Panels.
* `TelemetrySettings` – Configurable flags and telemetry level settings (MATCH/PRACTICE/DEBUG).
* `RobotStatusLogger` – logs robot status information including OpMode state,
  battery voltage, and error messages.

The system supports three telemetry levels:
* **MATCH Mode**: Minimal telemetry for competition (<10ms overhead)
* **PRACTICE Mode**: Moderate telemetry for tuning (<20ms overhead)
* **DEBUG Mode**: Full telemetry for development

All other code should import from this package to keep telemetry organized.
