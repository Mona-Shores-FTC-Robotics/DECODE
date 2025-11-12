# 📐 Coding Conventions - DECODE FTC 2025

## Robot Structure
- `Robot.java` initializes all subsystems and passes shared references.
- Subsystems (e.g., DriveSubsystem, VisionSubsystem) do not access each other directly unless injected.
- We use a `RobotContainer` pattern to centralize all field references and subsystem coordination.

## Drive Control
- All movement is handled through `DriveSubsystem`.
- Normal teleop drive uses `setTeleopDrive(forward, strafeLeft, turnCW, isRobotCentric)`.
- Aiming overrides `turn` input by using `aimAndDrive()`.
- Field-centric mode is default (`isRobotCentric = false`).

## Vision
- `VisionSubsystem.getAimAngle()` returns `Optional<Double>`.
- `VisionSubsystem.getRobotPoseFromTag()` returns `Pose`.
- `VisionSubsystem.shouldUpdateOdometry()` returns true once per tag lock.

## Odometry
- `DriveSubsystem` owns the `Follower` instance and updates its odometry in `periodic()`.
- When a valid tag is seen, odometry may be reset via `follower.setPose(...)`.