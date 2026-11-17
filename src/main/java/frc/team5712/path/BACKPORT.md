# 5712 path library: WPILib 2026 backport

Vendored from [Hemlock5712/path-editor](https://github.com/Hemlock5712/path-editor) `lib/` at `4988097`
(2026-11-17). Upstream targets WPILib 2027 / Java 25. This copy runs on the roboRIO (WPILib 2026, Java 17).

## What's copied

- `core`, `follow`, `plan`: copied from `lib/src/main/java` as-is. The one Java 21 call is replaced
  (`List.getLast()` in `plan/Pathfinder.dedupe`).
- `wpilib`: ported from `lib/src/wpilib/java`, **Commands v2 only**. The v3 classes (`FollowPathCommand`,
  `PathfindCommand`) and v3 overloads are dropped.
- Left out: `tools`, `sim`, and the tests.

## Changes in `wpilib`

- `org.wpilib.*` → `edu.wpi.first.*`; `ChassisVelocities` → `ChassisSpeeds`; `Timer.getTimestamp` →
  `Timer.getFPGATimestamp`; `MatchState` → `DriverStation`.
- **Field origin.** The core and the editor put the origin at field centre (as 2027 does); 2026 puts it at
  the blue corner. `Conversions.toPose` / `toPose2d` shift by half the field, so every `Pose2d` in or out of
  this package is in the normal 2026 frame. `PathLibrary.loadAll` sets the size from `field.json`.
- **Telemetry.** 2027's `Telemetry` API is replaced by `TelemetryTable`. It publishes to NetworkTables under
  `/Telemetry/...` (centre frame, so the editor's live view works) and logs to AdvantageKit under
  `Telemetry/...` (poses in the 2026 frame). `PathTelemetry.robotPose` publishes `/Telemetry/Robot/Pose`.
- **Events run inline.** `FollowPathCommandV2` runs event commands as its own children (as v3 forks them)
  instead of scheduling them. Scheduling one would cancel an auto group that already requires the same
  subsystem. A new event command interrupts a running one that shares a requirement with it, and any still
  running end when the path ends.

## Resyncing

Copy `core`, `follow` and `plan` over, re-apply the `getLast()` fix (javac `--release 17` points it out),
and diff upstream `src/wpilib` against this `wpilib` for API changes.
