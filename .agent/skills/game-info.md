---
name: Game Info
description: Reference info for the 2026 FRC game — field layout, scoring zones, alliance conventions, and how those map onto the robot's autonomous routines and field constants. Read this whenever a task involves field positions, alliance flipping, or game-piece scoring.
---

# Game Info

This skill captures the season-specific knowledge an agent needs to reason about robot behavior in match context — *what* is on the field, *where* the robot is allowed to be, and *how* alliance affects everything. Anything purely mechanical / electrical / software is in [Robot Description](.agent/skills/robot-description.md) instead.

## Season

- **2026 FRC season.** Project root is named `2026-AutoTesting`. WPILib year is **2026**, vendordeps are 2026 builds:
  - GradleRIO `2026.2.1`
  - Phoenix6 `26.1.3`
  - ChoreoLib `2026`
  - AdvantageKit (current)
- Robot model assets are tracked under the AdvantageScope name `Robot_2026` (see [README.md](README.md) for install).

## Field & alliance conventions used in this codebase

The codebase enforces these as project conventions — agents should respect them rather than re-deriving:

- **Field origin is blue alliance.** Choreo paths are authored in the blue-alliance frame.
- **Red alliance = rotated** (180° rotational symmetry, *not* mirrored). The 2026 field uses `SymmetryType.ROTATE` — a path point at blue (x, y, θ) maps to red (`fieldLength - x`, `fieldWidth - y`, `θ + 180°`). Choreo's `trajectory.flipped()` does this; don't hand-compute. (Distinct from earlier-season fields that used MIRROR symmetry — copy/pasting old flip code from a 2024/2025 codebase will produce wrong red poses.)
- **Where the flip lives:** [AutoPath.get()](src/main/java/frc/robot/utils/path/AutoPath.java) returns `red` when `FieldInfo.shouldFlip()` is true, where `red = ArcLengthTrajectory.fromChoreo(trajectory.flipped())`. Both alliances are pre-computed at load time — flipping at runtime is a const-time pointer swap, not a recompute.
- **Driver perspective rotation:** `kBlueAlliancePerspectiveRotation = 0°`, `kRedAlliancePerspectiveRotation = 180°` (see [CommandSwerveDrivetrain.java](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java)). The swerve applies this once per alliance change so "forward on the joystick" always means "downfield from the driver's POV."
- **Alliance is cached** — `FieldInfo.resetAllianceCache()` is wired into `autonomousInit()` and `teleopInit()` in [Robot.java](src/main/java/frc/robot/Robot.java) so DS reconnects mid-match don't silently flip behavior. Don't read `DriverStation.getAlliance()` directly anywhere on the hot path; go through `FieldInfo`.

## Field zones / scoring locations

> **Status: TODO.** As autos are added, fill in the named field positions an agent should know about — substations, scoring locations, defensive zones — keyed to the game-specific terminology and to the `Pose2d` constants in code (or the named waypoints in `src/main/deploy/choreo/Choreo.chor`).
>
> Recommended structure once known:
>
> | Name             | Blue pose (x, y, θ)        | Red pose                 | Defined in                                           |
> | ---------------- | -------------------------- | ------------------------ | ---------------------------------------------------- |
> | `STATION_LEFT`   | `(1.20, 7.05, 0°)`         | call `.get()` on ExtPose | [utils/FieldInfo.java](src/main/java/frc/robot/utils/FieldInfo.java) |
> | `SCORING_REEF_A` | …                          | …                 | …                                                    |
>
> Until the season's game-piece / scoring constants are pinned in code, refer to **the official 2026 FRC game manual** for canonical zone names, dimensions, and rules. Don't make up coordinates from memory — pull them from the manual or the field CAD.

## Game-piece handling

> **Status: TODO.** When manipulator subsystems land (intake, scoring mechanism, etc.), document here:
>
> - What pieces the robot can hold / score, and the named states (e.g. `EMPTY`, `STAGED`, `SCORING`).
> - Which sensor or NT key indicates each state.
> - Auto routines' assumptions about starting piece configuration.
>
> Today the only subsystems are the drivetrain and Limelights — no game-piece handling exists yet, so any reasoning about piece flow is necessarily speculative.

## Authoritative references

When this skill is silent or stale, defer to (in order):

1. **The 2026 game manual** — rules, field dimensions, scoring values. Always treat as source of truth over anything written here.
2. **`src/main/deploy/choreo/Choreo.chor`** — opened in the Choreo desktop app, this contains the field image, named waypoints, and obstacle constraints used to author paths.
3. **WPILib `AprilTagFieldLayout`** for the 2026 season — tag IDs / poses on the field. Loaded by `LimelightHelpers` for vision pose estimation.
4. **6328's published season guide** — the codebase already borrows their vision std-dev formula and JVM tuning recipe; their field-zone analysis is usually the cleanest secondary reference.

## Anti-patterns

- Don't hardcode red-alliance poses. Author in blue, flip via `FieldInfo` / `AutoPath`.
- Don't assume the 2025 field layout. Tag IDs, reef/processor/whatever-the-2026-equivalent layouts changed.
- Don't read `DriverStation.getAlliance()` in subsystem `periodic()` — use `FieldInfo`'s cached value, refreshed at mode-change boundaries.
- Don't introduce a "match clock" path-trigger. Triggers are arc-length-based ([PathAction](src/main/java/frc/robot/autonomous/AutoCommands.java)), not time-based — see [Robot Description](.agent/skills/robot-description.md) for the why.
