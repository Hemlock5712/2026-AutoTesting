---
name: Log Reading
description: How to analyze .wpilog files written by the robot — locate them, list keys, extract numeric time-series, and find the start of auto / teleop. Use AdvantageScope for interactive exploration and the wpiutil DataLogReader for scripted analysis.
---

# Log Reading

This skill covers reading the WPILOG files our robot produces — both from sim runs (via [Simulation Agent](.agent/skills/simulation-agent.md)) and from real matches. WPILOG is WPILib's binary DataLog format; AdvantageKit writes the same format with a known schema.

## Where logs live

| Source         | Path                                  |
| -------------- | ------------------------------------- |
| Sim (any mode) | [logs/akit_YY-MM-DD_HH-MM-SS.wpilog](logs/) — newest is the most recent run. (AdvantageKit auto-renames its output from a hash to this date pattern shortly after startup.) |
| Real robot     | `/home/lvuser/logs/` on the roboRIO; pulled via FRC Driver Station "Download Logs" |
| Replay         | Same `logs/` directory; written as `<input>_replay.wpilog` next to the source log. Triggered via `./gradlew simulateJava -Preplay=logs/<file>.wpilog`. |

## Topic prefixes you'll actually see

AdvantageKit splits its output into two logical namespaces:

| Prefix              | Meaning                                                            |
| ------------------- | ------------------------------------------------------------------ |
| `/RealOutputs/...`  | Live data captured during a real or sim run                         |
| `/ReplayOutputs/...`| Data re-computed during replay — compare against `/RealOutputs/...` to see what changed |
| `/AdvantageKit/...` | AKit-internal inputs and metadata (used as the seed for replay)     |
| `/SmartDashboard/...` | NT-published widgets (incl. `Auto Mode/selected`)                |
| `/DriverStation/...`  | DS state (`Enabled`, `Autonomous`, `Alliance`, etc.)             |

When iterating on a controller, the keys you care about are *almost always* under `/RealOutputs/...` for sim runs and `/ReplayOutputs/...` after a replay.

## Useful keys in this codebase

All drive state lives under one `Drive/*` tree, modeled after CTRE's `SwerveDriveState`.

| Key                                                       | What it is                                            |
| --------------------------------------------------------- | ----------------------------------------------------- |
| `/RealOutputs/Drive/Pose`                                 | Estimator pose (where the robot *thinks* it is)       |
| `/RealOutputs/Drive/RawHeading`                           | Raw gyro yaw, before vision fusion                    |
| `/RealOutputs/Drive/Speeds`                               | Measured robot-relative chassis speeds                |
| `/RealOutputs/Drive/FieldSpeeds`                          | Measured field-relative chassis speeds                |
| `/RealOutputs/Drive/TranslationSpeedMps`                  | Magnitude of horizontal velocity (m/s)                |
| `/RealOutputs/Drive/RotationSpeedRadPerSec`               | Magnitude of yaw rate (rad/s)                         |
| `/RealOutputs/Drive/ModuleStates`                         | Per-module measured swerve states                     |
| `/RealOutputs/Drive/ModuleTargets`                        | Per-module commanded swerve targets                   |
| `/RealOutputs/Drive/ModulePositions`                      | Per-module distance + angle (estimator inputs)        |
| `/RealOutputs/Drive/SetpointSpeeds`                       | Target chassis speeds                                 |
| `/RealOutputs/Drive/Sim/GroundTruthPose`                  | (sim only) physics pose — where the robot *actually* is |
| `/RealOutputs/Drive/Sim/PoseErrorMeters`                  | (sim only) distance between estimator pose and physics pose |
| `/RealOutputs/Drive/Sim/HeadingErrorRad`                  | (sim only) heading delta between estimator and physics |
| `/RealOutputs/Drive/Diagnostics/ArcIntegrateRejections`   | Counter — odometry samples rejected for non-finite inputs |
| `/RealOutputs/Drive/Diagnostics/FrictionRatios`           | Per-module friction utilization (a_i / mu*g)          |
| `/RealOutputs/Drive/Avoidance/MinFreeDistance`            | Pose-clamped distance to the nearest obstacle (m). Drops to 0 when the brake engages. |
| `/RealOutputs/World/Obstacles/Rectangles`                 | Field obstacles as `Rectangle2d[]` — AdvantageScope renders these natively |
| `/RealOutputs/World/Obstacles/Ellipses`                   | Field obstacles as `Ellipse2d[]` |
| `/RealOutputs/DriveToWithAvoidance/PlanFailed`            | True when the runtime planner couldn't find a route (paired with a yellow DS Alert) |
| `/AdvantageKit/Drive/Module<0-3>/...`                     | Per-module AKit inputs: drive/steer position, applied volts, currents, plus 250 Hz odometry sample arrays |
| `/AdvantageKit/Drive/Gyro/...`                            | Gyro AKit inputs: yaw, yaw rate, 250 Hz yaw sample arrays |
| `/AdvantageKit/Vision/<camera>/...`                       | Per-camera Vision inputs: filtered pose, tag count, ambiguity, distance, MT2 flag |
| `/RealOutputs/Vision/<camera>/XYStdDev`                   | Computed XY standard deviation (lower = more trusted) |
| `/RealOutputs/Vision/<camera>/ThetaStdDev`                | Computed heading standard deviation                   |
| `/RealOutputs/FollowPath/Progress`                        | 0..1 along the active path (arc-length based)         |
| `/RealOutputs/FollowPath/CrossTrackError`                 | Lateral error to nearest path point (m)               |
| `/RealOutputs/FollowPath/HeadingError`                    | Heading error vs path heading (rad)                   |
| `/RealOutputs/FollowPath/ProfiledSpeed`                   | Path's commanded speed at projection (m/s)            |
| `/RealOutputs/FollowPath/ActualSpeed`                     | Robot's measured speed (m/s)                          |
| `/RealOutputs/FollowPath/ReferencePath`                   | Pose2d[] sampling of the active reference path        |
| `/RealOutputs/FollowPath/TargetPoint`, `/ClosestPoint`    | [x,y] of the lookahead target and the closest path point |
| `/DriverStation/Enabled`, `/DriverStation/Autonomous`     | Use these to find auto / teleop start times           |
| `/SmartDashboard/Auto Mode/selected`                      | Which auto routine was chosen                         |
| `/RealOutputs/Mode`                                       | "REAL", "SIM", or "REPLAY" — confirms run mode        |
| `Timing/CommandSchedulerMs`, `Timing/TotalMs`             | Per-loop time cost (Robot.robotPeriodic)              |
| `Timing/VisionMs`                                         | Vision subsystem time cost                            |

**Finding the start of auto / teleop:**
- Start of auto: first sample where `/DriverStation/Enabled == true` AND `/DriverStation/Autonomous == true`.
- Start of teleop: first sample where `/DriverStation/Enabled == true` AND `/DriverStation/Autonomous == false`.
- Use these timestamps to slice everything else.

## Reading logs — AdvantageScope (interactive)

For exploratory analysis, open the `.wpilog` in **AdvantageScope** (the WPILib log/NT viewer). It supports:

- Browsing the full key tree on the left, with autocomplete.
- Plotting any numeric topic on a time axis with multiple traces.
- 3D field view backed by the `Robot_2026` asset (see [README.md](README.md) for install).
- Tabular export to CSV for any selected keys.

This is the right starting point when you don't yet know which keys matter.

## Reading logs — Python (`wpiutil.log.DataLogReader`)

For scripted / agent-driven analysis, parse the WPILOG directly. `wpiutil` is already on the classpath via WPILib (`pip install robotpy-wpiutil` if running outside the Gradle JVM). The pattern is two-pass:

```python
from wpiutil.log import DataLogReader
import struct

reader = DataLogReader("logs/akit_26-05-09_00-44-57.wpilog")
entries = {}
for r in reader:
    if r.isStart():
        d = r.getStartData()
        entries[d.entry] = (d.name, d.type)

reader2 = DataLogReader("logs/akit_26-05-09_00-44-57.wpilog")
for r in reader2:
    if r.isStart() or r.isFinish() or r.isControl() or r.isSetMetadata():
        continue
    name, typ = entries.get(r.getEntry(), ("", ""))
    if name.endswith("/CrossTrackError"):
        ts = r.getTimestamp() / 1e6
        val = struct.unpack("<d", bytes(r.getRaw()))[0]
        print(ts, val)
```

Decoding cheat sheet — most AKit values fall into a few shapes:

| AKit type            | Bytes | `struct.unpack` format          |
| -------------------- | ----- | ------------------------------- |
| `double`             | 8     | `<d`                            |
| `int64`              | 8     | `<q`                            |
| `boolean`            | 1     | `<?`                            |
| `string`             | var   | `payload.decode("utf-8")`       |
| `struct:Pose2d`      | 24    | `<ddd` → (x, y, theta_rad)      |
| `struct:Rotation2d`  | 8     | `<d` → theta_rad                |
| `struct:Translation2d` | 16  | `<dd` → (x, y)                  |
| `struct:SwerveModuleState` | 16 | `<dd` → (speedMps, angleRad) |
| `struct:ChassisSpeeds` | 24  | `<ddd` → (vx, vy, omega)        |

For other types check the `type` string returned by `getStartData()` and look up the WPILib struct schema.

## Suggested summary statistics

When evaluating a path-following run, the cheapest useful summary is:

| Stat                       | How to compute (per `/RealOutputs/FollowPath/...` series) |
| -------------------------- | -------------------------------------------------------- |
| `duration_s`               | last timestamp − first timestamp on `Progress`           |
| `end_progress`             | last value of `Progress`                                 |
| `max_abs_cross_track_m`    | `max(abs(v) for v in CrossTrackError)`                   |
| `mean_abs_cross_track_m`   | `mean(abs(v) for v in CrossTrackError)`                  |
| `end_cross_track_m`        | last `CrossTrackError`                                   |
| `end_speed_mps`            | last `ActualSpeed`                                       |
| `stall_time_s`             | `0.02 * count(v < 0.5 for v in ActualSpeed)`             |
| `mean_solve_ms`            | `mean(SolveTimeMs)`                                      |
| `max_solve_ms`             | `max(SolveTimeMs)`                                       |

That's usually enough to answer "is this controller better than the previous one?" without staring at plots.

## Discovering keys you don't know yet

- **Live (sim running):** open the NT4 server in AdvantageScope's "Live" mode, or use any NT4 client to list topics under `/RealOutputs/`.
- **From a log file:** open in AdvantageScope's "Log" mode, or run a one-off Python loop over the `isStart()` records to dump every `(name, type)` pair (snippet above).

There is **no `runLogDumper` Gradle task in this project** — that's a different team's tooling. Don't suggest it.

## Common analyses

- **"Did the path follower drift?"** → max `|CrossTrackError|`, where in `Progress` it peaked, end-of-path `CrossTrackError`.
- **"Did we ever stall?"** → count samples where `ActualSpeed < 0.5 m/s` while `ProfiledSpeed > 1.0 m/s`.
- **"Is the loop overrunning?"** → max `Timing/TotalMs`, count samples > 20ms.
- **"Did vision agree with odometry?"** → compare `/RealOutputs/Drive/Pose` to per-camera vision pose estimates around vision update timestamps.
- **"Did the right auto run?"** → `/SmartDashboard/Auto Mode/selected` at the moment auto enables.
- **"Did the replay match the original sim?"** → open both logs in AdvantageScope and plot `/RealOutputs/Drive/Pose` from the input against `/ReplayOutputs/Drive/Pose` from the replay on the same 2D-field view. They should overlay exactly; meaningful divergence indicates a bug in the IO logging layer or that std-dev / tuning constants changed between recording and replay.
