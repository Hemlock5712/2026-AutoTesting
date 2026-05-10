---
name: Robot Description
description: High-level explanation of the robot's subsystems and behavior — what's on the robot, how the major control flows fit together, and where in the source tree to look for each piece. Read this before reasoning about controller / autonomous changes.
---

# Robot Description

This is a Java FRC robot project running on the **WPILib 2026 / GradleRIO 2026.2.1** stack. Code entry point is [Main.java](src/main/java/frc/robot/Main.java) → [Robot.java](src/main/java/frc/robot/Robot.java) → [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java). Everything is `LoggedRobot` (AdvantageKit), so all subsystem state is published to NetworkTables and recorded to a WPILOG.

## Subsystems

### Drive — [src/main/java/frc/robot/subsystems/drive/](src/main/java/frc/robot/subsystems/drive/)

Direct port of the [AdvantageKit talonfx_swerve template](https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/template_projects/sources/talonfx_swerve/src/main/java/frc/robot/subsystems/drive). One [Drive](src/main/java/frc/robot/subsystems/drive/Drive.java) subsystem with a Java `SwerveDrivePoseEstimator` as the source of truth in *all* modes — sim ↔ replay is bit-identical (verified at 0.0 m delta over a 53-second run; 100× looser std-dev produces a 15 m pose shift, confirming tuning works).

The Drive constructor takes one `GyroIO` and four `ModuleIO`s. [RobotContainer](src/main/java/frc/robot/RobotContainer.java) selects implementations from [Constants.getMode()](src/main/java/frc/robot/Constants.java):

| Mode | GyroIO | ModuleIO |
| --- | --- | --- |
| REAL | [GyroIOPigeon2](src/main/java/frc/robot/subsystems/drive/GyroIOPigeon2.java) | [ModuleIOTalonFX](src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java) — Phoenix6 TalonFX×2 + CANcoder per module |
| SIM | [GyroIOSim](src/main/java/frc/robot/subsystems/drive/GyroIOSim.java) (wraps maple-sim `GyroSimulation`) | [ModuleIOSim](src/main/java/frc/robot/subsystems/drive/ModuleIOSim.java) — maple-sim `SwerveModuleSimulation` (rigid-body physics via dyn4j) |
| REPLAY | empty | empty (AKit fills `@AutoLog` inputs from log) |

**250 Hz odometry on real hardware.** [PhoenixOdometryThread](src/main/java/frc/robot/subsystems/drive/PhoenixOdometryThread.java) is a singleton thread that does `BaseStatusSignal.waitForAll(...)` (CAN-FD) or sleeps + `refreshAll(...)` (CAN 2.0) at `Drive.ODOMETRY_FREQUENCY` (250 Hz on FD, 100 Hz else). It samples each registered position signal into a per-signal queue. `Drive.periodic()` acquires `Drive.odometryLock`, the IOs drain their queues into `@AutoLog` arrays, and the pose estimator is updated once per logged sample (so all 250 sub-cycles are walked through). Vision adds measurements via `drive.addVisionMeasurement(pose, fpgaTs, stdDevs)`.

**SIM uses [maple-sim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim)** (vendored under [simlib/](src/main/java/frc/robot/simlib/)). The chassis spawn pose lives in [RobotContainer.SIM_SPAWN_POSE](src/main/java/frc/robot/RobotContainer.java) (default `(8.0, 4.0)`); the estimator is reset to match at construction, and `Drive.onPoseReset` wires future estimator resets through to `SwerveDriveSimulation.setSimulationWorldPose` so auto-routine resets don't strand the sim chassis. `Robot.simulationPeriodic` ticks the arena via `RobotContainer.updateSimulation`, which calls `Drive.updateSimulationGroundTruth(truth)` — that logs `Drive/Sim/GroundTruthPose`, `Drive/Sim/PoseErrorMeters`, and `Drive/Sim/HeadingErrorRad` (the answer to "what does the estimator think vs. what's actually happening"). [VisionIOSim](src/main/java/frc/robot/subsystems/vision/VisionIOSim.java) feeds the simulated truth pose back into the estimator. Maple-sim uses 5 sub-ticks per 20 ms cycle (≈250 Hz effective); each `ModuleIO.updateInputs` writes one odometry sample per sub-tick.

**Drive odometry refinements (vs. the upstream AKit template):**
- **Arc-integrated module deltas.** [Drive.arcIntegrate](src/main/java/frc/robot/subsystems/drive/Drive.java) replaces straight-line per-sample integration with arc integration assuming constant module ω during the sample, then re-encodes the arc displacement as an effective `(distance, angle)` so WPILib's straight-chord kinematics produces the arc-correct twist.
- **Azimuth coupling compensation.** [ModuleIOTalonFX](src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java) subtracts the phantom drive motion induced by steer rotation (`steer_mech_rad * CouplingGearRatio / DriveMotorGearRatio`) from raw drive position and velocity. CTRE's `SwerveDrivetrain` does this internally; we don't use that class so we do it ourselves. SIM is untouched (maple-sim has independent shafts, no coupling to subtract).
- **Field-escape diagnostic.** `Drive/Diagnostics/FieldEscapeHits` counts periodic ticks where the estimator pose has crossed any field wall. Diagnostic only — we tried clamping the cached pose to the field but it bit legitimate near-wall path overshoots and was reverted.
- **Arc-integration rejection counter.** `Drive/Diagnostics/ArcIntegrateRejections` counts odometry samples thrown out because an input was non-finite (e.g. MapleSim brownout poisoning a steer angle with NaN).
- **Per-module friction utilization.** `Drive/Diagnostics/FrictionRatios` exposes each wheel's `a_i / (mu*g)` for tuning the friction limiter.

Drive exposes the canonical API: `runVelocity(ChassisSpeeds)`, `resetPose(Pose2d)`, `getPose`, `getRotation`, `getRobotSpeeds`, `getFieldSpeeds`, `addVisionMeasurement`, `samplePoseAt`, `stopWithX`, `pointWheelsAt`, plus the high-rate hook `setControl(SwerveRequest) / clearControl()`. CTRE's `SwerveDrivetrain` and `CommandSwerveDrivetrain` are not used.

**250 Hz fast loop for commands.** Drive owns a `Notifier` that fires at 4 ms (`Drive.HIGH_RATE_PERIOD_S`) and invokes the active `SwerveRequest` (see [SwerveRequest.java](src/main/java/frc/robot/subsystems/drive/SwerveRequest.java) and the implementations under [requests/](src/main/java/frc/robot/subsystems/drive/requests/)). Use it via `drive.setControl(request)` in `command.initialize()` and `drive.clearControl()` in `command.end()`. Commands publish their target velocity into the request's volatile fields on the 50 Hz main loop; the fast loop reads them and pushes to `runVelocity`. **Threading rules for `SwerveRequest.apply(...)`:**
- Reads of `getPose() / getRotation() / getRobotSpeeds() / getFieldSpeeds()` are tear-free (volatile snapshots published from `periodic`).
- `runVelocity(...)` is safe to call (Phoenix6 motor controls and ModuleIOSim setpoint writes are atomic enough; logging is deferred to `periodic`).
- **Do not call `Logger.recordOutput` from `apply()`** — AKit's `StructBuffer`s aren't thread-safe, and concurrent writes corrupt the byte buffer. Stash diagnostics in volatile fields and let the subsystem log them from `periodic()`.
- `AccelerationLimiter`'s scratch buffers are `ThreadLocal`, so the limiter is safe to call from any thread.

Module geometry / gearing / IDs / gains live in [generated/TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java), generated by Tuner X. Only the constant declarations are kept — the `createDrivetrain()` factory and `TunerSwerveDrivetrain` inner class were removed.

### Vision — [src/main/java/frc/robot/subsystems/vision/](src/main/java/frc/robot/subsystems/vision/)

Multi-camera AprilTag pose fusion behind an AKit IO layer.

- [VisionIO](src/main/java/frc/robot/subsystems/vision/VisionIO.java) — interface; one impl per camera type.
- [VisionIOLimelight](src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java) — real Limelight impl. Pulls `botpose_*` from NT, applies rejection rules (ambiguity, distance, field bounds, angular-velocity gating), and emits a single best pose per cycle into [VisionInputsAutoLogged](src/main/java/frc/robot/subsystems/vision/VisionInputs.java). Filtering happens here so the inputs replay deterministically.
- [Vision](src/main/java/frc/robot/subsystems/vision/Vision.java) — subsystem that pushes orientation back to the cameras (for MegaTag2), computes std-devs from logged tag distance / tag count, and either fuses time-synced poses across cameras or pushes them through individually. The std-dev coefficients live at the top of this file — they are the **vision tuning surface** for replay-based iteration.
- Five Limelights: `limelight-br/bl/fl/fr/mm`. Std-dev formula: `coefficient * avgTagDist^1.2 / tagCount^2`, capped at `MAX_EFFECTIVE_TAG_COUNT = 2.5` because tags on the same wall correlate.
- Rejection: `MAX_AMBIGUITY = 0.3`, field-border margin 0.5 m, angular-velocity gating (`MT1: 360 deg/s`, `MT2: 200 deg/s`).
- Tunings borrow from 6328 Mechanical Advantage's published trust ratios, scaled to WPILib defaults.

## Path following — distance-based, not time-based

This is the load-bearing detail of the codebase. The robot **does not chase a time-parameterized trajectory**; it tracks its actual position along an arc-length-parameterized path. If the robot stalls, the path waits for it. If it gets nudged, it reprojects to the closest point and keeps going.

### Pipeline

1. **Choreo `.traj` file** ([src/main/deploy/choreo/](src/main/deploy/choreo/)) authored in the Choreo desktop app.
2. **`AutoPath` enum** ([src/main/java/frc/robot/utils/path/AutoPath.java](src/main/java/frc/robot/utils/path/AutoPath.java)) loads it via `Choreo.loadTrajectory` and re-parameterizes time → arc-length.
3. **`ArcLengthTrajectory`** ([src/main/java/frc/robot/utils/path/ArcLengthTrajectory.java](src/main/java/frc/robot/utils/path/ArcLengthTrajectory.java)) and the `FollowablePath` interface expose:
   - `getPoint(s)` / `getHeading(s)` / `getCurvature(s)` / `getProfiledSpeed(s)`
   - `getClosestPointInRange(translation, sMin, sMax)` for projection
4. **`FollowPath` command** ([src/main/java/frc/robot/commands/FollowPath.java](src/main/java/frc/robot/commands/FollowPath.java)) is a PD cross-track controller:
   - Adaptive lookahead `= clamp(k * speed + min, [min, max])`, capped by max arc-angle.
   - Cross-track PD (`Kp = 3.0`, `Kd = 0.5`) plus curvature feedforward (`gain = 0.1 s`).
   - Output runs through `AccelerationLimiter.integrateVelocity` to enforce friction circle, motor torque, and jerk limits.
   - Completion: within `0.05 m` of path end *and* speed below `0.1 m/s`.
5. **`AutoCommands.followPathWithActions`** ([src/main/java/frc/robot/autonomous/AutoCommands.java](src/main/java/frc/robot/autonomous/AutoCommands.java)) wraps `FollowPath` with **`PathAction`s** triggered at specific arc-length positions. Choreo `EventMarker` timestamps are converted to arc-length via `PathAction.fromMarker`.

Alliance flipping is handled inside `AutoPath.get()` — pre-computed at load time, picked at runtime via `FieldInfo.shouldFlip()`. For non-path field locations, use the `Ext*` containers in [utils/geometry/](src/main/java/frc/robot/utils/geometry/) (`ExtPose`, `ExtTranslation`, `ExtRotation`) — declare a blue-origin value and call `.get()` at runtime to get the alliance-flipped variant. The flip math is private to that package; there is no longer a `FieldFlip` utility class.

> **Critical convention:** never introduce a time-based path follower. The whole control architecture assumes `s` (arc length), not `t`.

### Other movement commands

- **`TeleopDrive`** — default teleop command. Translation from left stick (rescaled with deadband + squared magnitude), rotation from right stick.
- **`DriveToPoint`** — closed-loop drive to a target `Pose2d`. Used for non-path-based moves (alignment, station approach).

## Logging

[Robot.java](src/main/java/frc/robot/Robot.java) wires AdvantageKit, switched on [Constants.getMode()](src/main/java/frc/robot/Constants.java):
- **REAL:** `WPILOGWriter` to `/home/lvuser/logs` + `NT4Publisher`.
- **SIM:** `WPILOGWriter` to `logs/` + `NT4Publisher`.
- **REPLAY:** `setUseTiming(false)`, `Logger.setReplaySource(WPILOGReader(System.getProperty("frc.replay.input")))`, output to a sibling `_replay.wpilog`. No NT publisher, no sim extensions (they're disabled in [build.gradle](build.gradle) when `-Preplay=...` is set).

Other knobs:
- `loopOverrunWarning = 0.2 s`, period overridden to that value (intentional — warnings only fire on truly bad cycles).
- Watchdog timeout reached via reflection (`IterativeRobotBase.m_watchdog`).
- Per-cycle timing logged to `Timing/CommandSchedulerMs`, `Timing/TotalMs`, `Timing/VisionMs`.

To activate REPLAY: `./gradlew simulateJava -Preplay=logs/<file>.wpilog -Pheadless`. See [Replay Testing](.agent/skills/replay-testing.md) for the full workflow.

## Tunables

[TunableTable](src/main/java/frc/robot/utils/TunableTable.java) + [Tunables](src/main/java/frc/robot/utils/Tunables.java) provide live-editable doubles published under `/Tunables/...`. `Tunables.update()` runs every `robotPeriodic`. To iterate on gains without rebuilding, write to those NT keys from any NT4 client (Glass, AdvantageScope's NT publisher, Shuffleboard, or a small `pyntcore` script).

## Autonomous selection

Single chooser entry today: `"NewPath (PD)"` → `newPathAutoPD()` in [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java). The routine:
1. Reset pose to the path's start pose (alliance-flipped).
2. Follow `AutoPath.NEW_PATH` with Choreo event markers wired through `actionsFromChoreoEvents`.

When adding new autos: add an `AutoPath` enum entry, a `Choreo.chor` path, and a `chooser.addOption` call in `RobotContainer`.

## Hardware target

- **roboRIO 1 / 2** — JVM args in [build.gradle](build.gradle) pin a 100 MB heap with `+AlwaysPreTouch`, `UseSerialGC`, `GCTimeRatio=5`, `MaxGCPauseMillis=50`. Following the 254 / 6328 recipe.
- **CAN FD** for swerve modules + Pigeon2.
- **Brownout floor** 6.0 V (`MIN_OCV` in `Robot.java`).

## What lives where (cheat sheet)

| Topic                          | File / package                                                         |
| ------------------------------ | ---------------------------------------------------------------------- |
| Match-time wiring + mode switch | [Robot.java](src/main/java/frc/robot/Robot.java)                      |
| Subsystem + binding wiring     | [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java)     |
| Mode enum (REAL/SIM/REPLAY)    | [Constants.java](src/main/java/frc/robot/Constants.java)               |
| Drive subsystem                | [subsystems/drive/Drive.java](src/main/java/frc/robot/subsystems/drive/Drive.java) |
| Module / Gyro IOs              | [subsystems/drive/ModuleIO.java](src/main/java/frc/robot/subsystems/drive/ModuleIO.java) (+ TalonFX, Sim impls), [GyroIO.java](src/main/java/frc/robot/subsystems/drive/GyroIO.java) (+ Pigeon2, Sim impls) |
| Maple-sim physics + spawn      | [simlib/](src/main/java/frc/robot/simlib/) (vendored), `Drive.getMapleSimConfig()`, `RobotContainer.SIM_SPAWN_POSE` |
| Sim vs estimator comparison    | `Drive/Sim/{GroundTruthPose, PoseErrorMeters, HeadingErrorRad}` from `Drive.updateSimulationGroundTruth` |
| Drive diagnostics              | `Drive/Diagnostics/{FieldEscapeHits, ArcIntegrateRejections, FrictionRatios}` |
| 250 Hz odometry collector      | [subsystems/drive/PhoenixOdometryThread.java](src/main/java/frc/robot/subsystems/drive/PhoenixOdometryThread.java) |
| Phoenix retry helper           | [utils/PhoenixUtil.java](src/main/java/frc/robot/utils/PhoenixUtil.java) |
| Swerve hardware constants      | [generated/TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java) |
| Path math                      | [utils/path/](src/main/java/frc/robot/utils/path/)                     |
| Path-following controller      | [commands/FollowPath.java](src/main/java/frc/robot/commands/FollowPath.java) |
| Auto building blocks           | [autonomous/AutoCommands.java](src/main/java/frc/robot/autonomous/AutoCommands.java) |
| Vision IO + fusion             | [subsystems/vision/](src/main/java/frc/robot/subsystems/vision/)       |
| Geometry helpers               | [utils/geometry/](src/main/java/frc/robot/utils/geometry/)             |
| Sim startup hook (headless)    | [utils/SimStartup.java](src/main/java/frc/robot/utils/SimStartup.java) |
| Pose comparison tool           | [scripts/compare_poses.py](scripts/compare_poses.py)                 |
