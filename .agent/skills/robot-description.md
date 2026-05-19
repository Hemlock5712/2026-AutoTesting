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

**SIM uses [maple-sim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim)** (vendored under [simlib/](src/main/java/frc/robot/simlib/)). The chassis spawn pose lives in [RobotContainer.SIM_SPAWN_POSE](src/main/java/frc/robot/RobotContainer.java) (default `(8.27, 4.0)`); the estimator is reset to match at construction (unconditionally — runs in REAL/SIM/REPLAY, so replay reproduces absolute poses, not just trajectory shape), and `Drive.onPoseReset` wires future estimator resets through to `SwerveDriveSimulation.setSimulationWorldPose` so auto-routine resets don't strand the sim chassis. `Robot.simulationPeriodic` ticks the arena via `RobotContainer.updateSimulation`, which calls `Drive.updateSimulationGroundTruth(truth)` — that logs `Drive/Sim/GroundTruthPose`, `Drive/Sim/PoseErrorMeters`, and `Drive/Sim/HeadingErrorRad` (the answer to "what does the estimator think vs. what's actually happening"). [VisionIOPhotonVisionSim](src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVisionSim.java) feeds simulated AprilTag observations into the estimator via PhotonVision's `VisionSystemSim`. Maple-sim uses 5 sub-ticks per 20 ms cycle (≈250 Hz effective); each `ModuleIO.updateInputs` writes one odometry sample per sub-tick.

**Drive odometry refinements (vs. the upstream AKit template):**
- **Arc-integrated module deltas.** [Drive.arcIntegrate](src/main/java/frc/robot/subsystems/drive/Drive.java) replaces straight-line per-sample integration with arc integration assuming constant module ω during the sample, then re-encodes the arc displacement as an effective `(distance, angle)` so WPILib's straight-chord kinematics produces the arc-correct twist.
- **Azimuth coupling compensation.** [ModuleIOTalonFX](src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java) subtracts the phantom drive motion induced by steer rotation (`steer_mech_rad * CouplingGearRatio / DriveMotorGearRatio`) from raw drive position and velocity. CTRE's `SwerveDrivetrain` does this internally; we don't use that class so we do it ourselves. SIM is untouched (maple-sim has independent shafts, no coupling to subtract).
- **Arc-integration rejection counter.** `Drive/Diagnostics/ArcIntegrateRejections` counts odometry samples thrown out because an input was non-finite (e.g. MapleSim brownout poisoning a steer angle with NaN).

Drive exposes the canonical API: `runVelocity(ChassisSpeeds)`, `resetPose(Pose2d)`, `getPose`, `getRotation`, `getRobotSpeeds`, `getFieldSpeeds`, `addVisionMeasurement`, `samplePoseAt`, `stopWithX`, `pointWheelsAt`, plus the high-rate hook `setControl(SwerveRequest) / clearControl()`. CTRE's `SwerveDrivetrain` and `CommandSwerveDrivetrain` are not used.

**250 Hz fast loop for commands.** Drive owns a `Notifier` that fires at 4 ms (`Drive.HIGH_RATE_PERIOD_S`) and invokes the active `SwerveRequest` (see [SwerveRequest.java](src/main/java/frc/robot/subsystems/drive/SwerveRequest.java) and the implementations under [requests/](src/main/java/frc/robot/subsystems/drive/requests/)). Use it via `drive.setControl(request)` in `command.initialize()` and `drive.clearControl()` in `command.end()`. Commands publish their target velocity into the request's volatile fields on the 50 Hz main loop; the fast loop reads them and pushes to `runVelocity`. **Threading rules for `SwerveRequest.apply(...)`:**
- Reads of `getPose() / getRotation() / getRobotSpeeds() / getFieldSpeeds()` are tear-free (volatile snapshots published from `periodic`).
- `runVelocity(...)` is safe to call (Phoenix6 motor controls and ModuleIOSim setpoint writes are atomic enough; logging is deferred to `periodic`).
- **Do not call `Logger.recordOutput` from `apply()`** — AKit's `StructBuffer`s aren't thread-safe, and concurrent writes corrupt the byte buffer. Stash diagnostics in volatile fields and let the subsystem log them from `periodic()`.
- Per-module slip/torque/steer-rate limiting lives inside `Drive.runVelocity` via PathPlanner's `SwerveSetpointGenerator`; requests pass raw targets and the generator handles the physics envelope.

Module geometry / gearing / IDs / gains live in [generated/TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java), generated by Tuner X. Only the constant declarations are kept — the `createDrivetrain()` factory and `TunerSwerveDrivetrain` inner class were removed.

### Vision — [src/main/java/frc/robot/subsystems/vision/](src/main/java/frc/robot/subsystems/vision/)

Multi-camera AprilTag pose fusion behind an AKit IO layer.

- [VisionIO](src/main/java/frc/robot/subsystems/vision/VisionIO.java) — interface; one impl per camera type.
- [VisionIOLimelight](src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java) — real Limelight impl. Pulls `botpose_*` from NT, applies rejection rules (ambiguity, distance, field bounds, angular-velocity gating), and emits a single best pose per cycle into [VisionInputsAutoLogged](src/main/java/frc/robot/subsystems/vision/VisionInputs.java). Filtering happens here so the inputs replay deterministically.
- [VisionIOPhotonVision](src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVision.java) and [VisionIOPhotonVisionSim](src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVisionSim.java) — base + sim subclass for PhotonVision coprocessor cameras. Only used in SIM by default (Limelight has no Java sim, so PhotonVision sim stands in); the real-PV class is available if a team switches hardware.
- [Vision](src/main/java/frc/robot/subsystems/vision/Vision.java) — subsystem that pushes orientation back to the cameras (for MegaTag2 / trig-solve), computes std-devs from logged tag distance / tag count, and either fuses time-synced poses across cameras or pushes them through individually. The std-dev coefficients live at the top of this file — they are the **vision tuning surface** for replay-based iteration.
- Camera names come from [VisionConstants](src/main/java/frc/robot/subsystems/vision/VisionConstants.java): default is one `limelight` on real and one `photon-front` in sim. Add entries to the arrays for more cameras.
- Std-dev formula: `coefficient * avgTagDist^1.2 / tagCount^2`, capped at `MAX_EFFECTIVE_TAG_COUNT = 2.5` because tags on the same wall correlate.
- Rejection: `MAX_AMBIGUITY = 0.3`, field-border margin 0.5 m, angular-velocity gating (`MT1: 360 deg/s`, `MT2: 200 deg/s`).
- Tunings borrow from 6328 Mechanical Advantage's published trust ratios, scaled to WPILib defaults.

## Path following

Pre-authored autonomous paths are followed by **PathPlannerLib** (distance-based, via the local fork at `c:/Users/joeoj/Downloads/pathplanner` published to mavenLocal). Paths and autos live under [src/main/deploy/pathplanner/](src/main/deploy/pathplanner/) as PathPlanner-native `.path` / `.auto` JSON. Choreo `.traj` exports are still accepted via `PathPlannerPath.fromChoreoTrajectory` for teams that prefer authoring there. The ChoreoLib runtime and the previous hand-rolled distance follower are gone.

### Pipeline

1. **`.path` and `.auto` files** under [src/main/deploy/pathplanner/](src/main/deploy/pathplanner/) authored in the PathPlanner desktop app (or imported from a Choreo `.traj`).
2. **`PathPlannerAutos`** ([src/main/java/frc/robot/commands/PathPlannerAutos.java](src/main/java/frc/robot/commands/PathPlannerAutos.java)) wires `AutoBuilder.configureDistanceBased` to `Drive.runVelocity` and exposes three factories:
   - `followPath(drive, name)` — load and follow a single `.path` file.
   - `runAuto(drive, name)` — runs a `.auto` file (sequenced paths + named-command actions).
   - `pathfindToPose(drive, goal)` — on-the-fly plan to an arbitrary goal, avoiding the static obstacle field registered in `configure`.
3. **`SwerveSetpointGenerator` inside `Drive.runVelocity`** enforces per-module slip, drive-motor torque, and steer-rate limits. It shares one `RobotConfig` (built by [DrivePhysics.buildRobotConfig()](src/main/java/frc/robot/subsystems/drive/DrivePhysics.java)) with the offline PathPlanner planner — so plan-time and runtime can't disagree on what the wheels can do.

Diagnostics land under `PathPlanner/*` (current pose, target pose, active path, target-state fields, `LastResult`).

### Other movement commands

- **`TeleopDrive`** — default teleop command. Translation from left stick (rescaled with deadband + squared magnitude), rotation from right stick. Wires the pose-based [ObstacleAvoidance](src/main/java/frc/robot/utils/path/ObstacleAvoidance.java) clamp; right-bumper bypass for emergencies.
- **`AxisLockDrive`** — teleop assist that locks X, Y, or heading. Locked-axis output is a feed-forward brake curve `v = sqrt(2·μg·distance)` clipped to chassis max — no PID, no profiler. Final speeds flow through `Drive.runVelocity` → PP setpoint generator for per-module slip/torque/steer-rate limits.
- **`PathPlannerAutos.pathfindToPose`** — on-the-fly plan + drive from current pose to a goal, avoiding the static obstacle field. Bound to driver A in [RobotContainer](src/main/java/frc/robot/RobotContainer.java).

## Logging

[Robot.java](src/main/java/frc/robot/Robot.java) wires AdvantageKit, switched on [Constants.getMode()](src/main/java/frc/robot/Constants.java):
- **REAL:** `WPILOGWriter` to `/home/lvuser/logs` + `NT4Publisher`.
- **SIM:** `WPILOGWriter` to `logs/` + `NT4Publisher`.
- **REPLAY:** `setUseTiming(false)`, `Logger.setReplaySource(WPILOGReader(System.getProperty("frc.replay.input")))`, output to a sibling `_replay.wpilog`. No NT publisher, no sim extensions (they're disabled in [build.gradle](build.gradle) when `-Preplay=...` is set).

Other knobs:
- Default WPILib loop period and watchdog timeout (no reflection hacks; the previous version's reflective `IterativeRobotBase.m_watchdog` override was removed).
- Per-cycle vision timing logged to `Timing/VisionMs`. No other `Timing/*` keys.

To activate REPLAY: `./gradlew simulateJava -Preplay=logs/<file>.wpilog`. `build.gradle` disables sim GUI + Driver Station + WebSocket extensions automatically when `-Preplay` is set, and `Robot.java` flips `setUseTiming(false)` so replay runs as fast as the CPU allows. The replay output is written to a sibling `_replay.wpilog`.

## Autonomous selection

The chooser in [AutoSelector](src/main/java/frc/robot/autonomous/AutoSelector.java) registers a handful of routines:
- `"NewPath (PathPlanner)"` (default) — runs the `New Auto.auto` file end-to-end via PathPlannerLib.
- `"Straight + U (back-to-back)"` — sequences `Straight` then `New Auto` to exercise back-to-back transitions.
- `"AxisLock test (drive-to-pose)"` — drives to a fixed target via `AxisLockDrive`, verifying the brake-curve drive-to-pose path.
- `"Straight (3m)"` — single-path smoke test.
- `"Pathfind Demo (reset + pathfind to goal)"` — resets to `DEMO_START` and runs `PathPlannerAutos.pathfindToPose(DEMO_GOAL)`, exercising the on-the-fly planner end-to-end.
- `"None"` — explicit `Commands.none()`. Safe default so a competition where no routine is selected doesn't accidentally execute path-following.

`DEMO_START` and `DEMO_GOAL` are `ExtPose` constants — blue-origin, auto-flipped via `.get()` at routine-build time. `DEMO_GOAL` is reused by the driver-A button binding in `RobotContainer` so both demos point at the same spot.

When adding new autos: author the `.path` / `.auto` in PathPlanner (or import a Choreo `.traj`), then add a `chooser.addOption("Name", () -> PathPlannerAutos.runAuto(drive, "FileName"))` call in `AutoSelector`.

## Hardware target

- **roboRIO 1 / 2** — JVM args in [build.gradle](build.gradle) pin a 100 MB heap with `+AlwaysPreTouch`, `UseSerialGC`, `GCTimeRatio=5`, `MaxGCPauseMillis=50`. Following the 254 / 6328 recipe.
- **CAN FD** for swerve modules + Pigeon2.
- **Brownout floor** is overridden to 6.0 V (`BROWNOUT_VOLTAGE` in [Robot.java](src/main/java/frc/robot/Robot.java)) — below WPILib's default of 6.75 V. Keeps motor outputs alive through transient battery sags during shooter spin-up / simultaneous module accel; raise if the chassis resets mid-match.

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
| Drive diagnostics              | `Drive/Diagnostics/ArcIntegrateRejections` |
| 250 Hz odometry collector      | [subsystems/drive/PhoenixOdometryThread.java](src/main/java/frc/robot/subsystems/drive/PhoenixOdometryThread.java) |
| Phoenix retry helper           | [utils/PhoenixUtil.java](src/main/java/frc/robot/utils/PhoenixUtil.java) |
| Swerve hardware constants      | [generated/TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java) |
| Friction / motor-curve math    | [subsystems/drive/DrivePhysics.java](src/main/java/frc/robot/subsystems/drive/DrivePhysics.java) (constants + `RobotConfig` factory; runtime limiter lives inside Drive's `SwerveSetpointGenerator`) |
| Obstacle layout + avoidance    | [Field2026Obstacles.java](src/main/java/frc/robot/Field2026Obstacles.java) returns PathPlanner-native `List<Pair<Translation2d, Translation2d>>` AABBs; the same list feeds [PathPlannerAutos](src/main/java/frc/robot/commands/PathPlannerAutos.java) (planner), [SimWorldSetup](src/main/java/frc/robot/simlib/SimWorldSetup.java) (dyn4j), [ObstacleVisualizer](src/main/java/frc/robot/utils/path/ObstacleVisualizer.java) (AdvantageScope), and [ObstacleAvoidance](src/main/java/frc/robot/utils/path/ObstacleAvoidance.java) (teleop brake clamp). |
| Path-following controller      | PathPlannerLib (external) — wired via [commands/PathPlannerAutos.java](src/main/java/frc/robot/commands/PathPlannerAutos.java) |
| Auto building blocks           | [commands/PathPlannerAutos.java](src/main/java/frc/robot/commands/PathPlannerAutos.java) + [autonomous/AutoSelector.java](src/main/java/frc/robot/autonomous/AutoSelector.java) |
| Vision IO + fusion             | [subsystems/vision/](src/main/java/frc/robot/subsystems/vision/)       |
| Geometry helpers               | [utils/geometry/](src/main/java/frc/robot/utils/geometry/) (`ExtPose`, `ExtTranslation`, `ExtRotation` for alliance-aware blue-origin constants) |
| Sim startup hook (headless)    | [utils/SimStartup.java](src/main/java/frc/robot/utils/SimStartup.java) |
| Season-specific field geometry | [Field2026Constants.java](src/main/java/frc/robot/Field2026Constants.java) + [Field2026Obstacles.java](src/main/java/frc/robot/Field2026Obstacles.java) — replace these each season |
| Reference subsystems (copy-and-rename) | [subsystems/examples/](src/main/java/frc/robot/subsystems/examples/) — `Flywheel` (velocity control) and `Arm` (profiled position control) |
