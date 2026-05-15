# 2026 Robot Template

FRC swerve drive template built around CTRE Phoenix 6, AdvantageKit, MapleSim physics, and Choreo path following. Use this as the starting point for a new season's robot code.

## What you get out of the box

- **Swerve drive** with 250 Hz odometry on a separate CAN bus thread (Phoenix Pro–ready)
- **Three modes** — `REAL` on the roboRIO, `SIM` with MapleSim physics on your laptop, `REPLAY` against a saved log
- **Vision** — Limelight on the real robot, PhotonVision sim in SIM (with PR #1767 PNP trig-solve for single tags), distance-weighted standard deviations
- **Distance-based path following** using Choreo `.traj` files re-parameterized by arc length
- **Runtime path planning around obstacles** — driver A button plans a route from current pose to a goal, avoiding the field obstacle layout
- **Acceleration limiting** with a friction-circle model so the wheels don't slip
- **Unified logging** — everything the drivetrain knows lives under `Drive/*` (see [Logging](#logging))
- **Two reference subsystems** under [subsystems/examples/](src/main/java/frc/robot/subsystems/examples/) — a velocity-controlled flywheel and a position-controlled arm — to copy when you add real mechanisms

---

## Quick start

Five steps from cloning the repo to a robot you can drive in sim.

### 1. Install prerequisites

| Tool | Where to get it | Notes |
|------|----------------|-------|
| WPILib 2026 | https://github.com/wpilibsuite/allwpilib/releases | Installs Java 17, VS Code, and the WPILib extension |
| Phoenix Tuner X | https://www.ctr-electronics.com/tools/tuner-x/ | For configuring TalonFX/CANcoder/Pigeon |
| Choreo | https://choreo.autos | Path planner |
| AdvantageScope | https://github.com/Mechanical-Advantage/AdvantageScope/releases | Log viewer |
| (Optional) Git | https://git-scm.com | For source control |

### 2. Clone, set your team number, and build

```bash
git clone <repo url> 2026-Robot
cd 2026-Robot
```

Open the folder in WPILib VS Code (`File → Open Folder…`), then edit [.wpilib/wpilib_preferences.json](.wpilib/wpilib_preferences.json) so `"teamNumber"` matches your team (or use the WPILib VS Code command `WPILib: Set Team Number`).

```bash
./gradlew build           # macOS / Linux
gradlew.bat build         # Windows
```

First build takes a few minutes — it downloads WPILib, CTRE Phoenix, AdvantageKit, ChoreoLib, and `dyn4j`. If you see *"Could not get unknown property 'teamNumber'"* you missed the team-number step.

### 3. Run the simulator

```bash
./gradlew simulateJava
```

The sim GUI opens. In the DriverStation window, click **Teleoperated** and check **Enable**. Plug in an Xbox controller:

| Control | Action |
| --- | --- |
| Left stick | Translate |
| Right stick | Rotate |
| A button | Plan a path to a demo goal, avoiding obstacles |
| Right bumper (hold) | Bypass the obstacle-avoidance brake |

MapleSim simulates wheel slip, friction, and inertia. AdvantageScope can connect to NetworkTables (`localhost`) and show pose, modules, vision, and obstacles live.

### 4. Configure your swerve hardware

[generated/TunerConstants.java](src/main/java/frc/robot/generated/TunerConstants.java) is **generated** by Phoenix Tuner X → Swerve Project Generator. Open Tuner, connect to your robot, run the generator with your module geometry and encoder offsets, and replace the file's contents. **Do not hand-edit it** — re-running Tuner will overwrite changes. The defaults for robot mass, friction, and CoG offsets in [lib/dynamics/AccelerationLimiter.java](src/main/java/frc/robot/lib/dynamics/AccelerationLimiter.java) work for most teams.

### 5. Add a subsystem or auto routine

See [Adding your first subsystem](#adding-your-first-subsystem) and [Adding an autonomous routine](#adding-an-autonomous-routine) below.

---

## Running the code

### Simulation

```bash
./gradlew simulateJava                                # GUI sim, manual DS control
./gradlew simulateJavaAgent                           # headless, auto-enters autonomous
./gradlew simulateJavaAgent -Pmode=teleop             # headless, starts in teleop
./gradlew simulateJavaAgent -Pmode=disabled           # headless, stays disabled
```

### Replay a saved log

```bash
./gradlew simulateJava -Preplay=logs/<your-log>.wpilog
```

The robot code re-runs against the saved sensor data and writes a new log with `_replay.wpilog` appended. Open both in AdvantageScope and plot `Drive/Pose` from each on the same 2D-Field view — they should overlay exactly. The constructor's `drivetrain.resetPose(SIM_SPAWN_POSE)` is intentionally outside the SIM-only block so REPLAY reproduces absolute poses, not just trajectory shape.

### Deploy to a real robot

```bash
./gradlew deploy
```

The robot must be connected via USB, Ethernet, or radio. Check the RioLog (WPILib VS Code → `WPILib: Start RioLog`) for output.

---

## Configuring for your robot

### Module + motor IDs (TunerConstants)

`src/main/java/frc/robot/generated/TunerConstants.java` is regenerated from **Phoenix Tuner X → Swerve Project Generator**. Re-export and overwrite that file whenever module geometry, gear ratios, or encoder offsets change.

Anything *not* in `TunerConstants` lives in:

- `lib/dynamics/AccelerationLimiter.java` — robot mass, wheel friction, CoG offset
- `subsystems/drive/Drive.java` — sim-only configuration in `getMapleSimConfig()`
- `subsystems/drive/ModuleIOSim.java` — sim-only PID gains (intentionally differ from real)
- `Robot.java` — `BROWNOUT_VOLTAGE` (default 6.0 V, below WPILib's 6.75 V) keeps motor outputs alive through transient battery sags during shooter spin-up or simultaneous module accel. Raise if the chassis browns out and resets mid-match

### Field info

`utils/FieldInfo.java` holds game-agnostic field state (dimensions, AprilTag layout, alliance flip). Game-specific geometry (Hub, Tower, bumps) lives in `Field2026Constants.java` and `Field2026Obstacles.java` — replace those files each season.

### Vision

The wiring in `RobotContainer.createVision` differs by mode:

- **REAL** — Limelight (`VisionIOLimelight`). Names in `LIMELIGHT_NAMES` must match each camera's NetworkTables name (set in the LL web UI). Add entries to the array for more cameras.
- **SIM** — Simulated PhotonVision (`VisionIOPhotonVisionSim`) because Limelight has no Java sim. Names in `PHOTON_CAMERA_NAMES`; per-camera mounts in `PHOTON_CAMERA_TRANSFORMS` — **tune against your CAD** before trusting trig-solve distances. The sim is fed from MapleSim ground truth in `updateSimulation`.
- **REPLAY** — both name sets are registered as `VisionIONoop` so logs from either source replay correctly.

---

## Adding your first subsystem

Copy one of the reference subsystems under [subsystems/examples/](src/main/java/frc/robot/subsystems/examples/) and rename it.

- [flywheel/](src/main/java/frc/robot/subsystems/examples/flywheel/) — velocity-controlled motor (shooter, intake roller)
- [arm/](src/main/java/frc/robot/subsystems/examples/arm/) — position-controlled joint (arm, wrist, elevator after small tweaks)

Each example ships four files following the AdvantageKit IO pattern:

1. `XxxIO.java` — interface + auto-logged inputs
2. `XxxIOTalonFX.java` — real hardware (TalonFX with Motion Magic or velocity control)
3. `XxxIOSim.java` — physics sim using WPILib's `FlywheelSim` / `SingleJointedArmSim`
4. `Xxx.java` — the subsystem itself, wiring IO + commands

Wire your new subsystem in [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java) by adding a field and a switch on `Constants.getMode()`:

```java
public final Shooter shooter;
// ...
shooter = switch (Constants.getMode()) {
  case REAL   -> new Shooter(new ShooterIOTalonFX(20));
  case SIM    -> new Shooter(new ShooterIOSim());
  case REPLAY -> new Shooter(new ShooterIO() {});
};
```

Bind it to a button in `configureBindings()`:

```java
driver.x().whileTrue(shooter.runAtRPM(3000));
```

---

## Adding an autonomous routine

### Option A — Choreo trajectory (the usual case)

1. Open [Choreo](https://choreo.autos) and point it at the project root. Draw a path in the visual editor and save.
2. Add the path to the [AutoPath](src/main/java/frc/robot/utils/path/AutoPath.java) enum. Entries reference `ChoreoTraj` constants (regenerated automatically when you save in Choreo), so typos fail at compile time:
   ```java
   public enum AutoPath {
     NEW_PATH(ChoreoTraj.NewPath),
     MY_AUTO(ChoreoTraj.MyAuto),
   }
   ```
3. Register a routine in [AutoSelector.java](src/main/java/frc/robot/autonomous/AutoSelector.java):
   ```java
   chooser.addOption("My Auto", () -> autoCommands.followPathWithActions(
       AutoPath.MY_AUTO.get(), Map.of()));
   ```

Then pick it from the **Auto Mode** dropdown in the DriverStation.

### Option B — Drive somewhere with obstacle avoidance

For "drive to that scoring location, plan around whatever's in the way," use `DriveToWithAvoidance`. The same command is bound to the driver A button — it works in autos too:

```java
chooser.addOption("Goto Scoring",
    () -> DriveToWithAvoidance.create(drive, () -> SCORING_POSE, obstacleField));
```

The planner runs **at trigger time** using the current pose as the start, so a single binding handles different starting positions.

---

## Project layout

```
src/main/java/frc/robot/
├── Robot.java                  - Top-level robot lifecycle, AKit logger setup
├── RobotContainer.java         - Subsystem wiring, auto chooser, default commands
├── Constants.java              - Mode detection (REAL/SIM/REPLAY)
├── Field2026Constants.java     - 2026 game geometry (Hub, Tower, bumps) — yearly throwaway
├── Field2026Obstacles.java     - 2026 obstacle layout for planner + avoidance clamp
├── generated/
│   └── TunerConstants.java     - Generated by Phoenix Tuner X — don't hand-edit
├── subsystems/
│   ├── drive/                  - Swerve subsystem + per-mode IO implementations
│   ├── vision/                 - Vision subsystem + Limelight (real) / PhotonVision (sim) IO
│   └── examples/               - Reference Flywheel + Arm subsystems (copy these for your robot)
├── commands/
│   ├── TeleopDrive.java        - Default teleop command
│   ├── FollowPath.java         - Distance-based path follower (PD + lookahead)
│   ├── DriveToPoint.java       - Drive to a fixed pose with profiled velocity
│   └── DriveToWithAvoidance.java - Plan around obstacles then drive (driver A button)
├── autonomous/
│   ├── AutoCommands.java       - Helpers for assembling auto routines
│   └── AutoSelector.java       - Dashboard chooser + registered routines
├── lib/
│   └── dynamics/               - Acceleration limiter, motor curves (rarely modified)
├── utils/                      - Geometry helpers, path infra, Limelight SDK
└── simlib/                     - Vendored MapleSim physics
```

---

## Key concepts

### Three modes, one codebase

`Constants.getMode()` returns one of:

| Mode | Where | What it does |
| --- | --- | --- |
| `REAL` | roboRIO | Talks to real motors, gyro, and cameras |
| `SIM` | Your laptop | MapleSim physics + simulated PhotonVision |
| `REPLAY` | Your laptop | Re-runs a saved log against the same code path, as fast as the CPU allows |

Replay is the killer feature: a bug that happened on the field can be debugged from the log without the robot present.

### IO pattern (AdvantageKit)

Every subsystem talks to hardware through an interface called `XxxIO`. For each hardware target we ship one implementation per mode:

- `ModuleIOTalonFX` — real CTRE modules
- `ModuleIOSim` — MapleSim physics
- `ModuleIO {}` (no-op) — REPLAY reads from the log instead of hardware

`Constants.getMode()` decides which one is wired up. The no-op IO is what makes replay deterministic.

### 250 Hz fast loop

The main robot loop runs at 50 Hz, but swerve odometry and the friction limiter need higher rates. A `Notifier` thread inside `Drive.java` runs at 250 Hz. Commands hook into it by passing a `SwerveRequest` to `drive.setControl(...)` — see `commands/FollowPath.java` for an example.

**Do not call `Logger.recordOutput` from a SwerveRequest's `apply()`** — AKit's log buffer isn't thread-safe. Stash diagnostics in volatile fields and let the subsystem log them from `periodic()`.

### Distance-based path following

`FollowPath` projects the robot onto the path and asks "how far along am I?" — not "what time is it?". If the robot stalls or gets bumped, the path waits. The follower uses:

- Arc-length re-parameterization of Choreo trajectories (`utils/path/ArcLengthTrajectory.java`)
- A speed-dependent lookahead point
- PD cross-track correction
- Curvature feedforward

---

## Logging

All drive-state logs live under one `Drive/*` tree, modeled after CTRE's `SwerveDrivetrain.getState()`. The whole tree updates every robot loop.

| Key | Meaning |
|-----|---------|
| `Drive/Pose` | Estimator pose (where the robot *thinks* it is) |
| `Drive/RawHeading` | Raw gyro yaw, before any vision fusion |
| `Drive/Speeds` | Measured robot-relative chassis speeds (m/s, rad/s) |
| `Drive/FieldSpeeds` | Measured field-relative chassis speeds |
| `Drive/TranslationSpeedMps` | Speed magnitude |
| `Drive/RotationSpeedRadPerSec` | Yaw rate magnitude |
| `Drive/ModuleStates` | Measured module states (speed + angle) |
| `Drive/ModuleTargets` | Commanded module states |
| `Drive/ModulePositions` | Module distance + angle (used by the pose estimator) |
| `Drive/SetpointSpeeds` | Target chassis speeds |
| `Drive/Gyro/*` | Processed gyro inputs (connected, yaw, odometry samples) |
| `Drive/Module<0-3>/*` | Processed per-module inputs |
| `Drive/Sim/GroundTruthPose` | **(sim only)** the physics pose — where the robot *actually* is |
| `Drive/Sim/PoseErrorMeters` | **(sim only)** distance between estimator pose and physics pose |
| `Drive/Sim/HeadingErrorRad` | **(sim only)** heading delta between estimator and physics |
| `Drive/Diagnostics/ArcIntegrateRejections` | Counter — odometry samples rejected for non-finite inputs |
| `Drive/Diagnostics/FrictionRatios` | Per-module friction utilization (a_i / mu*g) |
| `World/Obstacles/Rectangles` | Field obstacles as native `Rectangle2d[]` (AdvantageScope renders directly) |
| `World/Obstacles/Ellipses` | Field obstacles as native `Ellipse2d[]` |

To compare estimator vs. ground truth in sim, plot `Drive/Pose` and `Drive/Sim/GroundTruthPose` together in AdvantageScope's 2D field view, or watch `Drive/Sim/PoseErrorMeters` on a line graph.

---

## AdvantageScope: install the 3D robot model

1. Open AdvantageScope.
2. Menu bar → `App` (macOS) or `AdvantageScope` (Windows/Linux) → `Show Assets Folder`.
3. Download `Robot_2026` from https://drive.google.com/drive/folders/1M_KRpmZVoDL1LgyMo935e72sWXf8dQNe?usp=sharing
4. Copy the entire `Robot_2026` folder into the assets folder you just opened.
5. Restart AdvantageScope. Select `Robot_2026` from the 3D field's robot dropdown.

Custom articulated parts order:

| Index | Part |
|-------|------|
| 0 | Robot chassis |
| 1 | Turret |
| 2 | Hood |
| 3 | Front 4-bar arm |
| 4 | Back 4-bar arm |
| 5 | Intake structure |

Turret/Hood and the three turret components are grouped together.

---

## Troubleshooting

| Symptom | Try this |
|---------|----------|
| "No Robot Code" on DS, build is fine | Check `Constants.getMode()` — `REPLAY` mode runs faster than real time and disables the DS connection |
| Sim chassis floats away during auto | `resetPose()` must run before the path starts; check the auto sequence in `AutoSelector.java` |
| Vision pose jitter | Bump `XY_STD_DEV_BASE` in `Vision.java`; tags farther away get distrusted more |
| Modules misaligned at startup | Encoder offsets in `TunerConstants` are stale — re-run Tuner X's swerve generator |
| "Disconnected gyro" alert in sim | Expected — the alert is suppressed in `Mode.SIM`. If you see it, something is misconfigured |
| Driver-A planner button does nothing | Check the `DriveToWithAvoidance/PlanFailed` log key and the DS Alert — usually means the goal is unreachable or the robot is inside an obstacle |
| Replay's absolute poses are offset from the original | The constructor's `drivetrain.resetPose(SIM_SPAWN_POSE)` must run in REPLAY mode too (it does in this template) — if you move it back inside the SIM-only block, replay will start at the origin |

For deeper issues, drop a `.wpilog` from `logs/` into AdvantageScope and look at the `Drive/Diagnostics/*` entries. The unit-test suite (`./gradlew test`) covers the friction limiter, the path planner, and the field obstacle layout, so a regression there will fail loudly.

---

## Working with Claude Code on this project

If you use [Claude Code](https://claude.com/claude-code), this repo ships project-specific knowledge so Claude understands the codebase without re-deriving it every session.

### Project skills (Claude reads these automatically)

Under [.agent/skills/](.agent/skills/):

| File | What it covers |
|------|----------------|
| `robot-description.md` | Subsystem layout, IO pattern, 250 Hz threading model, key invariants Claude should respect when editing |
| `game-info.md` | 2026 field conventions — blue-origin coordinates, ROTATE symmetry, where alliance flipping happens, named field zones |
| `log-reading.md` | What each `Drive/*` log key means, how to read replay logs, common diagnostic patterns |
| `simulation-agent.md` | How to drive the headless sim (`simulateJavaAgent`), what the `-Pmode=` flags do, expected boot output |

These are agent-facing — they describe the codebase to Claude. You don't run them; they make Claude's edits more accurate and consistent.

### Useful global slash commands

Available from any Claude Code session (no per-project setup):

| Command | Purpose |
|---------|---------|
| `/simplify` | Three-agent pass over recent changes for code reuse, quality, and efficiency. Useful after any significant edit. |
| `/review` | PR-style review of recent changes. |
| `/security-review` | Security-focused audit of the pending diff. |
| `/init` | Generate or refresh a `CLAUDE.md` that primes Claude on the codebase. |

### Things Claude does well on this codebase

- Adding a subsystem from the example pattern (see [Adding your first subsystem](#adding-your-first-subsystem))
- Writing or tuning a Choreo-driven auto routine
- Wiring a button binding to a new command
- Adding a JUnit test for math-heavy utilities (planner, limiter, geometry)
- Investigating an AdvantageKit log to diagnose a behavior issue

### Things to double-check

- 250 Hz fast-path edits — Claude may suggest scratch-field tricks on shared objects (`Obstacle` records) that are not thread-safe. The skill docs above call this out, but verify against the codebase if you see it.
- Choreo path generation — Claude can edit `AutoPath` and `AutoSelector`, but cannot author `.traj` files; use the Choreo GUI for that.
