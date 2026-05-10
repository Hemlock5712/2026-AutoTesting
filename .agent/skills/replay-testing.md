---
name: Replay Testing
description: Use AdvantageKit replay to validate code changes. Re-run the robot's loop against a recorded log so you can see exactly how a controller / vision std-dev / heuristic change would have behaved during a previous match or sim run, without re-driving anything. Sim ↔ replay is bit-identical (verified at 0.0 m delta).
---

# Replay Testing

The minimum agent loop for this codebase is:

> **read logs → replay/simulate → compare → iterate**

This skill covers the **replay** half — taking a `.wpilog` from a previous run and re-executing the robot's logic against the recorded sensor stream. Replay runs as fast as the CPU permits (no real-time pacing).

## Why replay, not "just run sim again"

Sim is non-deterministic in a few places (timing jitter, vision noise generation, sim physics). Replay is deterministic: AdvantageKit feeds the recorded `/AdvantageKit/...` inputs into the same code paths your robot just ran, so any output difference is purely a function of the code change. **Verified bit-identical: a 53-second sim with vision injection produces the same final pose to 0.000000 m when replayed without code changes.**

## Architecture (full AKit pattern)

The drive subsystem is a direct port of the [AdvantageKit talonfx_swerve template](https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/template_projects/sources/talonfx_swerve). One [Drive](src/main/java/frc/robot/subsystems/drive/Drive.java) subsystem with a Java `SwerveDrivePoseEstimator` runs in **all** modes — there is no CTRE `SwerveDrivetrain`, no `setControl(SwerveRequest)`, no concurrent CTRE odometry thread. This is what makes determinism possible: the only thing that changes between modes is the IO layer.

| Mode | GyroIO | ModuleIO | What runs |
| --- | --- | --- | --- |
| REAL | [GyroIOPigeon2](src/main/java/frc/robot/subsystems/drive/GyroIOPigeon2.java) | [ModuleIOTalonFX](src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java) | Hardware via Phoenix6, 250 Hz odometry via [PhoenixOdometryThread](src/main/java/frc/robot/subsystems/drive/PhoenixOdometryThread.java) |
| SIM | [GyroIOSim](src/main/java/frc/robot/subsystems/drive/GyroIOSim.java) | [ModuleIOSim](src/main/java/frc/robot/subsystems/drive/ModuleIOSim.java) | maple-sim rigid-body physics (dyn4j); ~250 Hz effective via 5 sub-tick samples per 20 ms cycle |
| REPLAY | empty | empty `ModuleIO {}` | AKit replays `@AutoLog` inputs from the WPILOG; estimator runs the same code |

Vision pose-estimator weights live in [Vision.java](src/main/java/frc/robot/subsystems/vision/Vision.java) — these are the **primary tuning surface** for replay-based iteration.

## Running a replay

```powershell
./gradlew simulateJava -Preplay=logs/akit_26-05-09_02-49-08.wpilog -Pheadless
```

The `-Pheadless` flag isn't strictly required, but the gradle file disables sim GUI / DriverStation / WebSocket extensions when `-Preplay=...` is set anyway. AdvantageKit refuses to run with sim extensions enabled and the build will fail loudly if they're on.

The replay output goes to `logs/<input>_replay.wpilog`. Open it in AdvantageScope or pass it to the [Log Reading](.agent/skills/log-reading.md) skill.

### What's in the replay output

The output log contains both:

- `/RealOutputs/...` — copied forward from the source log (what *actually* happened)
- `/ReplayOutputs/...` — what *would have* happened with the current code

The keys are symmetric — same suffix under both prefixes — so you can pull both columns side by side and compute a delta.

Critical: when comparing, make sure you compare `/RealOutputs/Drive/Pose` (source's pose) vs `/ReplayOutputs/Drive/Pose` (replay's pose). If you compare `/RealOutputs/...` against itself in the replay log you'll see zero delta — that's just the source data carried forward, not a verification of anything.

## Log inspection scripts

Three small WPILOG helpers in [scripts/](scripts/), all using `wpiutil.log.DataLogReader`:

- **[scripts/compare_poses.py](scripts/compare_poses.py)** — reads the *last* `Drive/Pose` entry from each log and prints the delta. The default tool for replay regression checks.

  ```powershell
  python scripts/compare_poses.py logs/akit_X.wpilog logs/akit_X_replay.wpilog
  ```

  For an unaltered replay, expect **0.0000 m delta**. Anything non-zero indicates a code change between record and replay (or a bug in the IO logging layer).

- **[scripts/compare_poses_at_time.py](scripts/compare_poses_at_time.py)** — same idea, but samples poses at matching timestamps instead of just the last entry. Use this when AKit's record-dedup makes the "last pose" comparison misleading (e.g. one log keeps logging while the bot is parked because vision is still updating). Supports `--stride <seconds>` to walk through the run.

- **[scripts/check_vision_in_log.py](scripts/check_vision_in_log.py)** — prints `True/False` for whether a wpilog contains any vision observations. Useful when triaging why a replay's pose tracking diverged: was vision actually feeding the source log, or did the camera publish nothing?

## Tuning vision std-devs against a log

1. Run a sim or pull a real-match log with vision observations: `logs/<file>.wpilog`.
2. Edit the std-dev coefficients in [Vision.java](src/main/java/frc/robot/subsystems/vision/Vision.java#L20-L24):
   - `XY_STD_DEV_COEFFICIENT`
   - `ROTATION_STD_DEV_COEFFICIENT`
   - `MEGATAG2_ROTATION_STD_DEV`
   - `MAX_EFFECTIVE_TAG_COUNT`
3. `./gradlew simulateJava -Preplay=logs/<file>.wpilog -Pheadless`.
4. Compare `/ReplayOutputs/Drive/Pose` against the original `/RealOutputs/Drive/Pose` (or against ground truth).
5. If the new std-devs land closer to truth and nothing else regressed → ship it. Otherwise → revert and iterate.

Replay is fast enough that you can sweep coefficients in a script — invoke gradle in a loop, parse the resulting `_replay.wpilog`, plot. **Verified responsiveness:** changing `XY_STD_DEV_COEFFICIENT` from 0.333 to 33.3 (100× looser) produced a 15.22 m shift in the replayed final pose against the same log — std-dev tuning has measurable, predictable effects.

## Caveats

- **Replay has no hardware:** anything that talks to NT or hardware directly during replay will misbehave. The Vision IO impl ([VisionIOLimelight](src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java)) is bypassed in REPLAY mode (RobotContainer wires an empty `VisionIO` no-op instead) — vision observations come from the AKit-replayed `@AutoLog` inputs.
- **Build-time check:** AdvantageKit refuses to start replay if any `wpi.sim.*` extension is registered. The gradle file gates these on `!isReplay` so this stays automatic, but if you add new sim extensions remember to gate them similarly.
- **Sim odometry runs at ~250 Hz effective via maple-sim sub-ticks.** Each `ModuleIOSim.updateInputs` (called at 50 Hz) writes `SimulatedArena.getSimulationSubTicksIn1Period()` samples — defaults to 5, so 250 Hz effective. The pose estimator walks all sub-tick samples, just like the real bot's 250 Hz `PhoenixOdometryThread`. Maple-sim handles rigid-body physics; vision pose comes from the sim ground truth via [VisionIOSim](src/main/java/frc/robot/subsystems/vision/VisionIOSim.java).

## Typical agent loop

1. Run a sim or pull a real-match log → `logs/<file>.wpilog`.
2. Make the code change you want to evaluate.
3. `./gradlew simulateJava -Preplay=logs/<file>.wpilog -Pheadless`.
4. Compare `/RealOutputs/...` against `/ReplayOutputs/...` (use `scripts/compare_poses.py` for poses, AdvantageScope for time series).
5. If the replay outputs are better and nothing else regressed → ship it. Otherwise → revert and iterate.
