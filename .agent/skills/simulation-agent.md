---
name: Simulation Agent
description: Run robot simulation through the simulateJavaAgent Gradle task for automated auto / teleop testing — headless, no GUI, mode controlled from the command line.
---

# Simulation Agent

Use this skill whenever you need to **run the robot in simulation from a non-interactive context** — agent loops, CI, scripted testing — and have it actually start playing instead of sitting on the disabled screen waiting for a human to click "Enable."

The entry point is the `simulateJavaAgent` Gradle task. It launches the same robot code as `simulateJava`, but:

1. **Skips the sim GUI** (no Swing window). Faster startup, no display required.
2. **Auto-enables** at `robotInit` via `DriverStationSim`, in the mode you ask for.
3. **Keeps the HALSim WebSocket and NT4 servers up**, so external clients (NT4 viewers, AdvantageScope live, ad-hoc Python scripts) can still attach during the run.
4. **Still writes WPILOGs to `logs/`**, just like the GUI sim — feed those to the `Log Reading` skill.

Everything is wired in [build.gradle](build.gradle), [SimStartup.java](src/main/java/frc/robot/utils/SimStartup.java), and [Robot.java](src/main/java/frc/robot/Robot.java).

## CLI usage

```powershell
# Headless sim, robot enters autonomous immediately. This is the default.
./gradlew simulateJavaAgent

# Headless sim, robot enters teleop immediately.
./gradlew simulateJavaAgent -Pmode=teleop

# Headless sim, robot stays disabled until something external (NT4 / HALSim WS client) commands it.
./gradlew simulateJavaAgent -Pmode=disabled

# Standard GUI sim — unchanged, ignore this skill if that's what you want.
./gradlew simulateJava
```

You can also apply the agent flags to the regular `simulateJava` task if you want, e.g. `./gradlew simulateJava -Pheadless -Pmode=auto`. The `simulateJavaAgent` task just makes that the default.

### Project properties (agent flags)

| Property        | Effect                                                           |
| --------------- | ---------------------------------------------------------------- |
| `-Pheadless`    | Skip `wpi.sim.addGui()`. Implied by `simulateJavaAgent`.         |
| `-Pmode=auto`   | Arm DS as `autonomous=true, enabled=true`. Default for `Agent`.  |
| `-Pmode=teleop` | Arm DS as `autonomous=false, enabled=true`.                      |
| `-Pmode=disabled` (or omitted on `simulateJava`) | Robot stays disabled.       |

## What "auto-enable" actually does

[SimStartup.arm()](src/main/java/frc/robot/utils/SimStartup.java) runs once in the `Robot` constructor (sim only). It reads `frc.sim.startMode` (set by build.gradle) and calls:

```java
DriverStationSim.setAutonomous(true|false);
DriverStationSim.setEnabled(true);
DriverStationSim.setDsAttached(true);
DriverStationSim.notifyNewData();
DriverStationJNI.observeUserProgramStarting();
```

(In WPILib 2026 the call moved from `HAL.observeUserProgramStarting()` to `DriverStationJNI.observeUserProgramStarting()` — if you see compile errors mentioning `HAL.observe...`, you're on stale 2025 code.)

When the simulated DS is `defaultEnabled = true` *and* `arm()` has set the autonomous flag, the robot proceeds straight from `robotInit` into `autonomousInit` / `teleopInit` on the next iteration. No human interaction.

## Verifying it worked

In the gradle output, you should see:

```
[SimStartup] Headless start: enabled=true autonomous=true (mode=auto)
```

If that line is missing:
- The task probably wasn't `simulateJavaAgent` and you didn't pass `-Pmode=…`.
- You're running on real hardware (helper is a no-op outside of `RobotBase.isSimulation()`).

## Running an entire auto and inspecting the result

The typical agent loop:

1. `./gradlew simulateJavaAgent` (headless, starts in auto)
2. Wait until either:
   - `FollowPath/Progress` reaches ~1.0 on NetworkTables, or
   - The robot disables itself (auto routine ends), or
   - Some timeout you've set
3. Stop the gradle process (Ctrl+Break / SIGINT — flushes the WPILOG)
4. Pass the most recent `logs/FRC_*.wpilog` to the **[Log Reading](.agent/skills/log-reading.md)** skill for analysis

For after-the-fact validation of a code change against an existing log, use replay mode — it doesn't need a live sim at all:

```
./gradlew simulateJava -Preplay=logs/<file>.wpilog
```

The code re-runs against the saved sensor data and writes a new `_replay.wpilog` next to the input. `build.gradle` disables sim GUI / Driver Station / WebSocket extensions automatically when `-Preplay=…` is set, and `Robot.java` flips `setUseTiming(false)` so replay runs as fast as the CPU allows. Open both logs in AdvantageScope and overlay `Drive/Pose` on the 2D field view to verify the replay matches the original run.

## When NOT to use this skill

- You're iterating on visualization (Glass widgets, AdvantageScope live view) — keep the GUI sim.
- You want to use a physical joystick — sim DS GUI's joystick remap is unavailable headless.
- You're testing real hardware behavior that can't be simulated (CTRE motor closed-loop response, real Limelight tags) — sim only models a subset.

## Common pitfalls

- **`-Pmode=foo` silently ignored.** SimStartup logs an unknown mode to stderr but otherwise does nothing — robot stays disabled.
- **Process won't exit cleanly on Windows.** `gradlew.bat simulateJavaAgent` spawns a daemon; use `Ctrl+Break` rather than `Ctrl+C` if you want the WPILOG to flush (Python's `subprocess` equivalent is sending `CTRL_BREAK_EVENT` to the process group).
- **HALSim WS port already in use.** Another sim is still running. Kill it with `Stop-Process -Name java` before retrying.
