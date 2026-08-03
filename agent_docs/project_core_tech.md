# Core technology baseline

## WPILib 2027 alpha toolchain

- Java source and target compatibility are Java 25.
- The Gradle wrapper is Gradle 9.4.1; the build applies `org.wpilib.GradleRIO` `2027.0.0-alpha-6` and resolves the local WPILib `2027_alpha5` Maven installation.
- Deployment targets WPILib SystemCore. The `wpilibJava` artifact is the shaded jar, the static deploy tree is `/home/systemcore/deploy`, and real-robot WPILOG output is `/home/systemcore/logs`.
- Desktop support remains enabled with native simulation dependencies, the simulation GUI, and Driver Station simulation.

The continuation runtime requires `--add-opens=java.base/jdk.internal.vm=ALL-UNNAMED` and `--add-opens=java.base/java.lang=ALL-UNNAMED`. The build applies them to the SystemCore artifact, tests, and `JavaExec` tasks.

## Vendor dependencies and SystemCore simulation boundary

The checked-in vendor set is AdvantageKit `27.0.0-alpha-4`, Commands v3 `1.0.0` (`org.wpilib:commands3-java`, version `wpilib`), and CTRE Phoenix 6 `26.50.0-alpha-1`. Jackson annotations/core/databind are pinned to `2.19.2`.

Phoenix software-simulation (`swsim`) binaries remain desktop-only because the alpha does not publish them for Linux SystemCore. The two hardware JNI entries retain `linuxsystemcore`, allowing deployment without disabling desktop simulation.

## Commands v3 model

Commands v3 schedules coroutine-backed `Command` instances through a `Scheduler`; `Mechanism` instances own requirements, default commands, and periodic callbacks. `Robot.robotPeriodic()` runs the default scheduler after the superstructure update and before tunable updates.

The seven stateful command classes directly implement `Command`: `AxisLockDrive`, `DriveToPoint`, `FollowPath`, `GamePieceDrive`, `JamProtectedShoot`, `OrbitDrive`, and `TurretDrive`. Each declares its name and mechanism requirements, initializes per-schedule state at the start of `run(Coroutine)`, executes native control flow with a yield on every continuing iteration, and uses `onCancel()` for hardware-safe interruption. Finite commands perform natural cleanup before returning. Runtime failures are cleaned up and rethrown as `RuntimeException`, matching the alpha-6 scheduler's command-removal contract.

`Commands` remains the small composition/factory layer used by call sites, while `RobotMechanism` provides mechanism-owned one-shot/repeating commands and periodic registration.

The architecture is guarded by `NativeCommandArchitectureTest`; the verified full build passes 9/9 tests.
