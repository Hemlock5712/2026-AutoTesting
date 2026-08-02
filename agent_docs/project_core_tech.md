# Core technology baseline

## WPILib 2027 alpha toolchain

- Java source and target compatibility are Java 25.
- The Gradle wrapper is Gradle 9.4.1; the build applies `org.wpilib.GradleRIO` `2027.0.0-alpha-6` and resolves the local WPILib `2027_alpha5` Maven installation.
- Deployment targets WPILib SystemCore. The `wpilibJava` artifact is the shaded jar, the static deploy tree is `/home/systemcore/deploy`, and the real-robot WPILOG output is `/home/systemcore/logs`.
- Desktop support remains enabled (`includeDesktopSupport = true`) with native simulation dependencies, the simulation GUI, and Driver Station simulation enabled by default.

The continuation runtime requires these JVM opens wherever the project runs Java: `--add-opens=java.base/jdk.internal.vm=ALL-UNNAMED` and `--add-opens=java.base/java.lang=ALL-UNNAMED`. They are applied to the SystemCore artifact, tests, and `JavaExec` tasks.

## Vendor dependencies and SystemCore simulation boundary

The checked-in vendor set is AdvantageKit `27.0.0-alpha-4`, Commands v3 `1.0.0` (`org.wpilib:commands3-java`, version `wpilib`), and CTRE Phoenix 6 `26.50.0-alpha-1`. Jackson annotations/core/databind are pinned to `2.19.2` in the Gradle build.

Phoenix's 2027 vendordep is intentionally asymmetric for the new target: hardware-simulation JNI entries include `linuxsystemcore`, while Phoenix software-simulation (`swsim`) binaries remain desktop-only. This is the SystemCore workaround: deployment resolves the available hardware JNI instead of requesting a Phoenix swsim binary that has no SystemCore platform build, while desktop simulation keeps its software-simulation libraries.

## Commands v3 model

Commands v3 schedules coroutine-backed `Command` instances through a `Scheduler`; `Mechanism` instances own requirements, default commands, and periodic callbacks. Robot code calls the default scheduler from `Robot.robotPeriodic()` after the superstructure update and before tunable updates. Mode transitions schedule or cancel commands through that same scheduler.

The migration keeps concise v2-shaped call sites behind local helpers. `Commands` provides no-op, run, wait, composition, repetition, and cleanup factories; `RobotMechanism` provides mechanism-owned one-shot/repeating commands and periodic registration; and `CommandLifecycleAdapter` runs legacy initialize/execute/isFinished/end implementations as a v3 coroutine with exactly-once normal, cancellation, and failure cleanup.
