# Project diary

## WPILib 2027 alpha-6 / Commands v3 migration

The project moved from the 2026 toolchain to Java 25, Gradle 9.4.1, and GradleRIO `2027.0.0-alpha-6`. SystemCore is the deployment target: deployable Java and static files use the SystemCore GradleRIO target and `/home/systemcore/{deploy,logs}` paths. Desktop support was deliberately retained so the existing native simulation GUI and Driver Station workflow remain available during the hardware-target migration.

Commands v3 was adopted as the scheduling model rather than preserving a parallel Commands v2 dependency. Its coroutine scheduler and mechanism-owned requirements are integrated at the robot lifecycle boundary (`Robot.robotPeriodic`) and in container/subsystem defaults. Three small local compatibility surfaces keep migration risk bounded: `Commands` preserves common factory/composition call shapes, `RobotMechanism` centralizes mechanism-owned one-shot/repeating work, and `CommandLifecycleAdapter` preserves legacy command lifecycle implementations while guaranteeing end cleanup on normal completion, cancellation, or failure.

The continuation runtime needs JVM access to `jdk.internal.vm` and `java.lang`; the two `--add-opens` flags are therefore applied consistently to deployed Java, tests, and JavaExec tasks. This is a runtime compatibility requirement, not a production command behavior change.

Phoenix 6 `26.50.0-alpha-1` introduced a platform mismatch for the new target: its software-simulation libraries are published for desktop platforms, not Linux SystemCore. The vendordep keeps `swsim` desktop-only and adds `linuxsystemcore` only to the hardware-simulation JNI entries. That narrow workaround lets SystemCore resolve the available Phoenix native libraries without disabling desktop simulation.

The migration gate was a clean full `./gradlew build --no-daemon` with 11/11 tests passing and no remaining Commands v2 residue. The durable implementation surface is now the v3 scheduler/mechanism model plus the three helpers above.
