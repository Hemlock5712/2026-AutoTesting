# Project diary

## WPILib 2027 alpha-6 / Commands v3 migration

The project moved to Java 25, Gradle 9.4.1, GradleRIO `2027.0.0-alpha-6`, and SystemCore deployment. Deployable Java, static files, and logs use the SystemCore target and `/home/systemcore/{deploy,logs}` paths. Desktop simulation remains enabled.

Commands v3 is the sole scheduling model. The scheduler and mechanism-owned requirements integrate at the robot lifecycle boundary, controller bindings, subsystem defaults, and autonomous compositions. Local `Commands` and `RobotMechanism` helpers keep common factory and mechanism patterns concise.

The initial migration used a lifecycle bridge to preserve v2 command shapes while establishing a compiling alpha-6 baseline. It was deliberately removed after verification. All seven stateful command classes now express setup, control loops, completion, yields, and cleanup directly in their coroutine bodies. This makes rescheduling state and cancellation behavior visible in the owning class and avoids shared hidden lifecycle state.

Failure cleanup catches `RuntimeException`, stops the affected hardware, and rethrows. It intentionally does not catch `Error`: alpha-6 removes failed commands only for `RuntimeException`, so catching and rethrowing an `Error` could leave a command registered and allow later cancellation to repeat cleanup.

The continuation runtime needs JVM access to `jdk.internal.vm` and `java.lang`; the two `--add-opens` flags apply consistently to deployed Java, tests, and simulation tasks.

Phoenix 6 `26.50.0-alpha-1` advertises unavailable SystemCore software-simulation JNI artifacts. The vendordep keeps those entries desktop-only while retaining the real SystemCore hardware JNI entries.

The native conversion gate is a focused architecture test plus a clean Java 25 `./gradlew build --no-daemon`; the verified result is 9/9 tests passing with no production lifecycle-bridge residue.
