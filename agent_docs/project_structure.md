# Project structure

## Runtime and scheduling ownership

- `src/main/java/frc/robot/Robot.java` owns robot lifecycle entry points, calls `Scheduler.getDefault().run()` from `robotPeriodic()`, and schedules/cancels autonomous and mode-transition commands.
- `src/main/java/frc/robot/RobotContainer.java` constructs mechanisms, binds `CommandGamepad` triggers, installs the drivetrain default command, and builds autonomous command suppliers.
- `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java` exposes the Commands v3 `Mechanism` used for drivetrain requirements/defaults, registers drivetrain periodic work with the scheduler, and retains its faster simulation notifier.

## Commands v3 compatibility surface

- `src/main/java/frc/robot/utils/Commands.java` is the call-site compatibility layer. It wraps v3 coroutine factories for `none`, one-shot/repeating run, waits, `parallel`/`sequence`/`either`/`deadline`, repetition, `until`, and `finallyDo` cleanup.
- `src/main/java/frc/robot/utils/RobotMechanism.java` is the shared mechanism base. It adds scheduler periodic callbacks and mechanism-owned `runOnce`, `runPeriodic`, and v2-shaped `run` helpers; subsystem classes that own hardware extend it.
- `src/main/java/frc/robot/commands/CommandLifecycleAdapter.java` is the legacy lifecycle bridge. Concrete commands provide initialize/execute/isFinished/end while the adapter supplies v3 requirements, coroutine yielding, cancellation handling, and failure cleanup.

Bindings and autonomous code use the local `Commands` factories, while hardware-owning subsystems use `RobotMechanism` and their `Mechanism` requirement. The scheduler therefore remains the single integration point for default commands, trigger commands, autonomous actions, periodic callbacks, cancellation, and parent/child coroutine composition.

## Tests

Migration-sensitive behavior is covered by:

- `src/test/java/frc/robot/utils/CommandsTest.java`
- `src/test/java/frc/robot/utils/RobotMechanismTest.java`
- `src/test/java/frc/robot/commands/CommandLifecycleAdapterTest.java`
- `src/test/java/frc/robot/autonomous/AutoCommandsTest.java`

The verified full build reports 11/11 tests passing.
