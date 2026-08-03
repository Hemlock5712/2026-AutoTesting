# Project structure

## Runtime and scheduling ownership

- `src/main/java/frc/robot/Robot.java` owns robot lifecycle entry points, calls `Scheduler.getDefault().run()` from `robotPeriodic()`, and schedules or cancels mode commands.
- `src/main/java/frc/robot/RobotContainer.java` constructs mechanisms, binds `CommandGamepad` triggers, installs the drivetrain default command, and builds autonomous command suppliers.
- `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java` exposes the Commands v3 `Mechanism` used for drivetrain requirements/defaults, registers periodic work, and retains its faster simulation notifier.

## Native coroutine commands

The direct Commands v3 implementations under `src/main/java/frc/robot/commands/` are:

- Manual drivetrain control: `AxisLockDrive`, `OrbitDrive`, and `TurretDrive`.
- Finite autonomous motion: `DriveToPoint` and `FollowPath`.
- Game-piece control: `GamePieceDrive` and `JamProtectedShoot`.

These classes own their native `run(Coroutine)` flow, explicit name and requirement set, per-schedule state reset, scheduler yields, and cancellation/error cleanup. The finite motion commands also own their completion predicates and natural cleanup.

## Shared Commands v3 helpers

- `src/main/java/frc/robot/utils/Commands.java` wraps concise v3 coroutine factories for one-shot/repeating work, waits, parallel/sequence/either/deadline compositions, repetition, `until`, and cleanup.
- `src/main/java/frc/robot/utils/RobotMechanism.java` is the common hardware mechanism base and supplies periodic registration plus mechanism-owned run helpers.

## Tests

- `src/test/java/frc/robot/commands/NativeCommandArchitectureTest.java` enforces direct `Command` implementation, declared v3 methods, one yielding loop per converted class, and package-wide absence of lifecycle bridge residue.
- `src/test/java/frc/robot/autonomous/AutoCommandsTest.java` covers path-action composition and child cleanup.
- `src/test/java/frc/robot/utils/CommandsTest.java` and `RobotMechanismTest.java` cover shared coroutine compositions and mechanism ownership.

The verified full build reports 9/9 tests passing.
