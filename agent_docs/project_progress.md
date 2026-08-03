# Completed deployment: native Commands v3 coroutines

## Outcome

All seven stateful robot command classes directly implement Commands v3 `Command` and express their behavior through native `run(Coroutine)` control flow. The lifecycle adapter and its adapter-specific tests have been removed.

## Completed phases

| ID | Phase | Owner | Status |
| --- | --- | --- | --- |
| C3-1 | Convert manual drivetrain commands | executor_luna A | complete |
| C3-2 | Convert autonomous motion commands | executor_luna B | complete |
| C3-3 | Convert game-piece commands | executor_luna C | complete |
| C3-4 | Remove the adapter, replace focused tests, and independently review semantics | main + independent verifier | complete |
| C3-5 | Repair Scheduler edge cases, run the full build, review, and reconcile docs | executors + main | complete |

## Verification evidence

- The committed alpha-6 baseline is checkpoint `67e391f` (`Upgrade to WPILib 2027 alpha 6 and Commands v3`).
- Production integration compile passed with the bundled Java 25 runtime.
- Focused `NativeCommandArchitectureTest`: 2/2 tests passed after the final production changes.
- Full `JAVA_HOME=/Users/bacon/wpilib/2027_alpha5/jdk ./gradlew build --no-daemon`: 9/9 tests passed after the final production changes.
- The full gate included dependency prefetch, Spotless, tests, and shaded JAR creation.
- Residue searches find seven direct implementations and no production lifecycle adapter or legacy lifecycle declarations.
- `git diff --check` passes.

## Design result

- Continuous commands perform per-run setup, execute one control iteration per scheduler cycle, and yield explicitly.
- Finite commands evaluate completion after their control iteration and perform natural cleanup before returning.
- Every command declares its name and mechanism requirements directly and uses `onCancel()` for safe interruption.
- Runtime failures stop hardware and rethrow; `Error` is not intercepted because alpha-6 does not remove commands for that path and a later cancellation could otherwise repeat cleanup.
- Reusable commands reset mutable run state at the beginning of each schedule.

## Blockers

None.

## Next action

Perform a SystemCore hardware smoke test before field use. The native coroutine conversion remains uncommitted so it can be reviewed as a separate logical change from checkpoint `67e391f`.
