# Completed deployment: WPILib 2027 alpha 6 and Commands v3

## Outcome

The project is upgraded to WPILib/GradleRIO `2027.0.0-alpha-6`, Java 25, Gradle 9.4.1, the SystemCore deployment target, and the Java Commands v3 framework. Commands v2 dependencies and production imports have been removed.

## Completed phases

| ID | Phase | Owner | Status |
| --- | --- | --- | --- |
| WP27-1 | Inventory the build, command API surface, and official alpha 6 contracts | main + explorer | complete |
| WP27-2 | Upgrade the toolchain/dependencies and migrate production code | executor_luna | complete |
| WP27-3 | Add deterministic migration tests and run focused/full regression gates | independent verifier | complete |
| WP27-4 | Repair lifecycle, JVM-runtime, and Phoenix vendordep defects | executor_luna + independent verifier | complete |
| WP27-5 | Review integration boundaries and update durable documentation | main + doc-writer | complete |

## Verification evidence

- Focused Commands v3 suite: 11/11 tests passed with the bundled Java 25 runtime.
- Full gate: `JAVA_HOME=/Users/bacon/wpilib/2027_alpha5/jdk ./gradlew build --no-daemon` passed after the final production and test changes.
- The full gate included dependency prefetch, Spotless, tests, and `shadowJar` creation.
- Runtime dependency and source-residue audits found Commands v3 alpha 6 and no Commands v2 dependency or production imports.
- `git diff --check` passed.

## Known constraints

- WPILib and Phoenix are alpha releases. The project carries the required Java continuation module-opening flags for deploy, tests, and desktop simulation.
- Phoenix `26.50.0-alpha-1` advertises unavailable SystemCore software-simulation JNI ZIPs; those `linuxsystemcore` entries are excluded while the two real hardware JNI entries remain enabled.
- Java emits non-blocking restricted-native-access warnings from the current Gradle native platform library.

## Blockers

None.

## Next action

Perform a SystemCore hardware smoke test before field use: deploy, enable each mechanism, exercise controller bindings and cancellation cleanup, and confirm `/home/systemcore/logs` output.
