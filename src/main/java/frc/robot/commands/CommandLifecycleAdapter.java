package frc.robot.commands;

import java.util.Set;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;

/** Adapts the legacy initialize/execute/isFinished shape to a Commands v3 coroutine. */
abstract class CommandLifecycleAdapter implements Command {
  private final Set<Mechanism> requirements;
  private final String name;
  private boolean initialized;
  private boolean ended;

  protected CommandLifecycleAdapter(Mechanism requirement) {
    requirements = Set.of(requirement);
    name = getClass().getSimpleName();
  }

  protected abstract void initialize();

  protected abstract void execute();

  protected abstract void end(boolean interrupted);

  protected abstract boolean isFinished();

  @Override
  public final void run(Coroutine coroutine) {
    initialized = false;
    ended = false;
    initialize();
    initialized = true;
    try {
      while (true) {
        execute();
        if (isFinished()) {
          finish(false);
          return;
        }
        coroutine.yield();
      }
    } catch (RuntimeException | Error ex) {
      // Scheduler cancellation invokes onCancel; preserve cleanup for command failures too.
      finish(true);
      throw ex;
    }
  }

  @Override
  public final void onCancel() {
    finish(true);
  }

  private void finish(boolean interrupted) {
    if (initialized && !ended) {
      ended = true;
      end(interrupted);
    }
  }

  @Override
  public final String name() {
    return name;
  }

  @Override
  public final Set<Mechanism> requirements() {
    return requirements;
  }
}
