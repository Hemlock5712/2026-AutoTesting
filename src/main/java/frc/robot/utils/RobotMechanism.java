package frc.robot.utils;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

/** Shared v3 mechanism base with the small one-shot/repeating helpers used by robot code. */
public abstract class RobotMechanism extends Mechanism {
  protected RobotMechanism() {
    super();
  }

  protected RobotMechanism(String name) {
    super(name);
  }

  protected final void addPeriodicCallback(Runnable callback) {
    Scheduler.getDefault().addPeriodic(callback);
  }

  /** Runs a body once while owning this mechanism. */
  public Command runOnce(Runnable body) {
    return run(coroutine -> body.run()).named(getName() + "/RunOnce");
  }

  /** Runs a body every scheduler cycle while owning this mechanism. */
  public Command runPeriodic(Runnable body) {
    return run(coroutine -> {
          while (true) {
            body.run();
            coroutine.yield();
          }
        })
        .named(getName() + "/Run");
  }

  /** v2-style overload retained for concise mechanism-owned periodic commands. */
  public Command run(Runnable body) {
    return runPeriodic(body);
  }
}
