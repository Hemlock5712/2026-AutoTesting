package frc.robot.utils;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Time;

/** Small migration helpers for the v2 factory call sites used by this robot. */
public final class Commands {
  private Commands() {}

  public static Command none() {
    return Command.noRequirements(c -> {}).named("None");
  }

  public static Command runOnce(Runnable body, Mechanism... requirements) {
    return Command.requiring(Arrays.asList(requirements))
        .executing(c -> body.run())
        .named("RunOnce");
  }

  public static Command run(Runnable body, Mechanism... requirements) {
    return Command.requiring(Arrays.asList(requirements))
        .executing(
            c -> {
              while (true) {
                body.run();
                c.yield();
              }
            })
        .named("Run");
  }

  public static Command waitSeconds(double seconds) {
    return Command.waitFor(Units.Seconds.of(seconds)).named("Wait " + seconds + "s");
  }

  public static Command waitFor(Time duration) {
    return Command.waitFor(duration).named("Wait");
  }

  public static Command waitUntil(BooleanSupplier condition) {
    return Command.noRequirements(
            c -> {
              while (!condition.getAsBoolean()) {
                c.yield();
              }
            })
        .named("WaitUntil");
  }

  public static Command parallel(Command... commands) {
    return Command.parallel(commands).withAutomaticName();
  }

  public static Command sequence(Command... commands) {
    return Command.sequence(commands).withAutomaticName();
  }

  public static Command either(Command onTrue, Command onFalse, BooleanSupplier condition) {
    return Command.noRequirements(c -> c.await(condition.getAsBoolean() ? onTrue : onFalse))
        .named("Either");
  }

  public static Command deadline(Command deadline, Command... alongside) {
    return Command.parallel(deadline).optional(alongside).withAutomaticName();
  }

  public static Command repeatedly(Command command) {
    return Command.noRequirements(
            c -> {
              while (true) {
                c.await(command);
                c.yield();
              }
            })
        .named(command.name() + " [repeated]");
  }

  public static Command until(Command command, BooleanSupplier condition) {
    return Command.race(command, waitUntil(condition)).withAutomaticName();
  }

  public static Command finallyDo(Command command, Runnable cleanup) {
    return Command.noRequirements(
            coroutine -> {
              coroutine.await(command);
              cleanup.run();
            })
        .whenCanceled(cleanup)
        .named(command.name() + " [cleanup]");
  }
}
