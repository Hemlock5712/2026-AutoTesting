package frc.robot.autonomous;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.autonomous.AutoCommands.ScheduledPathAction;
import frc.robot.utils.Commands;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;

class AutoCommandsTest {
  private Scheduler scheduler;

  @BeforeEach
  void setUp() {
    scheduler = Scheduler.createIndependentScheduler();
  }

  @AfterEach
  void tearDown() {
    scheduler.cancelAll();
  }

  @Test
  void equalTriggerActionsAreGroupedLazilyAndRunTogether() {
    AutoCommands autoCommands = new AutoCommands(null);
    AtomicInteger supplierCalls = new AtomicInteger();
    AtomicInteger commandRuns = new AtomicInteger();
    Command later = Commands.runOnce(commandRuns::incrementAndGet);

    Supplier<Command> first =
        () -> {
          supplierCalls.incrementAndGet();
          return Commands.runOnce(commandRuns::incrementAndGet);
        };
    Supplier<Command> second =
        () -> {
          supplierCalls.incrementAndGet();
          return Commands.runOnce(commandRuns::incrementAndGet);
        };

    List<ScheduledPathAction> grouped =
        autoCommands.groupScheduledActions(
            List.of(
                new ScheduledPathAction(0.5, first),
                new ScheduledPathAction(0.5, second),
                new ScheduledPathAction(1.25, () -> later)));

    assertEquals(2, grouped.size());
    assertEquals(0.5, grouped.get(0).triggerS());
    assertEquals(1.25, grouped.get(1).triggerS());
    assertEquals(0, supplierCalls.get());

    Command simultaneous = grouped.get(0).commandSupplier().get();
    assertEquals(2, supplierCalls.get());
    scheduler.schedule(simultaneous);
    scheduler.run();
    assertEquals(2, commandRuns.get());
    assertFalse(scheduler.isScheduledOrRunning(simultaneous));

    assertSame(later, grouped.get(1).commandSupplier().get());
  }

  @Test
  void activeRunnerReplacesPriorActionAndParentCancellationCleansUpChild() {
    AtomicReference<Double> progress = new AtomicReference<>(0.0);
    AtomicInteger firstStarts = new AtomicInteger();
    AtomicInteger firstCancellations = new AtomicInteger();
    AtomicInteger secondStarts = new AtomicInteger();
    AtomicInteger secondCancellations = new AtomicInteger();
    AtomicReference<Command> firstCommand = new AtomicReference<>();
    AtomicReference<Command> secondCommand = new AtomicReference<>();

    Supplier<Command> first =
        () -> {
          Command command = longRunning(firstStarts, firstCancellations, "first");
          firstCommand.set(command);
          return command;
        };
    Supplier<Command> second =
        () -> {
          Command command = longRunning(secondStarts, secondCancellations, "second");
          secondCommand.set(command);
          return command;
        };

    Command runner =
        AutoCommands.activePathActionRunner(
            List.of(new ScheduledPathAction(0.5, first), new ScheduledPathAction(1.5, second)),
            progress::get);
    scheduler.schedule(runner);
    scheduler.run();
    assertEquals(0, firstStarts.get());

    progress.set(0.5);
    scheduler.run();
    assertEquals(1, firstStarts.get());
    assertTrue(scheduler.isRunning(firstCommand.get()));
    assertSame(runner, scheduler.getParentOf(firstCommand.get()));

    progress.set(1.5);
    // The runner is still suspended immediately after forking the first action. One cycle resumes
    // that iteration; the next samples the updated progress and schedules the replacement.
    scheduler.run();
    scheduler.run();
    assertEquals(1, firstCancellations.get());
    assertFalse(scheduler.isScheduledOrRunning(firstCommand.get()));
    assertEquals(1, secondStarts.get());
    assertTrue(scheduler.isRunning(secondCommand.get()));
    assertSame(runner, scheduler.getParentOf(secondCommand.get()));

    scheduler.cancel(runner);
    assertEquals(1, secondCancellations.get());
    assertFalse(scheduler.isScheduledOrRunning(secondCommand.get()));
    assertFalse(scheduler.isScheduledOrRunning(runner));
  }

  private static Command longRunning(
      AtomicInteger starts, AtomicInteger cancellations, String name) {
    return Command.noRequirements(
            coroutine -> {
              starts.incrementAndGet();
              while (true) {
                coroutine.yield();
              }
            })
        .whenCanceled(cancellations::incrementAndGet)
        .named(name);
  }
}
