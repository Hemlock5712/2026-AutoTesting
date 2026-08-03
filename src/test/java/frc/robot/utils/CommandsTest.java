package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

class CommandsTest {
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
  void runFactoriesOwnRequirementsAndFollowSchedulerCycles() {
    Mechanism mechanism = new Mechanism("test", scheduler);
    AtomicInteger oneShotCalls = new AtomicInteger();
    AtomicInteger periodicCalls = new AtomicInteger();

    Command oneShot = Commands.runOnce(oneShotCalls::incrementAndGet, mechanism);
    assertEquals("RunOnce", oneShot.name());
    assertEquals(1, oneShot.requirements().size());
    assertTrue(oneShot.requires(mechanism));

    scheduler.schedule(oneShot);
    scheduler.run();
    scheduler.run();

    assertEquals(1, oneShotCalls.get());
    assertFalse(scheduler.isScheduledOrRunning(oneShot));

    Command periodic = Commands.run(periodicCalls::incrementAndGet, mechanism);
    assertEquals("Run", periodic.name());
    assertTrue(periodic.requires(mechanism));

    scheduler.schedule(periodic);
    scheduler.run();
    scheduler.run();
    scheduler.run();

    assertEquals(3, periodicCalls.get());
    assertTrue(scheduler.isRunning(periodic));

    scheduler.cancel(periodic);
    scheduler.run();
    assertEquals(3, periodicCalls.get());
    assertFalse(scheduler.isScheduledOrRunning(periodic));
  }

  @Test
  void sequenceParallelEitherAndDeadlinePreserveCompositionSemantics() {
    List<String> events = new ArrayList<>();

    Command sequence =
        Commands.sequence(
            Commands.runOnce(() -> events.add("first")),
            Commands.runOnce(() -> events.add("second")));
    scheduler.schedule(sequence);
    scheduler.run();
    assertEquals(List.of("first", "second"), events);
    assertFalse(scheduler.isScheduledOrRunning(sequence));

    AtomicBoolean condition = new AtomicBoolean(true);
    Command either =
        Commands.either(
            Commands.runOnce(() -> events.add("true")),
            Commands.runOnce(() -> events.add("false")),
            condition::get);
    scheduler.schedule(either);
    scheduler.run();
    assertEquals(List.of("first", "second", "true"), events);

    Command parallel =
        Commands.parallel(
            Commands.runOnce(() -> events.add("left")),
            Commands.runOnce(() -> events.add("right")));
    scheduler.schedule(parallel);
    scheduler.run();
    assertTrue(events.containsAll(List.of("left", "right")));
    assertFalse(scheduler.isScheduledOrRunning(parallel));

    AtomicInteger alongsideRuns = new AtomicInteger();
    AtomicInteger alongsideCancellations = new AtomicInteger();
    Command deadline = Command.noRequirements(coroutine -> coroutine.yield()).named("deadline");
    Command alongside =
        Command.noRequirements(
                coroutine -> {
                  while (true) {
                    alongsideRuns.incrementAndGet();
                    coroutine.yield();
                  }
                })
            .whenCanceled(alongsideCancellations::incrementAndGet)
            .named("alongside");
    Command deadlineGroup = Commands.deadline(deadline, alongside);

    scheduler.schedule(deadlineGroup);
    scheduler.run();
    assertTrue(scheduler.isRunning(deadlineGroup));
    assertEquals(1, alongsideRuns.get());

    scheduler.run();
    assertFalse(scheduler.isScheduledOrRunning(deadlineGroup));
    assertEquals(1, alongsideCancellations.get());
  }

  @Test
  void repeatedlyYieldsBetweenOneShotIterations() {
    AtomicInteger totalExecutions = new AtomicInteger();
    AtomicInteger executionsThisCycle = new AtomicInteger();
    Command oneShot =
        Commands.runOnce(
            () -> {
              totalExecutions.incrementAndGet();
              if (executionsThisCycle.incrementAndGet() > 1) {
                throw new IllegalStateException("one-shot repeated within one scheduler cycle");
              }
            });
    Command repeated = Commands.repeatedly(oneShot);
    scheduler.schedule(repeated);

    for (int cycle = 1; cycle <= 3; cycle++) {
      executionsThisCycle.set(0);
      assertDoesNotThrow(scheduler::run);
      assertEquals(cycle, totalExecutions.get());
      assertTrue(scheduler.isRunning(repeated));
    }
  }

  @Test
  void finallyDoCleansUpExactlyOnceOnCompletionAndCancellation() {
    AtomicInteger naturalBodyCalls = new AtomicInteger();
    AtomicInteger naturalCleanupCalls = new AtomicInteger();
    Command natural =
        Commands.finallyDo(
            Commands.runOnce(naturalBodyCalls::incrementAndGet),
            naturalCleanupCalls::incrementAndGet);

    scheduler.schedule(natural);
    scheduler.run();
    assertEquals(1, naturalBodyCalls.get());
    assertEquals(1, naturalCleanupCalls.get());
    assertFalse(scheduler.isScheduledOrRunning(natural));

    AtomicInteger canceledCleanupCalls = new AtomicInteger();
    Command canceled =
        Commands.finallyDo(Commands.run(() -> {}), canceledCleanupCalls::incrementAndGet);
    scheduler.schedule(canceled);
    scheduler.run();
    scheduler.cancel(canceled);
    scheduler.cancel(canceled);

    assertEquals(1, canceledCleanupCalls.get());
    assertFalse(scheduler.isScheduledOrRunning(canceled));
  }
}
