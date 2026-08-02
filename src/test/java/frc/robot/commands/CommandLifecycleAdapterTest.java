package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

class CommandLifecycleAdapterTest {
  private Scheduler scheduler;
  private Mechanism mechanism;

  @BeforeEach
  void setUp() {
    scheduler = Scheduler.createIndependentScheduler();
    mechanism = new Mechanism("test", scheduler);
  }

  @AfterEach
  void tearDown() {
    scheduler.cancelAll();
  }

  @Test
  void naturalCompletionRunsLegacyLifecycleInOrder() {
    TestCommand command = new TestCommand(mechanism, 2);

    scheduler.schedule(command);
    scheduler.run();
    assertEquals(1, command.initializeCalls);
    assertEquals(1, command.executeCalls);
    assertEquals(0, command.normalEndCalls);
    assertTrue(scheduler.isRunning(command));

    scheduler.run();
    scheduler.run();
    assertEquals(2, command.executeCalls);
    assertEquals(1, command.normalEndCalls);
    assertEquals(0, command.interruptedEndCalls);
    assertFalse(scheduler.isScheduledOrRunning(command));
    assertTrue(command.requires(mechanism));
    assertEquals("TestCommand", command.name());
  }

  @Test
  void initiallyFinishedCommandStillExecutesOnceBeforeNormalEnd() {
    TestCommand command = new TestCommand(mechanism, 0);

    scheduler.schedule(command);
    scheduler.run();

    assertEquals(1, command.initializeCalls);
    assertEquals(1, command.executeCalls);
    assertEquals(1, command.normalEndCalls);
    assertEquals(0, command.interruptedEndCalls);
    assertFalse(scheduler.isScheduledOrRunning(command));
  }

  @Test
  void cancellationAndFailureCleanUpExactlyOnce() {
    TestCommand canceled = new TestCommand(mechanism, Integer.MAX_VALUE);
    scheduler.schedule(canceled);
    scheduler.run();
    scheduler.cancel(canceled);
    scheduler.cancel(canceled);

    assertEquals(1, canceled.initializeCalls);
    assertEquals(1, canceled.interruptedEndCalls);
    assertEquals(0, canceled.normalEndCalls);

    TestCommand failed = new TestCommand(mechanism, Integer.MAX_VALUE);
    failed.throwDuringExecute = true;
    scheduler.schedule(failed);
    assertThrows(IllegalStateException.class, scheduler::run);

    assertEquals(1, failed.initializeCalls);
    assertEquals(1, failed.interruptedEndCalls);
    assertFalse(scheduler.isScheduledOrRunning(failed));
  }

  @Test
  void sameCommandInstanceCanBeScheduledAgainWithFreshLifecycle() {
    TestCommand command = new TestCommand(mechanism, 1);

    runToCompletion(command);
    runToCompletion(command);

    assertEquals(2, command.initializeCalls);
    assertEquals(2, command.executeCalls);
    assertEquals(2, command.normalEndCalls);
    assertEquals(0, command.interruptedEndCalls);
  }

  private void runToCompletion(TestCommand command) {
    scheduler.schedule(command);
    scheduler.run();
    scheduler.run();
    assertFalse(scheduler.isScheduledOrRunning(command));
  }

  private static final class TestCommand extends CommandLifecycleAdapter {
    private final int executionsToFinish;
    private int executionsThisRun;
    private int initializeCalls;
    private int executeCalls;
    private int normalEndCalls;
    private int interruptedEndCalls;
    private boolean throwDuringExecute;

    TestCommand(Mechanism mechanism, int executionsToFinish) {
      super(mechanism);
      this.executionsToFinish = executionsToFinish;
    }

    @Override
    protected void initialize() {
      initializeCalls++;
      executionsThisRun = 0;
    }

    @Override
    protected void execute() {
      executeCalls++;
      executionsThisRun++;
      if (throwDuringExecute) {
        throw new IllegalStateException("test failure");
      }
    }

    @Override
    protected void end(boolean interrupted) {
      if (interrupted) {
        interruptedEndCalls++;
      } else {
        normalEndCalls++;
      }
    }

    @Override
    protected boolean isFinished() {
      return executionsThisRun >= executionsToFinish;
    }
  }
}
