package frc.robot.autonomous;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AutoCommandsTest {
  @BeforeEach
  void setUp() {
    TrackingCommand.lastExecutingName.set(null);
  }

  @AfterEach
  void tearDown() {
    TrackingCommand.lastExecutingName.set(null);
  }

  @Test
  void triggeredActionStaysScheduledUntilReplaced() {
    TestProgress progress = new TestProgress();
    TrackingCommand hubShoot = new TrackingCommand("hub");
    TrackingCommand stopShoot = new TrackingCommand("stop");

    AutoCommands.ActivePathActionRunner runner =
        new AutoCommands.ActivePathActionRunner(
            List.of(
                new AutoCommands.ScheduledPathAction(1.0, () -> hubShoot),
                new AutoCommands.ScheduledPathAction(3.0, () -> stopShoot)),
            progress::get);

    progress.set(1.0);
    runner.execute();
    runner.execute();

    assertEquals(1, hubShoot.initializeCount);
    assertEquals(2, hubShoot.executeCount);
    assertEquals("hub", TrackingCommand.lastExecutingName.get());

    progress.set(2.0);
    runner.execute();

    assertEquals(3, hubShoot.executeCount);
    assertEquals(1, hubShoot.initializeCount);
    assertEquals(0, hubShoot.endCount);

    progress.set(3.0);
    runner.execute();
    runner.execute();

    assertEquals(1, hubShoot.endCount);
    assertTrue(hubShoot.wasInterrupted);
    assertEquals(1, stopShoot.initializeCount);
    assertEquals(2, stopShoot.executeCount);
    assertEquals("stop", TrackingCommand.lastExecutingName.get());
  }

  @Test
  void sameTriggerActionsRunInParallel() {
    TestProgress progress = new TestProgress();
    TrackingCommand hubShoot = new TrackingCommand("hub");
    TrackingCommand intake = new TrackingCommand("intake");

    AutoCommands autoCommands = new AutoCommands(null);
    AutoCommands.ActivePathActionRunner runner =
        new AutoCommands.ActivePathActionRunner(
            autoCommands.groupScheduledActions(
                List.of(
                    new AutoCommands.ScheduledPathAction(1.0, () -> hubShoot),
                    new AutoCommands.ScheduledPathAction(1.0, () -> intake))),
            progress::get);

    progress.set(1.0);
    runner.execute();
    runner.execute();

    assertEquals(1, hubShoot.initializeCount);
    assertEquals(1, intake.initializeCount);
    assertEquals(2, hubShoot.executeCount);
    assertEquals(2, intake.executeCount);
    assertEquals(0, hubShoot.endCount);
    assertEquals(0, intake.endCount);
  }

  @Test
  void endingRunnerCancelsActiveAction() {
    TestProgress progress = new TestProgress();
    TrackingCommand hubShoot = new TrackingCommand("hub");

    AutoCommands.ActivePathActionRunner runner =
        new AutoCommands.ActivePathActionRunner(
            List.of(new AutoCommands.ScheduledPathAction(0.5, () -> hubShoot)), progress::get);

    progress.set(1.0);
    runner.execute();
    runner.execute();

    assertEquals(1, hubShoot.initializeCount);
    assertEquals(2, hubShoot.executeCount);

    runner.end(true);
    assertTrue(hubShoot.wasInterrupted);
  }

  @Test
  void laterTriggerReplacesEarlierActionGroup() {
    TestProgress progress = new TestProgress();
    TrackingCommand hubShoot = new TrackingCommand("hub");
    TrackingCommand intake = new TrackingCommand("intake");
    TrackingCommand stopShoot = new TrackingCommand("stop");

    AutoCommands autoCommands = new AutoCommands(null);
    AutoCommands.ActivePathActionRunner runner =
        new AutoCommands.ActivePathActionRunner(
            autoCommands.groupScheduledActions(
                List.of(
                    new AutoCommands.ScheduledPathAction(1.0, () -> hubShoot),
                    new AutoCommands.ScheduledPathAction(1.0, () -> intake),
                    new AutoCommands.ScheduledPathAction(3.0, () -> stopShoot))),
            progress::get);

    progress.set(1.0);
    runner.execute();
    runner.execute();

    progress.set(3.0);
    runner.execute();
    runner.execute();

    assertEquals(1, hubShoot.endCount);
    assertEquals(1, intake.endCount);
    assertTrue(hubShoot.wasInterrupted);
    assertTrue(intake.wasInterrupted);
    assertEquals(1, stopShoot.initializeCount);
    assertEquals(2, stopShoot.executeCount);
  }

  private static final class TestProgress {
    private double value;

    double get() {
      return value;
    }

    void set(double value) {
      this.value = value;
    }
  }

  private static final class TrackingCommand extends Command {
    private static final AtomicReference<String> lastExecutingName = new AtomicReference<>();

    private final String name;
    private int initializeCount;
    private int executeCount;
    private int endCount;
    private boolean wasInterrupted;

    TrackingCommand(String name) {
      this.name = name;
    }

    @Override
    public void initialize() {
      initializeCount++;
    }

    @Override
    public void execute() {
      executeCount++;
      lastExecutingName.set(name);
    }

    @Override
    public void end(boolean interrupted) {
      endCount++;
      wasInterrupted = interrupted;
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }
}
