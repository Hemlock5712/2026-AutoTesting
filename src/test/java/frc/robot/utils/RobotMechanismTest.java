package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;

class RobotMechanismTest {
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
  void ownedHelpersUseMechanismNameAndExpectedLifetime() {
    TestMechanism mechanism = new TestMechanism();
    AtomicInteger oneShotCalls = new AtomicInteger();
    AtomicInteger periodicCalls = new AtomicInteger();

    Command oneShot = mechanism.runOnce(oneShotCalls::incrementAndGet);
    assertEquals("TestMechanism/RunOnce", oneShot.name());
    assertTrue(oneShot.requires(mechanism));
    scheduler.schedule(oneShot);
    scheduler.run();
    assertEquals(1, oneShotCalls.get());
    assertFalse(scheduler.isScheduledOrRunning(oneShot));

    Command periodic = mechanism.runPeriodic(periodicCalls::incrementAndGet);
    assertEquals("TestMechanism/Run", periodic.name());
    assertTrue(periodic.requires(mechanism));
    scheduler.schedule(periodic);
    scheduler.run();
    scheduler.run();
    assertEquals(2, periodicCalls.get());
    assertTrue(scheduler.isRunning(periodic));

    scheduler.cancel(periodic);
    scheduler.run();
    assertEquals(2, periodicCalls.get());
  }

  private static final class TestMechanism extends RobotMechanism {
    TestMechanism() {
      super("TestMechanism");
    }
  }
}
