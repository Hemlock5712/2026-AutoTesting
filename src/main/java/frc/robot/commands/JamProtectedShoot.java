package frc.robot.commands;

import frc.robot.subsystems.hopper.Hopper;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.system.Timer;

public class JamProtectedShoot implements Command {

  private enum State {
    NORMAL,
    MONITORING_SIDEWAYS,
    WAITING_FOR_KICKER,
    JAM_RECOVERY
  }

  // Timing constants
  private static final double SIDEWAYS_CONFIRM_TIME = 0.15; // 0.5s to confirm ball in sideways
  private static final double KICKER_TIMEOUT = 0.15; // 0.33s for ball to reach kicker
  private static final double RECOVERY_TIME = 0.3; // 0.5s recovery sequence

  // Hopper speeds in rotations per second (CTRE native unit)
  private static final double HOPPER_MAIN_RPS = 34;
  private static final double HOPPER_SIDE_RPS = 30;

  // Pre-built state names to avoid String allocation from .name()
  private static final String[] STATE_NAMES =
      java.util.Arrays.stream(State.values()).map(Enum::name).toArray(String[]::new);

  private final Hopper hopper;
  private final Set<Mechanism> requirements;
  private final String name;
  private final BooleanSupplier isReadyToFeed;

  private State state = State.NORMAL;
  private final Timer sidewaysTimer = new Timer();
  private final Timer kickerTimer = new Timer();
  private final Timer recoveryTimer = new Timer();

  /**
   * Creates a jam-protected hopper control command.
   *
   * @param hopper The hopper subsystem with CANrange sensors
   * @param isReadyToFeed Supplier that returns true when ready to feed (isHubReady or isFeedReady)
   */
  public JamProtectedShoot(Hopper hopper, BooleanSupplier isReadyToFeed) {
    this.hopper = hopper;
    this.requirements = Set.of(hopper);
    this.name = getClass().getSimpleName();
    this.isReadyToFeed = isReadyToFeed;
  }

  @Override
  public void run(Coroutine coroutine) {
    state = State.NORMAL;
    sidewaysTimer.stop();
    sidewaysTimer.reset();
    kickerTimer.stop();
    kickerTimer.reset();
    recoveryTimer.stop();
    recoveryTimer.reset();

    try {
      while (true) {
        updateControl();
        coroutine.yield();
      }
    } catch (RuntimeException ex) {
      safeStop();
      throw ex;
    }
  }

  @Override
  public void onCancel() {
    safeStop();
  }

  private void updateControl() {
    Logger.recordOutput("JamProtection/State", STATE_NAMES[state.ordinal()]);
    Logger.recordOutput("JamProtection/IsReadyToFeed", isReadyToFeed.getAsBoolean());

    // switch (state) {
    //   case NORMAL:
    executeNormal();
    //     break;
    //   case MONITORING_SIDEWAYS:
    //     executeMonitoringSideways();
    //     break;
    //   case WAITING_FOR_KICKER:
    //     executeWaitingForKicker();
    //     break;
    //   case JAM_RECOVERY:
    //     executeJamRecovery();
    //     break;
    // }
  }

  private void executeNormal() {
    // Normal hopper control: start if ready, stop if not
    if (isReadyToFeed.getAsBoolean()) {
      hopper.setVelocityRPS(HOPPER_MAIN_RPS, HOPPER_SIDE_RPS);

      // Start monitoring for jams if ball is in sideways
      if (hopper.hasBallInSideways()) {
        state = State.MONITORING_SIDEWAYS;
        sidewaysTimer.restart();
      }
    } else {
      hopper.setVelocityRPS(0, 0);
    }
  }

  private void executeMonitoringSideways() {
    // Keep hopper running while monitoring
    hopper.setVelocityRPS(HOPPER_MAIN_RPS, HOPPER_SIDE_RPS);

    // If ball leaves sideways, go back to normal
    if (!hopper.hasBallInSideways()) {
      state = State.NORMAL;
      sidewaysTimer.stop();
      sidewaysTimer.reset();
      return;
    }

    // If we're no longer ready to feed, go back to normal
    if (!isReadyToFeed.getAsBoolean()) {
      state = State.NORMAL;
      sidewaysTimer.stop();
      sidewaysTimer.reset();
      return;
    }

    // If ball has been in sideways for confirmation time, start monitoring kicker
    if (sidewaysTimer.hasElapsed(SIDEWAYS_CONFIRM_TIME)) {
      state = State.WAITING_FOR_KICKER;
      sidewaysTimer.stop();
      sidewaysTimer.reset();
      kickerTimer.restart();
    }
  }

  private void executeWaitingForKicker() {
    // Keep hopper running while waiting for ball to reach kicker
    hopper.setVelocityRPS(HOPPER_MAIN_RPS, HOPPER_SIDE_RPS);

    // If we're no longer ready to feed, go back to normal
    if (!isReadyToFeed.getAsBoolean()) {
      state = State.NORMAL;
      kickerTimer.stop();
      kickerTimer.reset();
      return;
    }

    // If ball reaches kicker, success - go back to normal
    if (hopper.hasBallInKicker()) {
      state = State.NORMAL;
      kickerTimer.stop();
      kickerTimer.reset();
      return;
    }

    // If kicker timeout exceeded without ball, we have a jam
    if (kickerTimer.hasElapsed(KICKER_TIMEOUT)) {
      state = State.JAM_RECOVERY;
      kickerTimer.stop();
      kickerTimer.reset();
      recoveryTimer.restart();

      Logger.recordOutput("JamProtection/JamDetected", true);
    }
  }

  private void executeJamRecovery() {
    // double[] killRobot = {};
    // killRobot[5] = 0;
    // Run jam recovery: reverse kicker (main), forward sideways (side)
    hopper.setJamRecovery();

    // When recovery time is complete, go back to normal
    if (recoveryTimer.hasElapsed(RECOVERY_TIME)) {
      state = State.NORMAL;
      recoveryTimer.stop();
      recoveryTimer.reset();

      Logger.recordOutput("JamProtection/JamDetected", false);
    }
  }

  private void safeStop() {
    // Clean up timers
    sidewaysTimer.stop();
    kickerTimer.stop();
    recoveryTimer.stop();

    // Stop hopper
    hopper.setVelocityRPS(0, 0);
  }

  @Override
  public String name() {
    return name;
  }

  @Override
  public Set<Mechanism> requirements() {
    return requirements;
  }
}
