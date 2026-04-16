package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class JamProtectedShoot extends Command {

  private enum State {
    NORMAL,
    MONITORING_SIDEWAYS,
    WAITING_FOR_KICKER,
    JAM_RECOVERY
  }

  // Timing constants
  private static final double SIDEWAYS_CONFIRM_TIME = 0.2; // 0.5s to confirm ball in sideways
  private static final double KICKER_TIMEOUT = 0.2; // 0.33s for ball to reach kicker
  private static final double RECOVERY_TIME = 0.1; // 0.5s recovery sequence

  private final Hopper hopper;
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
    this.isReadyToFeed = isReadyToFeed;

    addRequirements(hopper);
  }

  @Override
  public void initialize() {
    state = State.NORMAL;
    sidewaysTimer.stop();
    sidewaysTimer.reset();
    kickerTimer.stop();
    kickerTimer.reset();
    recoveryTimer.stop();
    recoveryTimer.reset();
  }

  @Override
  public void execute() {
    Logger.recordOutput("JamProtection/State", state.name());
    Logger.recordOutput("JamProtection/IsReadyToFeed", isReadyToFeed.getAsBoolean());

    switch (state) {
      case NORMAL:
        executeNormal();
        break;
      case MONITORING_SIDEWAYS:
        executeMonitoringSideways();
        break;
      case WAITING_FOR_KICKER:
        executeWaitingForKicker();
        break;
      case JAM_RECOVERY:
        executeJamRecovery();
        break;
    }
  }

  private void executeNormal() {
    // Normal hopper control: start if ready, stop if not
    if (isReadyToFeed.getAsBoolean()) {
      hopper.setVelocity(RotationsPerSecond.of(34), RotationsPerSecond.of(10));

      // Start monitoring for jams if ball is in sideways
      if (hopper.hasBallInSideways()) {
        state = State.MONITORING_SIDEWAYS;
        sidewaysTimer.restart();
      }
    } else {
      hopper.setVelocity(RotationsPerSecond.of(0), RotationsPerSecond.of(0));
    }
  }

  private void executeMonitoringSideways() {
    // Keep hopper running while monitoring
    hopper.setVelocity(RotationsPerSecond.of(34), RotationsPerSecond.of(10));

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
    hopper.setVelocity(RotationsPerSecond.of(34), RotationsPerSecond.of(10));

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

  @Override
  public void end(boolean interrupted) {
    // Clean up timers
    sidewaysTimer.stop();
    kickerTimer.stop();
    recoveryTimer.stop();

    // Stop hopper
    hopper.setVelocity(RotationsPerSecond.of(0), RotationsPerSecond.of(0));
  }

  @Override
  public boolean isFinished() {
    // This command runs until cancelled externally
    return false;
  }
}
