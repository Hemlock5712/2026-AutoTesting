package frc.robot.subsystems.drive;

/**
 * Strategy for turning the current drivetrain state into a setpoint, invoked at 250 Hz by {@link
 * Drive#setControl(SwerveRequest)}.
 *
 * <p>An implementation typically reads {@code drive.getPose()}, {@code drive.getRotation()}, and/or
 * {@code drive.getRobotSpeeds()} (all tear-free volatile snapshots, safe from any thread), holds
 * any per-request state in instance fields (e.g. a profiled PID controller, latched targets), and
 * ends in a call to {@code drive.runVelocity(...)} or {@code drive.stopWithX()}. Per-module slip,
 * torque, and steer-rate limiting live inside {@code Drive.runVelocity}, so requests should pass
 * raw targets rather than ramping them themselves.
 *
 * <h2>Thread-safety contract</h2>
 *
 * {@link #apply} runs on the {@link edu.wpi.first.wpilibj.Notifier} thread that drives the 250 Hz
 * fast loop, not on the main robot loop. Two constraints follow:
 *
 * <ul>
 *   <li><b>Do not call {@code Logger.recordOutput} from {@code apply}</b> — AKit's {@code
 *       StructBuffer}s aren't thread-safe and concurrent writes corrupt the byte buffer. Stash
 *       diagnostic data in volatile fields and let the consuming subsystem log it from its main
 *       {@code periodic}.
 *   <li>Inputs the driver layer mutates (target velocities, target heading, etc.) should be {@code
 *       volatile} or otherwise safely published, since the main loop writes them and the fast loop
 *       reads them.
 * </ul>
 *
 * <h2>Lifecycle</h2>
 *
 * Only one request is active at a time. Subsystem requirements on {@link Drive} provide the
 * mutual-exclusion guarantee — a new command that requires Drive interrupts the current one, whose
 * {@code end} should call {@link Drive#clearControl()} before the new one calls {@link
 * Drive#setControl(SwerveRequest)}.
 */
public interface SwerveRequest {
  /**
   * Compute and apply the next setpoint.
   *
   * @param drive the drivetrain to read state from and command
   * @param dt seconds since the previous {@code apply} call (clamped to one fast-loop period on the
   *     first invocation)
   */
  void apply(Drive drive, double dt);

  /**
   * Called once on the main thread when this request becomes the active controller (i.e. is passed
   * to {@link Drive#setControl(SwerveRequest)} and was not already active). Override to seed
   * per-activation state — e.g. resetting an internal PID controller. Default: no-op.
   */
  default void onActivate(Drive drive) {}
}
