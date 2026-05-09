package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;

/** Small helpers for Phoenix6 calls — modeled on the AKit template's {@code PhoenixUtil}. */
public final class PhoenixUtil {

  /**
   * Synthetic per-sub-tick timestamps that line up with MapleSim's internal integration steps. Sim
   * IO classes return one odometry sample per sub-tick, so the timestamp array length must match
   * the cached-positions array length read from the simulation.
   */
  public static double[] getSimulationOdometryTimeStamps() {
    int subTicks = SimulatedArena.getSimulationSubTicksIn1Period();
    double now = Timer.getFPGATimestamp();
    double dt = SimulatedArena.getSimulationDt().in(Seconds);
    double[] timestamps = new double[subTicks];
    for (int i = 0; i < subTicks; i++) {
      timestamps[i] = now - Constants.LOOP_PERIOD_SECONDS + i * dt;
    }
    return timestamps;
  }

  /**
   * Retry a Phoenix6 call up to {@code maxAttempts} times until it returns {@link StatusCode#OK}.
   * If every attempt fails, the final {@link StatusCode} is surfaced via {@link
   * DriverStation#reportError} so the failure shows up in the DS console / AKit alerts instead of
   * silently booting with default gains.
   */
  public static void tryUntilOk(String description, int maxAttempts, Supplier<StatusCode> command) {
    StatusCode lastStatus = StatusCode.OK;
    for (int i = 0; i < maxAttempts; i++) {
      lastStatus = command.get();
      if (lastStatus.isOK()) return;
    }
    DriverStation.reportError(
        "[PhoenixUtil] "
            + description
            + " failed after "
            + maxAttempts
            + " attempts: "
            + lastStatus,
        false);
  }

  private PhoenixUtil() {}
}
