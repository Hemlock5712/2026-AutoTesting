package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

/** Small helpers for Phoenix6 calls — modeled on the AKit template's {@code PhoenixUtil}. */
public final class PhoenixUtil {

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
