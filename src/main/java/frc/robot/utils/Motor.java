package frc.robot.utils;

/**
 * How much torque each motor can produce at different speeds (from dyno test data).
 *
 * <p>Motors produce less torque the faster they spin. The relationship is roughly a straight line:
 * {@code torque = stallTorque * (1 - rpm / freeSpeedRpm)}.
 */
public enum Motor {
  /** Kraken X60 in FOC mode. More torque, slightly lower top speed than non-FOC. */
  KRAKEN_X60_FOC(9.3615, 5784.65, 476.1),

  /** Kraken X60 without FOC. Less torque, slightly higher top speed than FOC. */
  KRAKEN_X60(7.1573, 6065.33, 374.4);

  /** rad/s to RPM. */
  private static final double RAD_PER_SEC_TO_RPM = 60.0 / (2.0 * Math.PI);

  private final double stallTorque; // N-m when not moving
  private final double freeSpeedRpm; // RPM at no load
  private final double stallCurrent; // Amps when not moving
  private final double kt; // N-m per Amp

  Motor(double stallTorque, double freeSpeedRpm, double stallCurrent) {
    this.stallTorque = stallTorque;
    this.freeSpeedRpm = freeSpeedRpm;
    this.stallCurrent = stallCurrent;
    this.kt = stallTorque / stallCurrent;
  }

  /** Available torque at this motor speed. Returns 0 at or above free speed. */
  public double getTorqueAtRpm(double rpm) {
    rpm = Math.abs(rpm);
    if (rpm >= freeSpeedRpm) {
      return 0.0;
    }
    return stallTorque * (1.0 - rpm / freeSpeedRpm);
  }

  /**
   * Available torque at this speed, but capped by the current limit. Real motors can't make more
   * torque than {@code Kt * currentLimit} no matter what the curve says.
   */
  public double getTorqueAtRpm(double rpm, double currentLimitAmps) {
    double torqueFromCurve = getTorqueAtRpm(rpm);
    double torqueFromCurrentLimit = kt * currentLimitAmps;
    return Math.min(torqueFromCurve, torqueFromCurrentLimit);
  }

  public double getStallTorque() {
    return stallTorque;
  }

  public double getFreeSpeedRpm() {
    return freeSpeedRpm;
  }

  public double getStallCurrent() {
    return stallCurrent;
  }

  public double getKt() {
    return kt;
  }

  /**
   * Max acceleration the robot can produce at the current wheel speed, given the motor curve and
   * the current limit.
   *
   * @param wheelSpeedMps Current wheel speed (m/s)
   * @param gearRatio Motor rotations per wheel rotation
   * @param wheelRadiusMeters Wheel radius (m)
   * @param robotMassKg Robot mass (kg)
   * @param numDriveMotors Number of drive motors
   * @param currentLimitAmps Current limit per motor (A)
   * @return Max acceleration (m/s²)
   */
  public double getMaxAcceleration(
      double wheelSpeedMps,
      double gearRatio,
      double wheelRadiusMeters,
      double robotMassKg,
      int numDriveMotors,
      double currentLimitAmps) {

    // Convert wheel speed to motor RPM.
    double wheelRadPerSec = wheelSpeedMps / wheelRadiusMeters;
    double motorRadPerSec = wheelRadPerSec * gearRatio;
    double motorRpm = motorRadPerSec * RAD_PER_SEC_TO_RPM;

    // Torque the motor can produce at this RPM, capped by the current limit.
    double motorTorque = getTorqueAtRpm(motorRpm, currentLimitAmps);

    // Convert torque to force: gearbox multiplies torque, then divide by wheel radius.
    double wheelTorque = motorTorque * gearRatio;
    double forcePerWheel = wheelTorque / wheelRadiusMeters;
    double totalForce = forcePerWheel * numDriveMotors;

    // F = ma, so a = F / m.
    return totalForce / robotMassKg;
  }
}
