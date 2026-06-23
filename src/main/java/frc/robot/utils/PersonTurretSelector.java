package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.turret.DualEncoderCRT;
import frc.robot.utils.LimelightHelpers.RawDetection;
import java.util.Random;
import org.littletonrobotics.junction.Logger;

/**
 * Selects a person detection from limelight-mm and converts its horizontal offset into a turret
 * angle. Holds the last commanded angle when no people are visible; randomly reselects among
 * multiple people every few seconds.
 */
public class PersonTurretSelector {

  /** Camera boresight yaw in turret mechanism rotations (0.5 = rear-facing). */
  public static final double CAMERA_YAW_OFFSET_ROT = 0.5;

  /**
   * Sign applied to Limelight tx (degrees) when mapping to turret rotations. Flip to -1 if the
   * turret turns the wrong way during bring-up.
   */
  public static final double TX_SIGN = -1.0;

  /** Seconds between random reselection when multiple people are visible. */
  public static final double RESELECT_PERIOD_SECONDS = 3.0;

  /**
   * Assumed ground distance (meters) for projecting field positions from bearing-only detections.
   * Debug visualization only — neural detector does not provide range.
   */
  public static final double ESTIMATED_PERSON_DISTANCE_M = 3.0;

  private final Random random = new Random();
  private double lastReselectTimeSec = 0;
  private int selectedIndex = 0;
  private double lastHeldAngleRot = Double.NaN;

  /**
   * Logs per-detection angles and estimated field positions for AdvantageScope debugging.
   *
   * @param people Person detections from limelight-mm (may be empty).
   * @param robotPose Current robot pose on the field.
   */
  public void logEstimatedPositions(RawDetection[] people, Pose2d robotPose) {
    int count = people.length;
    double[] txDeg = new double[count];
    double[] tyDeg = new double[count];
    double[] area = new double[count];
    double[] robotRelativeAngleDeg = new double[count];
    Pose2d[] fieldPositions = new Pose2d[count];

    double robotHeadingRad = robotPose.getRotation().getRadians();
    for (int i = 0; i < count; i++) {
      RawDetection person = people[i];
      txDeg[i] = person.txnc;
      tyDeg[i] = person.tync;
      area[i] = person.ta;

      double angleRot = personBearingRot(person);
      robotRelativeAngleDeg[i] = angleRot * 360.0;

      double fieldBearingRad = robotHeadingRad + angleRot * 2.0 * Math.PI;
      Translation2d offset =
          new Translation2d(
              ESTIMATED_PERSON_DISTANCE_M * Math.cos(fieldBearingRad),
              ESTIMATED_PERSON_DISTANCE_M * Math.sin(fieldBearingRad));
      fieldPositions[i] = new Pose2d(robotPose.getTranslation().plus(offset), Rotation2d.kZero);
    }

    Logger.recordOutput("PersonTrack/AllTxDeg", txDeg);
    Logger.recordOutput("PersonTrack/AllTyDeg", tyDeg);
    Logger.recordOutput("PersonTrack/AllArea", area);
    Logger.recordOutput("PersonTrack/RobotRelativeAngleDeg", robotRelativeAngleDeg);
    Logger.recordOutput("PersonTrack/EstimatedFieldPositions", fieldPositions);
  }

  /**
   * Computes the turret angle to aim at a selected person, or holds position when none detected.
   *
   * @param people Person detections from limelight-mm (may be empty).
   * @param currentAngleRot Current turret position in mechanism rotations.
   * @return Turret setpoint in mechanism rotations.
   */
  public double computeTurretAngle(RawDetection[] people, double currentAngleRot) {
    if (people.length == 0) {
      if (Double.isNaN(lastHeldAngleRot)) {
        lastHeldAngleRot = currentAngleRot;
      }
      Logger.recordOutput("PersonTrack/SelectedIndex", -1);
      Logger.recordOutput("PersonTrack/SelectedTxDeg", 0.0);
      Logger.recordOutput("PersonTrack/TargetAngleRot", lastHeldAngleRot);
      Logger.recordOutput("PersonTrack/Holding", true);
      return lastHeldAngleRot;
    }

    Logger.recordOutput("PersonTrack/Holding", false);

    if (people.length == 1) {
      selectedIndex = 0;
    } else {
      double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      if (now - lastReselectTimeSec >= RESELECT_PERIOD_SECONDS || selectedIndex >= people.length) {
        selectedIndex = random.nextInt(people.length);
        lastReselectTimeSec = now;
      }
    }

    RawDetection target = people[selectedIndex];
    double angleRot = personBearingRot(target);

    lastHeldAngleRot = angleRot;
    Logger.recordOutput("PersonTrack/SelectedIndex", selectedIndex);
    Logger.recordOutput("PersonTrack/SelectedTxDeg", target.txnc);
    Logger.recordOutput("PersonTrack/TargetAngleRot", angleRot);
    return angleRot;
  }

  private static double personBearingRot(RawDetection person) {
    return MathUtil.clamp(
        CAMERA_YAW_OFFSET_ROT + TX_SIGN * (person.txnc / 360.0),
        DualEncoderCRT.REVERSE_LIMIT,
        DualEncoderCRT.FORWARD_LIMIT);
  }
}
