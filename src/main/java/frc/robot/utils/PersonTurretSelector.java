package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.shooter.ShooterLookup;
import frc.robot.subsystems.turret.DualEncoderCRT;
import frc.robot.utils.LimelightHelpers.RawDetection;
import java.util.Arrays;
import java.util.Comparator;
import org.littletonrobotics.junction.Logger;

/**
 * Selects a person detection from limelight-mm and converts its horizontal offset into a turret
 * angle. Holds the last commanded angle when no people are visible. Multiple people are sorted
 * left-to-right so a stable selected slot keeps targeting the same ordered person.
 */
public class PersonTurretSelector {

  /** Camera boresight yaw in turret mechanism rotations (0.5 = rear-facing). */
  public static final double CAMERA_YAW_OFFSET_ROT = 0.5;

  /**
   * Sign applied to Limelight tx (degrees) when mapping to turret rotations. Flip to -1 if the
   * turret turns the wrong way during bring-up.
   */
  public static final double TX_SIGN = -1.0;

  /**
   * Assumed ground distance (meters) for projecting field positions from bearing-only detections.
   * Used as a fallback when vertical-angle range is not geometrically valid.
   */
  public static final double ESTIMATED_PERSON_DISTANCE_M = 3.0;

  /** Approximate limelight-mm lens height from carpet. Calibrate on the real robot. */
  public static final double CAMERA_HEIGHT_M = 0.532;

  /** Approximate vertical point to pass to on a standing person. Calibrate for demo behavior. */
  public static final double PERSON_TARGET_HEIGHT_M = 1.0;

  /** Camera pitch above horizontal in degrees. Positive means the camera points upward. */
  public static final double CAMERA_PITCH_DEG = 0.0;

  public static final double MIN_PERSON_DISTANCE_M = 0.5;
  public static final double MAX_PERSON_DISTANCE_M = 6.5;
  private static final double MAX_TRACK_DT_SECONDS = 0.25;
  private static final double VELOCITY_FILTER_ALPHA = 0.35;
  private static final double MAX_LEAD_SECONDS = 1.25;

  private int selectedIndex = 0;
  private double lastHeldAngleRot = Double.NaN;
  private double lastHeldDistanceM = ESTIMATED_PERSON_DISTANCE_M;
  private TrackState[] tracks = new TrackState[0];

  /** Full targeting result for the selected person. */
  public static final class AimSolution {
    public final int selectedIndex;
    public final double currentAngleRot;
    public final double predictedAngleRot;
    public final double distanceMeters;
    public final double leadTimeSeconds;
    public final double angularVelocityRotPerSec;
    public final boolean hasTarget;

    private AimSolution(
        int selectedIndex,
        double currentAngleRot,
        double predictedAngleRot,
        double distanceMeters,
        double leadTimeSeconds,
        double angularVelocityRotPerSec,
        boolean hasTarget) {
      this.selectedIndex = selectedIndex;
      this.currentAngleRot = currentAngleRot;
      this.predictedAngleRot = predictedAngleRot;
      this.distanceMeters = distanceMeters;
      this.leadTimeSeconds = leadTimeSeconds;
      this.angularVelocityRotPerSec = angularVelocityRotPerSec;
      this.hasTarget = hasTarget;
    }
  }

  private static final class TrackState {
    double angleRot = Double.NaN;
    double angularVelocityRotPerSec = 0.0;
    double timestampSec = Double.NaN;
  }

  /**
   * Logs per-detection angles and estimated field positions for AdvantageScope debugging.
   *
   * @param people Person detections from limelight-mm (may be empty).
   * @param robotPose Current robot pose on the field.
   */
  public void logEstimatedPositions(RawDetection[] people, Pose2d robotPose) {
    RawDetection[] sortedPeople = sortedLeftToRight(people);
    int count = sortedPeople.length;
    double[] txDeg = new double[count];
    double[] tyDeg = new double[count];
    double[] area = new double[count];
    double[] distance = new double[count];
    double[] robotRelativeAngleDeg = new double[count];
    Pose2d[] fieldPositions = new Pose2d[count];

    double robotHeadingRad = robotPose.getRotation().getRadians();
    for (int i = 0; i < count; i++) {
      RawDetection person = sortedPeople[i];
      txDeg[i] = person.txnc;
      tyDeg[i] = person.tync;
      area[i] = person.ta;
      distance[i] = estimateDistanceMeters(person);

      double angleRot = personBearingRot(person);
      robotRelativeAngleDeg[i] = angleRot * 360.0;

      double fieldBearingRad = robotHeadingRad + angleRot * 2.0 * Math.PI;
      Translation2d offset =
          new Translation2d(
              distance[i] * Math.cos(fieldBearingRad), distance[i] * Math.sin(fieldBearingRad));
      fieldPositions[i] = new Pose2d(robotPose.getTranslation().plus(offset), Rotation2d.kZero);
    }

    Logger.recordOutput("PersonTrack/AllTxDeg", txDeg);
    Logger.recordOutput("PersonTrack/AllTyDeg", tyDeg);
    Logger.recordOutput("PersonTrack/AllArea", area);
    Logger.recordOutput("PersonTrack/AllDistanceMeters", distance);
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
    return computeAimSolution(people, currentAngleRot).predictedAngleRot;
  }

  /**
   * Computes a led turret angle and distance for the selected left-to-right person slot.
   *
   * @param people Person detections from limelight-mm (may be empty).
   * @param currentAngleRot Current turret position in mechanism rotations.
   * @return Aim solution with predicted angle, range, and target motion estimate.
   */
  public AimSolution computeAimSolution(RawDetection[] people, double currentAngleRot) {
    if (people.length == 0) {
      if (Double.isNaN(lastHeldAngleRot)) {
        lastHeldAngleRot = currentAngleRot;
      }
      Logger.recordOutput("PersonTrack/SelectedIndex", -1);
      Logger.recordOutput("PersonTrack/SelectedTxDeg", 0.0);
      Logger.recordOutput("PersonTrack/SelectedDistanceMeters", lastHeldDistanceM);
      Logger.recordOutput("PersonTrack/LeadTimeSeconds", 0.0);
      Logger.recordOutput("PersonTrack/AngularVelocityRotPerSec", 0.0);
      Logger.recordOutput("PersonTrack/TargetAngleRot", lastHeldAngleRot);
      Logger.recordOutput("PersonTrack/Holding", true);
      return new AimSolution(
          -1, lastHeldAngleRot, lastHeldAngleRot, lastHeldDistanceM, 0.0, 0.0, false);
    }

    Logger.recordOutput("PersonTrack/Holding", false);

    RawDetection[] sortedPeople = sortedLeftToRight(people);
    if (selectedIndex >= sortedPeople.length) {
      selectedIndex = sortedPeople.length - 1;
    }
    if (selectedIndex < 0) {
      selectedIndex = 0;
    }

    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    updateTracks(sortedPeople, now);

    RawDetection target = sortedPeople[selectedIndex];
    double angleRot = personBearingRot(target);
    double distanceMeters = estimateDistanceMeters(target);
    double leadTimeSeconds =
        MathUtil.clamp(ShooterLookup.getToFMap().get(distanceMeters), 0.0, MAX_LEAD_SECONDS);
    double angularVelocityRotPerSec = tracks[selectedIndex].angularVelocityRotPerSec;
    double predictedAngleRot =
        clampTurretAngle(angleRot + angularVelocityRotPerSec * leadTimeSeconds);

    lastHeldAngleRot = predictedAngleRot;
    lastHeldDistanceM = distanceMeters;
    Logger.recordOutput("PersonTrack/SelectedIndex", selectedIndex);
    Logger.recordOutput("PersonTrack/SelectedTxDeg", target.txnc);
    Logger.recordOutput("PersonTrack/SelectedDistanceMeters", distanceMeters);
    Logger.recordOutput("PersonTrack/LeadTimeSeconds", leadTimeSeconds);
    Logger.recordOutput("PersonTrack/AngularVelocityRotPerSec", angularVelocityRotPerSec);
    Logger.recordOutput("PersonTrack/CurrentAngleRot", angleRot);
    Logger.recordOutput("PersonTrack/TargetAngleRot", predictedAngleRot);
    return new AimSolution(
        selectedIndex,
        angleRot,
        predictedAngleRot,
        distanceMeters,
        leadTimeSeconds,
        angularVelocityRotPerSec,
        true);
  }

  public double getLastDistanceMeters() {
    return lastHeldDistanceM;
  }

  private void updateTracks(RawDetection[] sortedPeople, double now) {
    if (tracks.length != sortedPeople.length) {
      tracks = new TrackState[sortedPeople.length];
      for (int i = 0; i < tracks.length; i++) {
        tracks[i] = new TrackState();
      }
    }

    for (int i = 0; i < sortedPeople.length; i++) {
      TrackState track = tracks[i];
      double angleRot = personBearingRot(sortedPeople[i]);
      if (!Double.isNaN(track.angleRot) && !Double.isNaN(track.timestampSec)) {
        double dt = now - track.timestampSec;
        if (dt > 1e-3 && dt <= MAX_TRACK_DT_SECONDS) {
          double measuredVelocity =
              MathUtil.inputModulus(angleRot - track.angleRot, -0.5, 0.5) / dt;
          track.angularVelocityRotPerSec =
              track.angularVelocityRotPerSec * (1.0 - VELOCITY_FILTER_ALPHA)
                  + measuredVelocity * VELOCITY_FILTER_ALPHA;
        } else if (dt > MAX_TRACK_DT_SECONDS) {
          track.angularVelocityRotPerSec = 0.0;
        }
      }

      track.angleRot = angleRot;
      track.timestampSec = now;
    }
  }

  private static RawDetection[] sortedLeftToRight(RawDetection[] people) {
    RawDetection[] sorted = Arrays.copyOf(people, people.length);
    Arrays.sort(sorted, Comparator.comparingDouble(person -> person.txnc));
    return sorted;
  }

  private static double estimateDistanceMeters(RawDetection person) {
    double verticalAngleRad = Math.toRadians(CAMERA_PITCH_DEG + person.tync);
    double heightDelta = PERSON_TARGET_HEIGHT_M - CAMERA_HEIGHT_M;
    double tan = Math.tan(verticalAngleRad);
    if (heightDelta <= 0.0
        || Math.abs(tan) < 1e-3
        || Math.signum(heightDelta) != Math.signum(tan)) {
      return ESTIMATED_PERSON_DISTANCE_M;
    }

    return MathUtil.clamp(
        Math.abs(heightDelta / tan), MIN_PERSON_DISTANCE_M, MAX_PERSON_DISTANCE_M);
  }

  private static double personBearingRot(RawDetection person) {
    return clampTurretAngle(CAMERA_YAW_OFFSET_ROT + TX_SIGN * (person.txnc / 360.0));
  }

  private static double clampTurretAngle(double angleRot) {
    return MathUtil.clamp(angleRot, DualEncoderCRT.REVERSE_LIMIT, DualEncoderCRT.FORWARD_LIMIT);
  }
}
