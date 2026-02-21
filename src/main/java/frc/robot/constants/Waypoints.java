package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Field position constants as Pose2d values. */
public final class Waypoints {
  private Waypoints() {} // Prevent instantiation

  // Starting positions
  public static final Pose2d START_LEFT = new Pose2d(7.150, 6.050, Rotation2d.fromDegrees(-120));
  public static final Pose2d START_CENTER = new Pose2d(7.150, 4.050, Rotation2d.fromDegrees(0));
  public static final Pose2d START_RIGHT = new Pose2d(7.150, 2.050, Rotation2d.fromDegrees(120));

  // Intake positions (where robot picks up game pieces)
  public static final Pose2d INTAKE_1 = new Pose2d(1.690, 7.374, Rotation2d.fromDegrees(-60));
  public static final Pose2d INTAKE_2 = new Pose2d(1.275, 7.074, Rotation2d.fromDegrees(-60));
  public static final Pose2d INTAKE_3 = new Pose2d(0.697, 6.645, Rotation2d.fromDegrees(-60));

  // Approach positions (0.5m back from intake positions to allow pathfinding room)
  public static final Pose2d INTAKE_1_APPROACH =
      new Pose2d(2.53, 3.63, Rotation2d.fromDegrees(145));
  public static final Pose2d INTAKE_2_APPROACH = new Pose2d(3.5, 7.074, Rotation2d.fromDegrees(0));
  public static final Pose2d INTAKE_3_APPROACH = new Pose2d(3.5, 7.074, Rotation2d.fromDegrees(0));

  // Endpoint positions (where robot drives to if no game piece detected during search)
  public static final Pose2d INTAKE_1_ENDPOINT = new Pose2d(1.0, 4.50, Rotation2d.fromDegrees(145));
  public static final Pose2d INTAKE_2_ENDPOINT =
      new Pose2d(1.275, 7.074, Rotation2d.fromDegrees(-60));
  public static final Pose2d INTAKE_3_ENDPOINT =
      new Pose2d(0.697, 6.645, Rotation2d.fromDegrees(-60));

  // Scoring positions (where robot places game pieces)
  public static final Pose2d SCORE_A = new Pose2d(3.161, 4.2, Rotation2d.fromDegrees(0));
  public static final Pose2d SCORE_B = new Pose2d(3.161, 3.864, Rotation2d.fromDegrees(0));
  public static final Pose2d SCORE_C = new Pose2d(3.671, 2.954, Rotation2d.fromDegrees(60));
  public static final Pose2d SCORE_D = new Pose2d(3.969, 2.778, Rotation2d.fromDegrees(60));
  public static final Pose2d SCORE_E = new Pose2d(5.022, 2.954, Rotation2d.fromDegrees(120));
  public static final Pose2d SCORE_F = new Pose2d(5.303, 2.778, Rotation2d.fromDegrees(120));
  public static final Pose2d SCORE_G = new Pose2d(5.832, 3.864, Rotation2d.fromDegrees(180));
  public static final Pose2d SCORE_H = new Pose2d(5.832, 4.2, Rotation2d.fromDegrees(180));
  public static final Pose2d SCORE_I = new Pose2d(5.303, 5.100, Rotation2d.fromDegrees(-120));
  public static final Pose2d SCORE_J = new Pose2d(5.022, 5.255, Rotation2d.fromDegrees(-120));
  public static final Pose2d SCORE_K = new Pose2d(3.969, 5.255, Rotation2d.fromDegrees(-60));
  public static final Pose2d SCORE_L = new Pose2d(3.671, 5.100, Rotation2d.fromDegrees(-60));

  // Intermediate waypoints (for path planning)
  public static final Pose2d INT_LEFT = new Pose2d(5, 6.323, Rotation2d.fromDegrees(0));
  public static final Pose2d INT_RIGHT = new Pose2d(5, 1.826, Rotation2d.fromDegrees(0));

  // Midfield center
  public static final Pose2d MIDFIELD_CENTER = new Pose2d(8.27, 4.1, Rotation2d.fromDegrees(0));

  // Testing Poses
  public static final Pose2d TEST_1 = new Pose2d(2.82, 4.02, Rotation2d.fromDegrees(180));
  public static final Pose2d TEST_2 = new Pose2d(1.5, 4.85, Rotation2d.fromDegrees(0));
  public static final Pose2d TEST_3 = new Pose2d(1.5, 3.15, Rotation2d.fromDegrees(0));
}
