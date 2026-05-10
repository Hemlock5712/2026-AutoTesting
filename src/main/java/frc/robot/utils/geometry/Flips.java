package frc.robot.utils.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.FieldInfo;

/**
 * Static helpers for flipping field-frame poses across the field's lines of symmetry. Used
 * internally by {@link ExtGeometry} subclasses to precompute the three flipped variants. External
 * code should use the {@code Ext*} containers and call {@code .get()} instead.
 */
final class Flips {

  private Flips() {}

  // ---------- Translation ----------

  static Translation2d overWidth(Translation2d t) {
    return new Translation2d(t.getMeasureX(), FieldInfo.width().minus(t.getMeasureY()));
  }

  static Translation2d overLength(Translation2d t) {
    return new Translation2d(FieldInfo.length().minus(t.getMeasureX()), t.getMeasureY());
  }

  static Translation2d overDiagonal(Translation2d t) {
    return new Translation2d(
        FieldInfo.length().minus(t.getMeasureX()), FieldInfo.width().minus(t.getMeasureY()));
  }

  // ---------- Rotation ----------

  static Rotation2d overWidth(Rotation2d r) {
    return r.unaryMinus();
  }

  static Rotation2d overLength(Rotation2d r) {
    return new Rotation2d(-r.getCos(), r.getSin());
  }

  static Rotation2d overDiagonal(Rotation2d r) {
    return new Rotation2d(-r.getCos(), -r.getSin());
  }

  // ---------- Pose ----------

  static Pose2d overWidth(Pose2d p) {
    return new Pose2d(overWidth(p.getTranslation()), overWidth(p.getRotation()));
  }

  static Pose2d overLength(Pose2d p) {
    return new Pose2d(overLength(p.getTranslation()), overLength(p.getRotation()));
  }

  static Pose2d overDiagonal(Pose2d p) {
    return new Pose2d(overDiagonal(p.getTranslation()), overDiagonal(p.getRotation()));
  }
}
