package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
  public static final Translation2d HUB_POSITION = new Translation2d(4.621, 4.030);

  // Drive-in autonomous coordinates
  public static final double DRIVE_POINT_IN_X_START = 5.5;
  public static final double DRIVE_POINT_IN_X_END = 4.1;
  public static final double DRIVE_POINT_IN_Y_RIGHT = 0.64;
  public static final double DRIVE_POINT_IN_Y_LEFT = 7.5;

  /** Preset Y-axis lock positions for AxisLockDrive. */
  public enum YAxisLockPosition {
    REEF_CENTER(4.0),
    REEF_LEFT(3.0),
    REEF_RIGHT(5.0);

    private final double blueY;

    YAxisLockPosition(double blueY) {
      this.blueY = blueY;
    }

    public double getY() {
      return blueY;
    }
  }

  /** Preset X-axis lock positions for AxisLockDrive. */
  public enum XAxisLockPosition {
    SCORING_LINE(3.0),
    PICKUP_LINE(12.0);

    private final double blueX;

    XAxisLockPosition(double blueX) {
      this.blueX = blueX;
    }

    public double getX() {
      return blueX;
    }
  }

  /** Preset rotation lock angles for AxisLockDrive. */
  public enum RotationLockAngle {
    FACING_FORWARD(Rotation2d.kZero),
    FACING_LEFT(Rotation2d.fromDegrees(90)),
    FACING_RIGHT(Rotation2d.fromDegrees(-90)),
    FACING_BACK(Rotation2d.fromDegrees(180));

    private final Rotation2d angle;

    RotationLockAngle(Rotation2d angle) {
      this.angle = angle;
    }

    public Rotation2d getAngle() {
      return angle;
    }
  }
}
