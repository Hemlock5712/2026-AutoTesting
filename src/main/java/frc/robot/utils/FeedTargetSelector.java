package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.Superstructure.FeedMode;

public final class FeedTargetSelector {
  public static final Distance CLEAR_PATH_SENTINEL = Meters.of(1000.0);
  private static final Distance ZERO_DISTANCE = Meters.of(0.0);

  public enum FeedSide {
    LEFT,
    RIGHT
  }

  public record FeedSelection(
      Translation2d preferredTarget,
      Translation2d resolvedTarget,
      FeedSide side,
      Distance offset,
      boolean blocked,
      Distance clearance) {}

  private FeedTargetSelector() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static FeedSelection selectTeleopTarget(
      FeedMode feedMode,
      Translation2d robotPosition,
      Translation2d turretPosition,
      Translation2d leftFeedTarget,
      Translation2d rightFeedTarget,
      Translation2d netCenter) {
    FeedSide side = resolveSide(feedMode, robotPosition, leftFeedTarget, rightFeedTarget);
    Translation2d target = side == FeedSide.LEFT ? leftFeedTarget : rightFeedTarget;
    Distance clearance = netClearance(turretPosition, target, netCenter);
    boolean blocked = clearance.in(Meters) <= 0.0;
    return new FeedSelection(target, target, side, ZERO_DISTANCE, blocked, clearance);
  }

  public static FeedSelection selectAutoTarget(
      Translation2d robotPosition, Translation2d leftFeedTarget, Translation2d rightFeedTarget) {
    FeedSide side = resolveAutoSide(robotPosition, leftFeedTarget, rightFeedTarget);
    Translation2d target = side == FeedSide.LEFT ? leftFeedTarget : rightFeedTarget;
    return new FeedSelection(target, target, side, ZERO_DISTANCE, false, CLEAR_PATH_SENTINEL);
  }

  public static Distance netClearance(
      Translation2d shooterPosition, Translation2d targetPosition, Translation2d netCenter) {
    double lineX = netCenter.getX();
    double lineMinY = netCenter.getY() - FieldInfo.NET_INSET.in(Meters) / 2.0;
    double lineMaxY = netCenter.getY() + FieldInfo.NET_INSET.in(Meters) / 2.0;

    double sx = shooterPosition.getX();
    double sy = shooterPosition.getY();
    double tx = targetPosition.getX();
    double ty = targetPosition.getY();
    double dx = tx - sx;

    if (Math.abs(dx) < 1e-9) {
      return Meters.of(Math.abs(sx - lineX));
    }

    double t = (lineX - sx) / dx;
    if (t < 0.0 || t > 1.0) {
      return CLEAR_PATH_SENTINEL;
    }

    double crossY = sy + t * (ty - sy);
    if (crossY >= lineMinY && crossY <= lineMaxY) {
      return ZERO_DISTANCE;
    }

    return Meters.of(Math.min(Math.abs(crossY - lineMinY), Math.abs(crossY - lineMaxY)));
  }

  public static boolean isPathBlocked(
      Translation2d shooterPosition, Translation2d targetPosition, Translation2d netCenter) {
    return netClearance(shooterPosition, targetPosition, netCenter).in(Meters) <= 0.0;
  }

  private static FeedSide resolveSide(
      FeedMode feedMode,
      Translation2d robotPosition,
      Translation2d leftFeedTarget,
      Translation2d rightFeedTarget) {
    return switch (feedMode) {
      case FORCE_LEFT -> FeedSide.LEFT;
      case FORCE_RIGHT -> FeedSide.RIGHT;
      case AUTO -> resolveAutoSide(robotPosition, leftFeedTarget, rightFeedTarget);
    };
  }

  private static FeedSide resolveAutoSide(
      Translation2d robotPosition, Translation2d leftFeedTarget, Translation2d rightFeedTarget) {
    Translation2d upperFeed =
        leftFeedTarget.getY() > rightFeedTarget.getY() ? leftFeedTarget : rightFeedTarget;
    Translation2d lowerFeed =
        leftFeedTarget.getY() > rightFeedTarget.getY() ? rightFeedTarget : leftFeedTarget;
    double midlineY = (upperFeed.getY() + lowerFeed.getY()) / 2.0;
    return robotPosition.getY() > midlineY
        ? sideForTarget(upperFeed, leftFeedTarget, rightFeedTarget)
        : sideForTarget(lowerFeed, leftFeedTarget, rightFeedTarget);
  }

  private static FeedSide sideForTarget(
      Translation2d target, Translation2d leftFeedTarget, Translation2d rightFeedTarget) {
    return target.getDistance(leftFeedTarget) <= target.getDistance(rightFeedTarget)
        ? FeedSide.LEFT
        : FeedSide.RIGHT;
  }
}
