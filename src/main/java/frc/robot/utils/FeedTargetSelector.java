package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Superstructure.FeedMode;

public final class FeedTargetSelector {
  public static final double HUB_SQUARE_HALF_SIZE_METERS = 1.4 / 2.0;
  public static final double HUB_EXTENSION_WIDTH_METERS = 58.0 * 0.0254;
  public static final double HUB_EXTENSION_DEPTH_METERS = 27.0 * 0.0254;
  public static final double HUB_EXTENSION_PROTRUSION_METERS = 10.0 * 0.0254;
  public static final double TARGET_SHIFT_RADIUS_METERS = 1.0;
  public static final double TARGET_SHIFT_STEP_METERS = 0.1;
  private static final int TARGET_SHIFT_ANGLE_SAMPLES = 24;

  public enum FeedSide {
    LEFT,
    RIGHT
  }

  public record FeedSelection(
      Translation2d preferredTarget,
      Translation2d resolvedTarget,
      FeedSide side,
      double offsetMeters,
      boolean blockedByHub,
      double clearanceMeters) {}

  private FeedTargetSelector() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static FeedSelection selectTeleopTarget(
      FeedMode feedMode,
      Translation2d robotPosition,
      Translation2d turretPosition,
      Translation2d leftFeedTarget,
      Translation2d rightFeedTarget,
      Translation2d hubPosition) {
    FeedSide side = resolveSide(feedMode, robotPosition, leftFeedTarget, rightFeedTarget);
    Translation2d preferredTarget = side == FeedSide.LEFT ? leftFeedTarget : rightFeedTarget;
    return resolveShiftedTarget(
        turretPosition, preferredTarget, side, leftFeedTarget, rightFeedTarget, hubPosition);
  }

  public static FeedSelection selectAutoTarget(
      Translation2d robotPosition, Translation2d leftFeedTarget, Translation2d rightFeedTarget) {
    FeedSide side = resolveAutoSide(robotPosition, leftFeedTarget, rightFeedTarget);
    Translation2d target = side == FeedSide.LEFT ? leftFeedTarget : rightFeedTarget;
    return new FeedSelection(target, target, side, 0.0, false, Double.POSITIVE_INFINITY);
  }

  public static FeedSelection resolveShiftedTarget(
      Translation2d turretPosition,
      Translation2d preferredTarget,
      FeedSide side,
      Translation2d leftFeedTarget,
      Translation2d rightFeedTarget,
      Translation2d hubPosition) {
    Translation2d shiftAxis = leftFeedTarget.minus(rightFeedTarget);
    double phaseRadians = shiftAxis.getNorm() < 1e-9 ? 0.0 : shiftAxis.getAngle().getRadians();
    double preferredClearance = hubClearance(turretPosition, preferredTarget, hubPosition);
    if (preferredClearance > 0.0) {
      return new FeedSelection(
          preferredTarget, preferredTarget, side, 0.0, false, preferredClearance);
    }

    int maxSteps = (int) Math.round(TARGET_SHIFT_RADIUS_METERS / TARGET_SHIFT_STEP_METERS);
    for (int step = 1; step <= maxSteps; step++) {
      double radius = step * TARGET_SHIFT_STEP_METERS;
      Translation2d bestCandidate = null;
      double bestClearance = Double.NEGATIVE_INFINITY;

      for (int sample = 0; sample < TARGET_SHIFT_ANGLE_SAMPLES; sample++) {
        double angleRadians = phaseRadians + (2.0 * Math.PI * sample) / TARGET_SHIFT_ANGLE_SAMPLES;
        Translation2d candidate =
            preferredTarget.plus(new Translation2d(radius, Rotation2d.fromRadians(angleRadians)));
        double candidateClearance = hubClearance(turretPosition, candidate, hubPosition);
        if (candidateClearance > 0.0 && candidateClearance > bestClearance) {
          bestCandidate = candidate;
          bestClearance = candidateClearance;
        }
      }

      if (bestCandidate != null) {
        return new FeedSelection(
            preferredTarget, bestCandidate, side, radius, false, bestClearance);
      }
    }

    return new FeedSelection(preferredTarget, preferredTarget, side, 0.0, true, preferredClearance);
  }

  public static double hubClearance(
      Translation2d shooterPosition, Translation2d targetPosition, Translation2d hubPosition) {
    RectangleGeometry square =
        new RectangleGeometry(
            hubPosition.getX(),
            hubPosition.getY(),
            HUB_SQUARE_HALF_SIZE_METERS,
            HUB_SQUARE_HALF_SIZE_METERS);
    RectangleGeometry extension = neutralSideExtension(shooterPosition, hubPosition);

    if (shooterPosition.getDistance(targetPosition) < 1e-9) {
      return Math.min(
          pointToRectangleDistance(shooterPosition, square),
          pointToRectangleDistance(shooterPosition, extension));
    }
    return Math.min(
        segmentToRectangleDistance(shooterPosition, targetPosition, square),
        segmentToRectangleDistance(shooterPosition, targetPosition, extension));
  }

  public static boolean isPathBlockedByHub(
      Translation2d shooterPosition, Translation2d targetPosition, Translation2d hubPosition) {
    return hubClearance(shooterPosition, targetPosition, hubPosition) <= 0.0;
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

  private static double pointToRectangleDistance(Translation2d point, RectangleGeometry rectangle) {
    double dx =
        Math.max(Math.abs(point.getX() - rectangle.centerX()) - rectangle.halfWidthX(), 0.0);
    double dy =
        Math.max(Math.abs(point.getY() - rectangle.centerY()) - rectangle.halfWidthY(), 0.0);
    return Math.hypot(dx, dy);
  }

  private static double segmentToRectangleDistance(
      Translation2d start, Translation2d end, RectangleGeometry rectangle) {
    if (segmentIntersectsRectangle(start, end, rectangle)) {
      return 0.0;
    }

    Translation2d topLeft =
        new Translation2d(
            rectangle.centerX() - rectangle.halfWidthX(),
            rectangle.centerY() + rectangle.halfWidthY());
    Translation2d topRight =
        new Translation2d(
            rectangle.centerX() + rectangle.halfWidthX(),
            rectangle.centerY() + rectangle.halfWidthY());
    Translation2d bottomLeft =
        new Translation2d(
            rectangle.centerX() - rectangle.halfWidthX(),
            rectangle.centerY() - rectangle.halfWidthY());
    Translation2d bottomRight =
        new Translation2d(
            rectangle.centerX() + rectangle.halfWidthX(),
            rectangle.centerY() - rectangle.halfWidthY());

    double minDistance =
        Math.min(
            pointToRectangleDistance(start, rectangle), pointToRectangleDistance(end, rectangle));
    minDistance = Math.min(minDistance, segmentToSegmentDistance(start, end, topLeft, topRight));
    minDistance =
        Math.min(minDistance, segmentToSegmentDistance(start, end, topRight, bottomRight));
    minDistance =
        Math.min(minDistance, segmentToSegmentDistance(start, end, bottomRight, bottomLeft));
    minDistance = Math.min(minDistance, segmentToSegmentDistance(start, end, bottomLeft, topLeft));
    return minDistance;
  }

  private static boolean segmentIntersectsRectangle(
      Translation2d start, Translation2d end, RectangleGeometry rectangle) {
    double minX = rectangle.centerX() - rectangle.halfWidthX();
    double maxX = rectangle.centerX() + rectangle.halfWidthX();
    double minY = rectangle.centerY() - rectangle.halfWidthY();
    double maxY = rectangle.centerY() + rectangle.halfWidthY();

    double tMin = 0.0;
    double tMax = 1.0;
    double dx = end.getX() - start.getX();
    double dy = end.getY() - start.getY();

    if (!clipSquareAxis(-dx, start.getX() - minX, tMin, tMax)) {
      return false;
    }
    tMin = clipMin;
    tMax = clipMax;
    if (!clipSquareAxis(dx, maxX - start.getX(), tMin, tMax)) {
      return false;
    }
    tMin = clipMin;
    tMax = clipMax;
    if (!clipSquareAxis(-dy, start.getY() - minY, tMin, tMax)) {
      return false;
    }
    tMin = clipMin;
    tMax = clipMax;
    return clipSquareAxis(dy, maxY - start.getY(), tMin, tMax);
  }

  private static double clipMin;
  private static double clipMax;

  private static boolean clipSquareAxis(double p, double q, double currentMin, double currentMax) {
    clipMin = currentMin;
    clipMax = currentMax;
    if (Math.abs(p) < 1e-9) {
      return q >= 0.0;
    }

    double r = q / p;
    if (p < 0.0) {
      if (r > clipMax) {
        return false;
      }
      clipMin = Math.max(clipMin, r);
      return true;
    }

    if (r < clipMin) {
      return false;
    }
    clipMax = Math.min(clipMax, r);
    return true;
  }

  private static double segmentToSegmentDistance(
      Translation2d aStart, Translation2d aEnd, Translation2d bStart, Translation2d bEnd) {
    if (segmentsIntersect(aStart, aEnd, bStart, bEnd)) {
      return 0.0;
    }

    double distance = pointToSegmentDistance(aStart, bStart, bEnd);
    distance = Math.min(distance, pointToSegmentDistance(aEnd, bStart, bEnd));
    distance = Math.min(distance, pointToSegmentDistance(bStart, aStart, aEnd));
    distance = Math.min(distance, pointToSegmentDistance(bEnd, aStart, aEnd));
    return distance;
  }

  private static double pointToSegmentDistance(
      Translation2d point, Translation2d segmentStart, Translation2d segmentEnd) {
    Translation2d segment = segmentEnd.minus(segmentStart);
    double segmentLengthSquared = segment.getNorm() * segment.getNorm();
    if (segmentLengthSquared < 1e-9) {
      return point.getDistance(segmentStart);
    }

    double t = point.minus(segmentStart).dot(segment) / segmentLengthSquared;
    t = Math.max(0.0, Math.min(1.0, t));
    Translation2d closestPoint = segmentStart.plus(segment.times(t));
    return point.getDistance(closestPoint);
  }

  private static boolean segmentsIntersect(
      Translation2d aStart, Translation2d aEnd, Translation2d bStart, Translation2d bEnd) {
    double o1 = orientation(aStart, aEnd, bStart);
    double o2 = orientation(aStart, aEnd, bEnd);
    double o3 = orientation(bStart, bEnd, aStart);
    double o4 = orientation(bStart, bEnd, aEnd);

    if (o1 * o2 < 0.0 && o3 * o4 < 0.0) {
      return true;
    }

    return (Math.abs(o1) < 1e-9 && onSegment(aStart, bStart, aEnd))
        || (Math.abs(o2) < 1e-9 && onSegment(aStart, bEnd, aEnd))
        || (Math.abs(o3) < 1e-9 && onSegment(bStart, aStart, bEnd))
        || (Math.abs(o4) < 1e-9 && onSegment(bStart, aEnd, bEnd));
  }

  private static double orientation(Translation2d a, Translation2d b, Translation2d c) {
    return (b.getX() - a.getX()) * (c.getY() - a.getY())
        - (b.getY() - a.getY()) * (c.getX() - a.getX());
  }

  private static boolean onSegment(Translation2d start, Translation2d point, Translation2d end) {
    return point.getX() >= Math.min(start.getX(), end.getX()) - 1e-9
        && point.getX() <= Math.max(start.getX(), end.getX()) + 1e-9
        && point.getY() >= Math.min(start.getY(), end.getY()) - 1e-9
        && point.getY() <= Math.max(start.getY(), end.getY()) + 1e-9;
  }

  private static RectangleGeometry neutralSideExtension(
      Translation2d shooterPosition, Translation2d hubPosition) {
    double neutralDirection = shooterPosition.getX() >= hubPosition.getX() ? 1.0 : -1.0;
    double extensionHalfWidthX = HUB_EXTENSION_DEPTH_METERS / 2.0;
    double extensionHalfWidthY = HUB_EXTENSION_WIDTH_METERS / 2.0;
    double extensionCenterX =
        hubPosition.getX()
            + neutralDirection
                * (HUB_SQUARE_HALF_SIZE_METERS
                    + HUB_EXTENSION_PROTRUSION_METERS
                    - extensionHalfWidthX);
    return new RectangleGeometry(
        extensionCenterX, hubPosition.getY(), extensionHalfWidthX, extensionHalfWidthY);
  }

  private record RectangleGeometry(
      double centerX, double centerY, double halfWidthX, double halfWidthY) {}
}
