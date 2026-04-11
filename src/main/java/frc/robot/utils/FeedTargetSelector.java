package frc.robot.utils;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.Superstructure.FeedMode;

public final class FeedTargetSelector {
  // public static final Distance HUB_SQUARE_HALF_SIZE = Meters.of(1.4 / 2.0);
  public static final Distance HUB_SQUARE_HALF_SIZE = Meters.of(0);
  public static final Distance HUB_EXTENSION_WIDTH = Inches.of(58.0);
  public static final Distance HUB_EXTENSION_DEPTH = Inches.of(10.0);
  public static final Distance HUB_EXTENSION_PROTRUSION = Inches.of(10.0);
  public static final Distance TARGET_SHIFT_RADIUS = Meters.of(1.0);
  public static final Distance TARGET_SHIFT_STEP = Meters.of(0.1);
  public static final Distance CLEAR_PATH_SENTINEL = Meters.of(1_000.0);
  private static final int TARGET_SHIFT_ANGLE_SAMPLES = 24;

  public enum FeedSide {
    LEFT,
    RIGHT
  }

  public record FeedSelection(
      Translation2d preferredTarget,
      Translation2d resolvedTarget,
      FeedSide side,
      Distance offset,
      boolean blockedByHub,
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
    return new FeedSelection(target, target, side, Meters.of(0.0), false, CLEAR_PATH_SENTINEL);
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
    Distance preferredClearance = hubClearance(turretPosition, preferredTarget, hubPosition);
    if (preferredClearance.in(Meters) > 0.0) {
      return new FeedSelection(
          preferredTarget, preferredTarget, side, Meters.of(0.0), false, preferredClearance);
    }

    int maxSteps = (int) Math.round(TARGET_SHIFT_RADIUS.in(Meters) / TARGET_SHIFT_STEP.in(Meters));
    for (int step = 1; step <= maxSteps; step++) {
      Distance radius = Meters.of(step * TARGET_SHIFT_STEP.in(Meters));
      Translation2d bestCandidate = null;
      double bestClearance = Double.NEGATIVE_INFINITY;

      for (int sample = 0; sample < TARGET_SHIFT_ANGLE_SAMPLES; sample++) {
        double angleRadians = phaseRadians + (2.0 * Math.PI * sample) / TARGET_SHIFT_ANGLE_SAMPLES;
        Translation2d candidate =
            preferredTarget.plus(
                new Translation2d(radius.in(Meters), Rotation2d.fromRadians(angleRadians)));
        double candidateClearance = hubClearance(turretPosition, candidate, hubPosition).in(Meters);
        if (candidateClearance > 0.0 && candidateClearance > bestClearance) {
          bestCandidate = candidate;
          bestClearance = candidateClearance;
        }
      }

      if (bestCandidate != null) {
        return new FeedSelection(
            preferredTarget, bestCandidate, side, radius, false, Meters.of(bestClearance));
      }
    }

    return new FeedSelection(
        preferredTarget, preferredTarget, side, Meters.of(0.0), true, preferredClearance);
  }

  public static Distance hubClearance(
      Translation2d shooterPosition, Translation2d targetPosition, Translation2d hubPosition) {
    RectangleGeometry square =
        new RectangleGeometry(
            hubPosition.getX(),
            hubPosition.getY(),
            HUB_SQUARE_HALF_SIZE.in(Meters),
            HUB_SQUARE_HALF_SIZE.in(Meters));
    RectangleGeometry extension = neutralSideExtension(shooterPosition, hubPosition);

    if (shooterPosition.getDistance(targetPosition) < 1e-9) {
      return Meters.of(
          Math.min(
              pointToRectangleDistance(shooterPosition, square),
              pointToRectangleDistance(shooterPosition, extension)));
    }
    return Meters.of(
        Math.min(
            segmentToRectangleDistance(shooterPosition, targetPosition, square),
            segmentToRectangleDistance(shooterPosition, targetPosition, extension)));
  }

  public static boolean isPathBlockedByHub(
      Translation2d shooterPosition, Translation2d targetPosition, Translation2d hubPosition) {
    return hubClearance(shooterPosition, targetPosition, hubPosition).in(Meters) <= 0.0;
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

    double dx = end.getX() - start.getX();
    double dy = end.getY() - start.getY();

    ClipRange clipped = clipRectangleAxis(-dx, start.getX() - minX, new ClipRange(0.0, 1.0));
    if (clipped == null) {
      return false;
    }
    clipped = clipRectangleAxis(dx, maxX - start.getX(), clipped);
    if (clipped == null) {
      return false;
    }
    clipped = clipRectangleAxis(-dy, start.getY() - minY, clipped);
    if (clipped == null) {
      return false;
    }
    return clipRectangleAxis(dy, maxY - start.getY(), clipped) != null;
  }

  private static ClipRange clipRectangleAxis(double p, double q, ClipRange currentRange) {
    if (Math.abs(p) < 1e-9) {
      return q >= 0.0 ? currentRange : null;
    }

    double min = currentRange.min();
    double max = currentRange.max();
    double r = q / p;
    if (p < 0.0) {
      if (r > max) {
        return null;
      }
      return new ClipRange(Math.max(min, r), max);
    }

    if (r < min) {
      return null;
    }
    return new ClipRange(min, Math.min(max, r));
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
    double extensionHalfWidthX = HUB_EXTENSION_DEPTH.in(Meters) / 2.0;
    double extensionHalfWidthY = HUB_EXTENSION_WIDTH.in(Meters) / 2.0;
    double extensionCenterX =
        hubPosition.getX()
            + neutralDirection
                * (HUB_SQUARE_HALF_SIZE.in(Meters)
                    + HUB_EXTENSION_PROTRUSION.in(Meters)
                    - extensionHalfWidthX);
    return new RectangleGeometry(
        extensionCenterX, hubPosition.getY(), extensionHalfWidthX, extensionHalfWidthY);
  }

  private record ClipRange(double min, double max) {}

  private record RectangleGeometry(
      double centerX, double centerY, double halfWidthX, double halfWidthY) {}
}
