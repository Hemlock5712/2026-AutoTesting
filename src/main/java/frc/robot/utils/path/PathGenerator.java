package frc.robot.utils.path;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Arrays;
import java.util.Optional;

/**
 * Builds a path at runtime: Theta* search → spline smoothing → velocity profile → {@link
 * GeneratedPath}.
 *
 * <p>Steps for the caller:
 *
 * <ul>
 *   <li>Set up an {@link ObstacleField} with the current obstacles
 *   <li>Build a {@link Costmap} from it and pass it in
 *   <li>Feed the returned path to {@link frc.robot.commands.FollowPath}
 * </ul>
 *
 * <p>The robot's heading is interpolated from start to goal. Use {@link
 * frc.robot.commands.FollowPath#withRotationSupplier} to override.
 */
public final class PathGenerator {

  /** A single path-generation request. */
  public record Request(Pose2d start, Pose2d goal, double startVelocity, double endVelocity) {}

  /** Generator configuration. */
  public record Config(double sampleSpacing, VelocityProfiler.Constraints constraints) {

    /** 0.05 m sample spacing + {@link VelocityProfiler.Constraints#defaults()}. */
    public static Config defaults() {
      return new Config(0.05, VelocityProfiler.Constraints.defaults());
    }
  }

  private PathGenerator() {}

  /**
   * Generates a path and double-checks that no point on the smoothed spline gets too close to an
   * obstacle. Theta* plans on a grid, but the smoothed spline can curve outside its cells - so we
   * verify clearance against the real obstacle shapes before returning.
   *
   * @return The path, or empty if no route exists or the smoothed path clips an obstacle.
   */
  public static Optional<GeneratedPath> generate(
      ObstacleField field, Costmap map, Request req, Config config) {
    ThetaStar.Result thetaResult =
        ThetaStar.search(map, req.start.getTranslation(), req.goal.getTranslation());
    if (!thetaResult.success()) {
      return Optional.empty();
    }

    SplineFitter.Samples spline = SplineFitter.fit(thetaResult.waypoints(), config.sampleSpacing);
    int n = spline.length();
    if (n < 2) {
      return Optional.empty();
    }

    // Treat the robot as a circle. Every sample AND every midpoint between samples must stay
    // clear of obstacles - midpoints catch thin obstacles that slip between two samples.
    double minClearance = map.robotRadius();
    for (int i = 0; i < n; i++) {
      if (field.signedDistance(spline.x()[i], spline.y()[i]) < minClearance) {
        return Optional.empty();
      }
      if (i < n - 1) {
        double mx = 0.5 * (spline.x()[i] + spline.x()[i + 1]);
        double my = 0.5 * (spline.y()[i] + spline.y()[i + 1]);
        if (field.signedDistance(mx, my) < minClearance) {
          return Optional.empty();
        }
      }
    }

    // The spline fitter spaces samples evenly, so every gap is the same.
    double[] ds = new double[n - 1];
    Arrays.fill(ds, spline.sampleSpacing());

    double[] velocity =
        VelocityProfiler.profile(
            spline.curvature(), ds, req.startVelocity, req.endVelocity, config.constraints);

    double[] heading = interpolateHeading(req.start.getRotation(), req.goal.getRotation(), n);

    return Optional.of(
        new GeneratedPath(spline.x(), spline.y(), heading, spline.curvature(), velocity));
  }

  /** Smoothly blends from start angle to end angle along the path. */
  private static double[] interpolateHeading(Rotation2d start, Rotation2d end, int n) {
    double startRad = start.getRadians();
    double diff = MathUtil.angleModulus(end.getRadians() - startRad);
    double[] h = new double[n];
    double denom = Math.max(1, n - 1);
    for (int i = 0; i < n; i++) {
      h[i] = startRad + diff * (i / denom);
    }
    return h;
  }

  /**
   * Like {@link #generate}, but treats the robot as a rectangle (not a circle). At narrow spots,
   * the robot must point along the path so its long side fits through. At open spots, it can rotate
   * freely.
   *
   * <p>Returns empty if the robot can't fit through a tight spot at any rotation.
   *
   * @param halfX Half the robot's length along its forward axis (m)
   * @param halfY Half the robot's width sideways (m)
   */
  public static Optional<GeneratedPath> generateOriented(
      ObstacleField field, Costmap map, Request req, double halfX, double halfY, Config config) {
    ThetaStar.Result thetaResult =
        ThetaStar.search(map, req.start.getTranslation(), req.goal.getTranslation());
    if (!thetaResult.success()) {
      return Optional.empty();
    }

    SplineFitter.Samples spline = SplineFitter.fit(thetaResult.waypoints(), config.sampleSpacing);
    int n = spline.length();
    if (n < 2) {
      return Optional.empty();
    }

    // The direction the path is pointing at each sample. The last one copies its neighbor.
    double[] tangentRad = new double[n];
    for (int i = 0; i < n - 1; i++) {
      double dx = spline.x()[i + 1] - spline.x()[i];
      double dy = spline.y()[i + 1] - spline.y()[i];
      tangentRad[i] = Math.atan2(dy, dx);
    }
    tangentRad[n - 1] = tangentRad[n - 2];

    // Step 1: check the robot fits along the path direction at every sample. If it can't fit
    // even pointed straight along the path, give up.
    for (int i = 0; i < n; i++) {
      double cos = Math.cos(tangentRad[i]);
      double sin = Math.sin(tangentRad[i]);
      if (field.clipsObb(spline.x()[i], spline.y()[i], cos, sin, halfX, halfY)) {
        return Optional.empty();
      }
      if (i < n - 1) {
        double mx = 0.5 * (spline.x()[i] + spline.x()[i + 1]);
        double my = 0.5 * (spline.y()[i] + spline.y()[i + 1]);
        // Average direction at the midpoint - close enough since samples are spaced tightly.
        double mc = 0.5 * (Math.cos(tangentRad[i]) + Math.cos(tangentRad[i + 1]));
        double ms = 0.5 * (Math.sin(tangentRad[i]) + Math.sin(tangentRad[i + 1]));
        double mmag = Math.hypot(mc, ms);
        if (mmag > 1e-9) {
          mc /= mmag;
          ms /= mmag;
        }
        if (field.clipsObb(mx, my, mc, ms, halfX, halfY)) {
          return Optional.empty();
        }
      }
    }

    // Step 2a: figure out how much each sample can rotate before it clips an obstacle.
    // If the sample is clear of all obstacles by more than the robot's diagonal, no rotation
    // can hit anything - skip the expensive sweep.
    double circumscribed = Math.hypot(halfX, halfY);
    double[] posSlack = new double[n];
    double[] negSlack = new double[n];
    for (int i = 0; i < n; i++) {
      double clearance = field.signedDistance(spline.x()[i], spline.y()[i]);
      if (clearance > circumscribed) {
        posSlack[i] = Math.PI / 2;
        negSlack[i] = Math.PI / 2;
      } else {
        SlackBounds b =
            computeHeadingSlackBounds(
                spline.x()[i], spline.y()[i], tangentRad[i], halfX, halfY, field);
        posSlack[i] = b.pos;
        negSlack[i] = b.neg;
      }
    }

    // Step 2b: smooth out the rotation limits so the robot has time to rotate when leaving
    // a tight zone (about 0.5 m of room).
    int smoothWindow = Math.max(1, (int) Math.round(0.5 / spline.sampleSpacing()));
    double[] smoothPos = minWindow(posSlack, smoothWindow);
    double[] smoothNeg = minWindow(negSlack, smoothWindow);

    // Step 2c: blend start→goal heading at each sample, clamped to whatever rotation actually
    // fits at this point on the path.
    double startRad = req.start.getRotation().getRadians();
    double diff = MathUtil.angleModulus(req.goal.getRotation().getRadians() - startRad);
    double[] heading = new double[n];
    double denom = Math.max(1, n - 1);
    for (int i = 0; i < n; i++) {
      double idealHeading = startRad + diff * (i / denom);
      double offset = MathUtil.angleModulus(idealHeading - tangentRad[i]);
      double clamped = Math.max(-smoothNeg[i], Math.min(smoothPos[i], offset));
      heading[i] = tangentRad[i] + clamped;
    }

    // Step 3: double-check the headings we chose actually clear obstacles.
    for (int i = 0; i < n; i++) {
      double cos = Math.cos(heading[i]);
      double sin = Math.sin(heading[i]);
      if (field.clipsObb(spline.x()[i], spline.y()[i], cos, sin, halfX, halfY)) {
        return Optional.empty();
      }
    }

    double[] ds = new double[n - 1];
    Arrays.fill(ds, spline.sampleSpacing());
    double[] velocity =
        VelocityProfiler.profile(
            spline.curvature(), ds, req.startVelocity, req.endVelocity, config.constraints);

    return Optional.of(
        new GeneratedPath(spline.x(), spline.y(), heading, spline.curvature(), velocity));
  }

  /**
   * Convenience wrapper around {@link #generateOriented} that tries an inscribed-disc Theta* search
   * first (for the tight-corridor win) and falls back to a circumscribed-disc search if that
   * rejects.
   *
   * <p>Why a fallback: inscribed-disc Theta* finds tighter routes that the OBB at tangent can
   * thread when obstacle geometry is "flat" (parallel walls, corridor mouths). For obstacles the
   * path must curve around (point-like / circular), the tangent rotates as the path bends and the
   * OBB's corners can reach into the obstacle even though the disc clearance looks fine.
   * Circumscribed-disc planning prevents that case by giving every sample full corner clearance.
   */
  public static Optional<GeneratedPath> generateOrientedWithFallback(
      ObstacleField field,
      double minX,
      double minY,
      double maxX,
      double maxY,
      double cellSize,
      double plannerMargin,
      Request req,
      double halfX,
      double halfY,
      Config config) {
    double inscribed = Math.min(halfX, halfY);
    double circumscribed = Math.hypot(halfX, halfY);
    Costmap inscribedMap =
        Costmap.build(field, minX, minY, maxX, maxY, cellSize, inscribed, plannerMargin);
    Optional<GeneratedPath> tight =
        generateOriented(field, inscribedMap, req, halfX, halfY, config);
    if (tight.isPresent()) {
      return tight;
    }
    Costmap circumscribedMap =
        Costmap.build(field, minX, minY, maxX, maxY, cellSize, circumscribed, plannerMargin);
    return generateOriented(field, circumscribedMap, req, halfX, halfY, config);
  }

  /** How far the robot can rotate left or right of the path direction without clipping. */
  private record SlackBounds(double pos, double neg) {}

  /**
   * Sweeps rotations to find how far the robot can rotate left and right before the rectangle clips
   * an obstacle. We use a linear sweep (not binary search) because clipping can happen at weird
   * angles - e.g. clear at 0° and 90° but blocked at 45°.
   */
  private static SlackBounds computeHeadingSlackBounds(
      double cx, double cy, double tangentRad, double halfX, double halfY, ObstacleField field) {
    double cosT = Math.cos(tangentRad);
    double sinT = Math.sin(tangentRad);
    if (field.clipsObb(cx, cy, cosT, sinT, halfX, halfY)) {
      return new SlackBounds(0.0, 0.0);
    }
    double stepRad = Math.toRadians(1.0);
    double maxRad = Math.PI / 2;
    int steps = (int) Math.ceil(maxRad / stepRad);
    double posLimit = maxRad;
    double negLimit = maxRad;
    for (int i = 1; i <= steps; i++) {
      double off = i * stepRad;
      if (clipsAtOffset(cx, cy, tangentRad, off, halfX, halfY, field)) {
        posLimit = (i - 1) * stepRad;
        break;
      }
    }
    for (int i = 1; i <= steps; i++) {
      double off = i * stepRad;
      if (clipsAtOffset(cx, cy, tangentRad, -off, halfX, halfY, field)) {
        negLimit = (i - 1) * stepRad;
        break;
      }
    }
    // Back off by half a step so we stay safely inside the verified-clear range.
    double margin = 0.5 * stepRad;
    return new SlackBounds(Math.max(0.0, posLimit - margin), Math.max(0.0, negLimit - margin));
  }

  /** For each spot, takes the smallest value within w cells of it (smooths down peaks). */
  private static double[] minWindow(double[] in, int w) {
    int n = in.length;
    double[] out = new double[n];
    for (int i = 0; i < n; i++) {
      double m = in[i];
      int lo = Math.max(0, i - w);
      int hi = Math.min(n - 1, i + w);
      for (int j = lo; j <= hi; j++) {
        if (in[j] < m) m = in[j];
      }
      out[i] = m;
    }
    return out;
  }

  private static boolean clipsAtOffset(
      double cx,
      double cy,
      double tangentRad,
      double offset,
      double halfX,
      double halfY,
      ObstacleField field) {
    double a = tangentRad + offset;
    return field.clipsObb(cx, cy, Math.cos(a), Math.sin(a), halfX, halfY);
  }
}
