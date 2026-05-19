package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Field2026Constants.Hub;
import frc.robot.Field2026Constants.LeftBump;
import frc.robot.Field2026Constants.RightBump;
import frc.robot.Field2026Constants.Tower;
import frc.robot.utils.FieldInfo;
import java.util.ArrayList;
import java.util.List;

/**
 * 2026 field obstacles, blue + rotate-symmetric red. Each obstacle is an axis-aligned bounding box
 * stored as a {@code (min, max)} Translation2d pair — PathPlanner's pathfinder, the dyn4j sim, and
 * the AdvantageScope visualizer all consume this format directly. Includes perimeter, Hub, Tower
 * body wings, and floor bumps. Skipped: Trench (drive-under), Outpost / Depot (covered by
 * perimeter), FuelPool (scoring region, not solid).
 */
public final class Field2026Obstacles {

  /** Thickness of the perimeter wall slabs (m). */
  public static final double PERIMETER_THICKNESS_M = 0.10;

  private Field2026Obstacles() {}

  public static List<Pair<Translation2d, Translation2d>> build() {
    List<Pair<Translation2d, Translation2d>> field = new ArrayList<>();
    double L = FieldInfo.lengthMeters();
    double W = FieldInfo.widthMeters();
    double t = PERIMETER_THICKNESS_M;

    // Perimeter walls — shared (not mirrored).
    field.add(aabb(-t, -t, L + t, 0.0));
    field.add(aabb(-t, W, L + t, W + t));
    field.add(aabb(-t, 0.0, 0.0, W));
    field.add(aabb(L, 0.0, L + t, W));

    // Blue-side structures + rotate-mirrored red copies.
    for (Pair<Translation2d, Translation2d> blue : blueAllianceStructures(W)) {
      field.add(blue);
      field.add(rotateMirror(blue, L, W));
    }
    return field;
  }

  /** Hub footprint, Tower body wings, and floor bumps for the blue alliance. */
  private static List<Pair<Translation2d, Translation2d>> blueAllianceStructures(double W) {
    List<Pair<Translation2d, Translation2d>> list = new ArrayList<>();

    // Hub — 47" square centered at (Hub.centerX, W/2).
    double hubHalf = Hub.width / 2.0;
    double hubCx = Hub.nearLeftCorner.getX() + hubHalf;
    double hubCy = W / 2.0;
    list.add(aabb(hubCx - hubHalf, hubCy - hubHalf, hubCx + hubHalf, hubCy + hubHalf));

    addTowerWings(list);

    // Floor bumps — bounding box of each bump's four corner translations.
    list.add(
        cornerAabb(
            LeftBump.nearLeftCorner,
            LeftBump.nearRightCorner,
            LeftBump.farLeftCorner,
            LeftBump.farRightCorner));
    list.add(
        cornerAabb(
            RightBump.nearLeftCorner,
            RightBump.nearRightCorner,
            RightBump.farLeftCorner,
            RightBump.farRightCorner));

    return list;
  }

  /**
   * Two rectangles flanking the Tower's central opening, leaving the opening drivable.
   *
   * <p>Tight-corridor caveat: opening half-width is ~0.410 m and the bumpered robot half-Y is
   * ~0.359 m, leaving ~0.051 m of clearance per side. PathPlanner inflates obstacles by the robot
   * half-extent + {@code DrivePhysics.PATH_INFLATION_MARGIN_M} (0.10 m), so the pathfinder will
   * refuse to route through the Tower opening. Driving through requires straight-line teleop with
   * the robot centered — manual approach only.
   */
  private static void addTowerWings(List<Pair<Translation2d, Translation2d>> list) {
    double towerBack = Math.max(0.0, Tower.frontFaceX - Tower.depth);
    double centerY = (Tower.leftUpright.getY() + Tower.rightUpright.getY()) / 2.0;
    double outerHalf = Tower.width / 2.0;
    double openingHalf = Tower.innerOpeningWidth / 2.0;
    list.add(aabb(towerBack, centerY + openingHalf, Tower.frontFaceX, centerY + outerHalf));
    list.add(aabb(towerBack, centerY - outerHalf, Tower.frontFaceX, centerY - openingHalf));
  }

  /**
   * Reflects an axis-aligned bounding box through the field-center rotation point {@code (L/2,
   * W/2)}. A 180° rotation of an AABB is still an AABB; the two corners just swap.
   */
  private static Pair<Translation2d, Translation2d> rotateMirror(
      Pair<Translation2d, Translation2d> box, double L, double W) {
    Translation2d min = box.getFirst();
    Translation2d max = box.getSecond();
    // (x, y) → (L - x, W - y). The original min becomes the new max and vice versa.
    return Pair.of(
        new Translation2d(L - max.getX(), W - max.getY()),
        new Translation2d(L - min.getX(), W - min.getY()));
  }

  /** AABB from min/max coordinates. Validates ordering so a transposed call fails loudly. */
  private static Pair<Translation2d, Translation2d> aabb(
      double minX, double minY, double maxX, double maxY) {
    if (minX >= maxX || minY >= maxY) {
      throw new IllegalArgumentException(
          "AABB bounds must be ordered (min < max): got ("
              + minX
              + ", "
              + minY
              + ") .. ("
              + maxX
              + ", "
              + maxY
              + ")");
    }
    return Pair.of(new Translation2d(minX, minY), new Translation2d(maxX, maxY));
  }

  /** Bounding box of an arbitrary bag of points. */
  private static Pair<Translation2d, Translation2d> cornerAabb(Translation2d... corners) {
    double minX = Double.POSITIVE_INFINITY;
    double maxX = Double.NEGATIVE_INFINITY;
    double minY = Double.POSITIVE_INFINITY;
    double maxY = Double.NEGATIVE_INFINITY;
    for (Translation2d c : corners) {
      if (c.getX() < minX) minX = c.getX();
      if (c.getX() > maxX) maxX = c.getX();
      if (c.getY() < minY) minY = c.getY();
      if (c.getY() > maxY) maxY = c.getY();
    }
    return aabb(minX, minY, maxX, maxY);
  }
}
