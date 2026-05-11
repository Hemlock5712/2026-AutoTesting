package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.FieldInfo.Hub;
import frc.robot.utils.FieldInfo.LeftBump;
import frc.robot.utils.FieldInfo.RightBump;
import frc.robot.utils.FieldInfo.Tower;
import frc.robot.utils.path.Obstacle;
import frc.robot.utils.path.ObstacleField;
import java.util.ArrayList;
import java.util.List;

/**
 * 2026 field obstacles, blue + rotate-symmetric red. Single source of truth consumed by the dyn4j
 * sim, the avoidance clamp, the planner, and the visualizer. Includes perimeter, Hub, Tower body
 * wings, and floor bumps. Skipped: Trench (drive-under), Outpost / Depot (covered by perimeter),
 * FuelPool (scoring region, not solid).
 */
public final class Field2026Obstacles {

  /** Thickness of the perimeter wall slabs (m). */
  public static final double PERIMETER_THICKNESS_M = 0.10;

  private Field2026Obstacles() {}

  public static ObstacleField build() {
    ObstacleField field = new ObstacleField();
    double L = FieldInfo.lengthMeters();
    double W = FieldInfo.widthMeters();
    double t = PERIMETER_THICKNESS_M;

    // Perimeter walls — shared (not mirrored).
    field.addStatic(new Obstacle.Rectangle(-t, -t, L + t, 0.0));
    field.addStatic(new Obstacle.Rectangle(-t, W, L + t, W + t));
    field.addStatic(new Obstacle.Rectangle(-t, 0.0, 0.0, W));
    field.addStatic(new Obstacle.Rectangle(L, 0.0, L + t, W));

    // Blue-side structures + rotate-mirrored red copies.
    for (Obstacle.Rectangle blue : blueAllianceStructures(W)) {
      field.addStatic(blue);
      field.addStatic(rotateMirror(blue, L, W));
    }
    return field;
  }

  /** Hub footprint, Tower body wings, and floor bumps for the blue alliance. */
  private static List<Obstacle.Rectangle> blueAllianceStructures(double W) {
    List<Obstacle.Rectangle> list = new ArrayList<>();

    // Hub — 47" square centered at (Hub.centerX, W/2).
    double hubHalf = Hub.width / 2.0;
    double hubCx = Hub.nearLeftCorner.getX() + hubHalf;
    double hubCy = W / 2.0;
    list.add(rect(hubCx - hubHalf, hubCy - hubHalf, hubCx + hubHalf, hubCy + hubHalf));

    addTowerWings(list);

    // Floor bumps — AABB of each bump's four corner translations.
    list.add(
        aabb(
            LeftBump.nearLeftCorner,
            LeftBump.nearRightCorner,
            LeftBump.farLeftCorner,
            LeftBump.farRightCorner));
    list.add(
        aabb(
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
   * ~0.359 m, leaving ~0.051 m of clearance per side. That's LESS than {@code
   * Drive.AVOIDANCE_SAFETY_MARGIN_M} (0.10 m), so the clamp's brake curve says {@code vMax=0}
   * anywhere inside the corridor. Pure forward motion is unaffected (approach onto each wing normal
   * is ~0), but any lateral drift or non-axis-aligned heading will see the clamp pin the chassis
   * against the wing. Traversing the Tower opening therefore requires near-centerline pose with
   * heading aligned to the corridor axis — drivers can hold the avoidance-override bumper to cross
   * manually, or a future change can give Tower wings a per-obstacle margin override.
   */
  private static void addTowerWings(List<Obstacle.Rectangle> list) {
    double towerBack = Math.max(0.0, Tower.frontFaceX - Tower.depth);
    double centerY = (Tower.leftUpright.getY() + Tower.rightUpright.getY()) / 2.0;
    double outerHalf = Tower.width / 2.0;
    double openingHalf = Tower.innerOpeningWidth / 2.0;
    list.add(rect(towerBack, centerY + openingHalf, Tower.frontFaceX, centerY + outerHalf));
    list.add(rect(towerBack, centerY - outerHalf, Tower.frontFaceX, centerY - openingHalf));
  }

  /** Reflects a rectangle through the field-center rotation point {@code (L/2, W/2)}. */
  private static Obstacle.Rectangle rotateMirror(Obstacle.Rectangle r, double L, double W) {
    return new Obstacle.Rectangle(L - r.maxX(), W - r.maxY(), L - r.minX(), W - r.minY());
  }

  private static Obstacle.Rectangle rect(double minX, double minY, double maxX, double maxY) {
    return new Obstacle.Rectangle(minX, minY, maxX, maxY);
  }

  /** AABB of a bag of points. */
  private static Obstacle.Rectangle aabb(Translation2d... corners) {
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
    return new Obstacle.Rectangle(minX, minY, maxX, maxY);
  }
}
