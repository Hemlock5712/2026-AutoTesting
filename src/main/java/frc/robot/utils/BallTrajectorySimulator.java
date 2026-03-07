package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;

/**
 * Simulates ball trajectory using projectile motion with air drag.
 *
 * <p>This is a game-agnostic physics simulator that takes ball properties and initial conditions,
 * then calculates the trajectory using Euler integration with drag forces.
 */
public class BallTrajectorySimulator {
  private static final double GRAVITY = 9.81; // m/s²
  private static final double AIR_DENSITY = 1.225; // kg/m³ at sea level

  private final double ballMass; // kg
  private final double ballRadius; // m
  private final double dragCoefficient;
  private final double crossSectionalArea; // m²

  /**
   * Creates a new ball trajectory simulator with the specified ball properties.
   *
   * @param ballMass Mass of the ball in kilograms
   * @param ballDiameter Diameter of the ball in meters
   * @param dragCoefficient Drag coefficient (dimensionless, typically 0.4-0.5 for spheres)
   */
  public BallTrajectorySimulator(double ballMass, double ballDiameter, double dragCoefficient) {
    this.ballMass = ballMass;
    this.ballRadius = ballDiameter / 2.0;
    this.dragCoefficient = dragCoefficient;
    this.crossSectionalArea = Math.PI * ballRadius * ballRadius;
  }

  /**
   * Simulates the ball trajectory from launch position and velocity.
   *
   * @param launchPos Initial launch position in field coordinates (meters)
   * @param launchVel Initial launch velocity vector in field coordinates (m/s)
   * @param timestep Time step for simulation in seconds (e.g., 0.01)
   * @param maxTime Maximum simulation time in seconds
   * @return List of Pose3d points along the trajectory
   */
  public List<Pose3d> simulate(
      Translation3d launchPos, Translation3d launchVel, double timestep, double maxTime) {
    List<Pose3d> trajectory = new ArrayList<>();

    Translation3d position = launchPos;
    Translation3d velocity = launchVel;

    double time = 0.0;

    // Add initial position
    trajectory.add(new Pose3d(position, new Rotation3d()));

    while (time < maxTime && position.getZ() >= 0.0) {
      // Calculate drag force
      Translation3d dragForce = calculateDragForce(velocity);

      // Calculate acceleration (gravity + drag)
      Translation3d acceleration =
          new Translation3d(0.0, 0.0, -GRAVITY) // Gravity points down
              .plus(
                  new Translation3d(
                      dragForce.getX() / ballMass,
                      dragForce.getY() / ballMass,
                      dragForce.getZ() / ballMass)); // Drag acceleration

      // Euler integration: update velocity
      velocity =
          new Translation3d(
              velocity.getX() + acceleration.getX() * timestep,
              velocity.getY() + acceleration.getY() * timestep,
              velocity.getZ() + acceleration.getZ() * timestep);

      // Euler integration: update position
      position =
          new Translation3d(
              position.getX() + velocity.getX() * timestep,
              position.getY() + velocity.getY() * timestep,
              position.getZ() + velocity.getZ() * timestep);

      time += timestep;

      // Add point to trajectory
      trajectory.add(new Pose3d(position, new Rotation3d()));

      // Stop if ball hits ground
      if (position.getZ() <= 0.0) {
        break;
      }
    }

    return trajectory;
  }

  /**
   * Calculates the drag force vector based on velocity.
   *
   * <p>Drag force: F_drag = 0.5 * p * C_d * A * v²
   *
   * <p>The force opposes the direction of motion.
   *
   * @param velocity Current velocity vector (m/s)
   * @return Drag force vector (N)
   */
  private Translation3d calculateDragForce(Translation3d velocity) {
    double speed =
        Math.sqrt(
            velocity.getX() * velocity.getX()
                + velocity.getY() * velocity.getY()
                + velocity.getZ() * velocity.getZ());

    if (speed < 1e-6) {
      // Avoid division by zero for very small velocities
      return new Translation3d(0.0, 0.0, 0.0);
    }

    // Drag force magnitude: F = 0.5 * p * C_d * A * v²
    double dragMagnitude = 0.5 * AIR_DENSITY * dragCoefficient * crossSectionalArea * speed * speed;

    // Drag force opposes velocity direction
    double dragX = -(velocity.getX() / speed) * dragMagnitude;
    double dragY = -(velocity.getY() / speed) * dragMagnitude;
    double dragZ = -(velocity.getZ() / speed) * dragMagnitude;

    return new Translation3d(dragX, dragY, dragZ);
  }
}
