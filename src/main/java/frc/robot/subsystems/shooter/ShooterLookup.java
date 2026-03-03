package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterLookup {
  private static final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

  public static void initialize() {
    buildFlywheel();
    buildHood();
  }

  private static void buildFlywheel() {
    flywheelMap.put(0.0, 20.0);
    flywheelMap.put(1.193, 20.0);
    flywheelMap.put(1.68, 30.0);
    flywheelMap.put(3.15, 35.0);
    flywheelMap.put(5.0, 50.0);
  }

  private static void buildHood() {
    hoodMap.put(0.0, 0.00);
    hoodMap.put(1.193, 0.00);
    hoodMap.put(3.15, 0.00);
    hoodMap.put(5.0, 0.00);
  }

  public static InterpolatingDoubleTreeMap getFlywheelMap() {
    return flywheelMap;
  }

  public static InterpolatingDoubleTreeMap getHoodMap() {
    return hoodMap;
  }
}
