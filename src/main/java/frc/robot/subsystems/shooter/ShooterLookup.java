package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterLookup {
  private static final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

  static {
    buildFlywheel();
    buildHood();
  }

  private static void buildFlywheel() {
    flywheelMap.put(0.0, 20.0);
    flywheelMap.put(1.0, 20.0);
    flywheelMap.put(1.5, 24.0);
    flywheelMap.put(2.0, 26.0);
    flywheelMap.put(2.5, 28.0);
    flywheelMap.put(3.0, 29.5);
    flywheelMap.put(3.5, 31.5);
    flywheelMap.put(4.0, 34.0);
    flywheelMap.put(4.5, 37.0);
    flywheelMap.put(5.0, 40.0);
  }

  private static void buildHood() {
    hoodMap.put(0.0, 0.00);
    hoodMap.put(1.0, 0.00);
    hoodMap.put(1.5, 3.0);
    hoodMap.put(2.0, 5.0);
    hoodMap.put(2.5, 9.0);
    hoodMap.put(3.0, 10.0);
    hoodMap.put(3.5, 11.0);
    hoodMap.put(4.0, 13.0);
    hoodMap.put(4.0, 14.5);
    hoodMap.put(5.0, 15.00);
  }

  public static InterpolatingDoubleTreeMap getFlywheelMap() {
    return flywheelMap;
  }

  public static InterpolatingDoubleTreeMap getHoodMap() {
    return hoodMap;
  }
}
