package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterLookup {
  private static final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

  static {
    buildFlywheel();
    buildHood();
    buildToF();
  }

  private static void buildFlywheel() {
    flywheelMap.put(0.0, 20.0);
    flywheelMap.put(1.0, 20.0);
    flywheelMap.put(1.5, 26.0);
    flywheelMap.put(2.0, 30.0);
    flywheelMap.put(2.5, 29.0);
    flywheelMap.put(3.0, 30.0);
    flywheelMap.put(3.5, 32.0);
    flywheelMap.put(4.0, 35.0);
    flywheelMap.put(4.5, 36.0);
    flywheelMap.put(5.0, 40.0);
  }

  private static void buildHood() {
    hoodMap.put(0.0, 0.00);
    hoodMap.put(1.0, 0.00);
    hoodMap.put(1.5, 3.0);
    hoodMap.put(2.0, 5.0);
    hoodMap.put(2.5, 9.0);
    hoodMap.put(3.0, 12.0);
    hoodMap.put(3.5, 13.0);
    hoodMap.put(4.0, 14.0);
    hoodMap.put(4.5, 15.5);
    hoodMap.put(5.0, 18.0);
  }

  private static void buildToF() {
    tofMap.put(0.0, 0.983);
    tofMap.put(1.0, 0.983);
    tofMap.put(1.5, 0.983);
    tofMap.put(2.0, 1.117);
    tofMap.put(2.5, 0.967);
    tofMap.put(3.0, 1.00);
    tofMap.put(3.5, 1.00);
    tofMap.put(4.0, 1.167);
    tofMap.put(4.5, 1.183);
    tofMap.put(5.0, 1.25);
  }

  public static InterpolatingDoubleTreeMap getFlywheelMap() {
    return flywheelMap;
  }

  public static InterpolatingDoubleTreeMap getHoodMap() {
    return hoodMap;
  }

  public static InterpolatingDoubleTreeMap getToFMap() {
    return tofMap;
  }
}
