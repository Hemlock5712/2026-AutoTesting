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

  // private static void buildFlywheel() {
  //   flywheelMap.put(0.0, 20.0);
  //   flywheelMap.put(1.0, 20.0);
  //   flywheelMap.put(1.5, 26.0);
  //   flywheelMap.put(2.0, 30.0);
  //   flywheelMap.put(2.5, 29.0);
  //   flywheelMap.put(3.0, 30.0);
  //   flywheelMap.put(3.5, 32.0);
  //   flywheelMap.put(4.0, 35.0);
  //   flywheelMap.put(4.5, 36.0);
  //   flywheelMap.put(5.0, 40.0);
  // }

  private static void buildFlywheel() {
    flywheelMap.put(0.0, 20.0);
    flywheelMap.put(1.0, 20.0);
    flywheelMap.put(1.5, 22.0);
    flywheelMap.put(2.0, 26.0);
    flywheelMap.put(2.5, 27.0);
    flywheelMap.put(3.0, 29.0);
    flywheelMap.put(3.5, 32.0);
    flywheelMap.put(4.0, 35.0);
    flywheelMap.put(4.5, 38.0);
    flywheelMap.put(5.0, 40.0);
    flywheelMap.put(5.5, 43.0);
  }

  private static void buildHood() {
    hoodMap.put(0.0, 0.00);
    hoodMap.put(1.0, 0.00);
    hoodMap.put(1.5, 3.0);
    hoodMap.put(2.0, 5.0);
    hoodMap.put(2.5, 9.0);
    hoodMap.put(3.0, 11.0);
    hoodMap.put(3.5, 12.0);
    hoodMap.put(4.0, 14.0);
    hoodMap.put(4.5, 15.5);
    hoodMap.put(5.0, 16.0);
    hoodMap.put(5.5, 17.0);
  }

  private static void buildToF() {
    tofMap.put(0.0, 0.8667);
    tofMap.put(1.0, 0.8667);
    tofMap.put(1.5, 0.8667);
    tofMap.put(2.0, 0.9667);
    tofMap.put(2.5, 0.9556);
    tofMap.put(3.0, 0.9833);
    tofMap.put(3.5, 1.100);
    tofMap.put(4.0, 1.1389);
    tofMap.put(4.5, 1.2167);
    tofMap.put(5.0, 1.2667);
    tofMap.put(5.5, 1.3278);
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
