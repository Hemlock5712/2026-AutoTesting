package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterLookup {
  private static final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap feedShootMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap feedHoodMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap feedTime = new InterpolatingDoubleTreeMap();

  static {
    buildFlywheel();
    buildHood();
    buildToF();
    buildFeedFlywheel();
    buildFeedHood();
    buildFeedTime();
  }

  private static void buildFlywheel() {
    flywheelMap.put(0.0, 23.0);
    flywheelMap.put(1.0, 23.0);
    flywheelMap.put(1.5, 25.0);
    flywheelMap.put(2.0, 28.0);
    flywheelMap.put(2.5, 29.0);
    flywheelMap.put(3.0, 31.0);
    flywheelMap.put(3.5, 33.0);
    flywheelMap.put(4.0, 34.0);
    flywheelMap.put(4.5, 37.0);
    flywheelMap.put(5.0, 39.0);
    flywheelMap.put(5.5, 42.0);
  }

  private static void buildHood() {
    hoodMap.put(0.0, 0.00);
    hoodMap.put(1.0, 0.00);
    hoodMap.put(1.5, 3.0);
    hoodMap.put(2.0, 5.0);
    hoodMap.put(2.5, 9.0);
    hoodMap.put(3.0, 11.0);
    hoodMap.put(3.5, 13.0);
    hoodMap.put(4.0, 15.0);
    hoodMap.put(4.5, 15.0);
    hoodMap.put(5.0, 16.0);
    hoodMap.put(5.5, 19.0);
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

  private static void buildFeedFlywheel() {
    feedShootMap.put(0.0, 20.0);
    feedShootMap.put(9.5, 50.0);
  }

  private static void buildFeedHood() {
    feedHoodMap.put(0.0, 0.00);
    feedHoodMap.put(9.5, 20.0);
  }

  private static void buildFeedTime() {
    feedTime.put(0.0, 0.00);
    feedTime.put(9.5, 1.67);
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

  public static InterpolatingDoubleTreeMap getFeedFlywheelMap() {
    return feedShootMap;
  }

  public static InterpolatingDoubleTreeMap getFeedHoodMap() {
    return feedHoodMap;
  }

  public static InterpolatingDoubleTreeMap getFeedTimeMap() {
    return feedTime;
  }
}
