package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterLookup {
  private static final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap feedShootMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap feedHoodMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap feedTime = new InterpolatingDoubleTreeMap();
  private static final double tofMult = 0.25;

  static {
    buildFlywheel();
    buildHood();
    buildToF();
    buildFeedFlywheel();
    buildFeedHood();
    buildFeedTime();
  }

  // private static void buildFlywheel() {
  //   flywheelMap.put(0.0, 23.0);
  //   flywheelMap.put(1.0, 23.0);
  //   flywheelMap.put(1.5, 26.0);
  //   flywheelMap.put(2.0, 28.0);
  //   flywheelMap.put(2.5, 29.0);
  //   flywheelMap.put(3.0, 32.0);
  //   flywheelMap.put(3.5, 34.0);
  //   flywheelMap.put(4.0, 36.0);
  //   flywheelMap.put(4.5, 39.0);
  //   flywheelMap.put(5.0, 44.0);
  //   flywheelMap.put(5.5, 45.0);
  // }

  private static void buildFlywheel() {
    flywheelMap.put(0.0, 23.0 + 2.0);
    flywheelMap.put(1.0, 23.0 + 2.0);
    flywheelMap.put(1.5, 26.0 + 2.0);
    flywheelMap.put(2.0, 30.0);
    flywheelMap.put(2.5, 30.0);
    flywheelMap.put(3.0, 30.0);
    flywheelMap.put(3.5, 33.0);
    flywheelMap.put(4.0, 38.0);
    flywheelMap.put(4.5, 40.0);
    flywheelMap.put(5.0, 46.5);
    flywheelMap.put(5.5, 45.0 + 6.0);
  }

  private static void buildHood() {
    hoodMap.put(0.0, 0.00);
    hoodMap.put(1.0, 0.00);
    hoodMap.put(1.5, 3.0);
    hoodMap.put(2.0, 10.0);
    hoodMap.put(2.5, 12.0);
    hoodMap.put(3.0, 15.0);
    hoodMap.put(3.5, 18.0);
    hoodMap.put(4.0, 20.0);
    hoodMap.put(4.5, 24.0);
    hoodMap.put(5.0, 25.0);
    hoodMap.put(5.5, 25.0);
  }

  private static void buildToF() {
    tofMap.put(0.0, 0.890);
    tofMap.put(1.0, 0.890);
    tofMap.put(1.5, 0.890);
    tofMap.put(2.0, 0.890);
    tofMap.put(2.5, 0.900);
    tofMap.put(3.0, 0.984);
    tofMap.put(3.5, 1.060);
    tofMap.put(4.0, 0.97);
    tofMap.put(4.5, 0.96);
    tofMap.put(5.0, 0.90);
    tofMap.put(5.5, 0.90);
  }

  private static void buildFeedFlywheel() {
    feedShootMap.put(0.0, 20.0);
    feedShootMap.put(9.5, 52.0);
  }

  private static void buildFeedHood() {
    feedHoodMap.put(0.0, 0.00);
    feedHoodMap.put(9.5, 18.0);
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
