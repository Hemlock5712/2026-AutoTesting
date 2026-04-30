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

  // ===================================
  // =           HOME VALUES           =
  // ===================================
  // private static void buildFlywheel() {
  //   flywheelMap.put(0.0, 21.0);
  //   flywheelMap.put(1.0, 22.0);
  //   flywheelMap.put(1.5, 24.0);
  //   flywheelMap.put(2.0, 25.0);
  //   flywheelMap.put(2.5, 27.0);
  //   flywheelMap.put(3.0, 28.0);
  //   flywheelMap.put(3.5, 30.0);
  //   flywheelMap.put(4.0, 33.0);
  //   flywheelMap.put(4.5, 36.0);
  //   flywheelMap.put(5.0, 39.0);
  //   flywheelMap.put(5.5, 40.0);
  // }

  // private static void buildHood() {
  //   hoodMap.put(0.0, 0.00);
  //   hoodMap.put(1.0, 0.00);
  //   hoodMap.put(1.5, 3.0);
  //   hoodMap.put(2.0, 5.0);
  //   hoodMap.put(2.5, 9.0);
  //   hoodMap.put(3.0, 11.0);
  //   hoodMap.put(3.5, 13.0);
  //   hoodMap.put(4.0, 14.0);
  //   hoodMap.put(4.5, 14.5);
  //   hoodMap.put(5.0, 16.0);
  //   hoodMap.put(5.5, 19.0);
  // }

  // private static void buildToF() {
  //   tofMap.put(0.0, 0.889 - tofMult);
  //   tofMap.put(1.0, 0.889 - tofMult);
  //   tofMap.put(1.5, 0.889 - tofMult);
  //   tofMap.put(2.0, 0.933 - tofMult);
  //   tofMap.put(2.5, 0.983 - tofMult);
  //   tofMap.put(3.0, 1.044 - tofMult);
  //   tofMap.put(3.5, 1.044 - tofMult);
  //   tofMap.put(4.0, 1.017 - tofMult);
  //   tofMap.put(4.5, 1.161 - tofMult);
  //   tofMap.put(5.0, 1.211 - tofMult);
  //   tofMap.put(5.5, 1.333 - tofMult);
  // }

  // ===================================
  // =       COMPETITION VALUES        =
  // ===================================
  private static void buildFlywheel() {
    flywheelMap.put(0.0, 25.0);
    flywheelMap.put(1.0, 25.0);
    flywheelMap.put(1.5, 28.0);
    flywheelMap.put(2.0, 27.0);
    flywheelMap.put(2.5, 28.0);
    flywheelMap.put(3.0, 29.5);
    flywheelMap.put(3.5, 34.0);
    flywheelMap.put(4.0, 35.0);
    flywheelMap.put(4.5, 37.0);
    flywheelMap.put(5.0, 41.0);
    flywheelMap.put(5.5, 47.0);
  }

  private static void buildHood() {
    hoodMap.put(0.0, 0.00);
    hoodMap.put(1.0, 0.00);
    hoodMap.put(1.5, 0.0);
    hoodMap.put(2.0, 7.0);
    hoodMap.put(2.5, 10.0);
    hoodMap.put(3.0, 13.5);
    hoodMap.put(3.5, 17.0);
    hoodMap.put(4.0, 20.0);
    hoodMap.put(4.5, 25.0);
    hoodMap.put(5.0, 27.0);
    hoodMap.put(5.5, 27.0);
  }

  private static void buildToF() {
    tofMap.put(0.0, 1.24);
    tofMap.put(1.0, 1.24);
    tofMap.put(1.5, 1.24);
    tofMap.put(2.0, 1.059);
    tofMap.put(2.5, 1.093);
    tofMap.put(3.0, 1.098);
    tofMap.put(3.5, 1.097);
    tofMap.put(4.0, 1.104);
    tofMap.put(4.5, 1.056);
    tofMap.put(5.0, 1.079);
    tofMap.put(5.5, 1.027);
  }

  private static void buildFeedFlywheel() {
    feedShootMap.put(0.0, 20.0);
    feedShootMap.put(10.0, 45.0);
    feedShootMap.put(12.0, 50.0);
  }

  private static void buildFeedHood() {
    feedHoodMap.put(0.0, 32.0);
    feedHoodMap.put(10.0, 32.0);
    feedHoodMap.put(12.0, 32.0);
  }

  private static void buildFeedTime() {
    feedTime.put(0.0, 0.00);
    feedTime.put(10.0, 1.67);
    feedTime.put(12.0, 2.0);
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
