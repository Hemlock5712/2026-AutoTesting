package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterLookup {
  private static final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap feedShootMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap feedHoodMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap feedTime = new InterpolatingDoubleTreeMap();
  private static final double tofMult = 0.70;

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
    flywheelMap.put(2.0, 25.0);
    flywheelMap.put(2.5, 28.0);
    flywheelMap.put(3.0, 30.0);
    flywheelMap.put(3.5, 34.0);
    flywheelMap.put(4.0, 35.0);
    flywheelMap.put(4.5, 37.0);
    flywheelMap.put(5.0, 41.0);
    flywheelMap.put(5.5, 41.0);
    flywheelMap.put(6.0, 44.5);
    flywheelMap.put(6.5, 48.0);
    // flywheelMap.put(7.0, 65.0);
  }

  private static void buildHood() {
    hoodMap.put(0.0, 0.00);
    hoodMap.put(1.0, 0.00);
    hoodMap.put(1.5, 0.0);
    hoodMap.put(2.0, 7.0);
    hoodMap.put(2.5, 10.0);
    hoodMap.put(3.0, 14.0);
    hoodMap.put(3.5, 17.0);
    hoodMap.put(4.0, 20.0);
    hoodMap.put(4.5, 25.0);
    hoodMap.put(5.0, 27.0);
    hoodMap.put(5.5, 32.0);
    hoodMap.put(6.0, 32.0);
    hoodMap.put(6.5, 32.0);
    // hoodMap.put(7.0, 32.0);
  }

  private static void buildToF() {
    tofMap.put(0.0, 1.24 * tofMult);
    tofMap.put(1.0, 1.24 * tofMult);
    tofMap.put(1.5, 1.24 * tofMult);
    tofMap.put(2.0, 1.059 * tofMult);
    tofMap.put(2.5, 1.093 * tofMult);
    tofMap.put(3.0, 1.082 * tofMult);
    tofMap.put(3.5, 1.097 * tofMult);
    tofMap.put(4.0, 1.104 * tofMult);
    tofMap.put(4.5, 1.056 * tofMult);
    tofMap.put(5.0, 1.079 * tofMult);
    tofMap.put(5.5, 1.027 * tofMult);
    tofMap.put(6.0, 1.086 * tofMult);
    tofMap.put(6.5, 1.142 * tofMult);
    // tofMap.put(7.0, 1.6);
  }

  private static void buildFeedFlywheel() {
    feedShootMap.put(0.0, 20.0);
    feedShootMap.put(10.0, 45.0);
    feedShootMap.put(15.0, 52.0);
    feedShootMap.put(25.0, 57.0);
  }

  private static void buildFeedHood() {
    feedHoodMap.put(0.0, 32.0);
    feedHoodMap.put(10.0, 32.0);
    feedHoodMap.put(15.0, 32.0);
    feedHoodMap.put(25.0, 32.0);
  }

  private static void buildFeedTime() {
    feedTime.put(0.0, 0.00);
    feedTime.put(10.0, 1.0);
    feedTime.put(15.0, 1.0);
    feedTime.put(25.0, 1.0);
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
