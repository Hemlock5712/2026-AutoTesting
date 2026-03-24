package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

/**
 * Tracks whether our alliance's hub is currently active based on the shift schedule. Call {@link
 * #initialize()} at the start of teleop and {@link #update()} every robot periodic loop.
 */
public class HubShiftUtil {
  private static final Timer timer = new Timer();

  // Shift boundaries (seconds into teleop)
  // Defines 6 shifts: TRANSITION(0-10), SHIFT1(10-35), SHIFT2(35-60),
  //                    SHIFT3(60-85), SHIFT4(85-110), ENDGAME(110-140)
  private static final double[] shiftBoundaries = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

  // Whether each shift is active for the alliance that goes first
  private static final boolean[] firstActiveSchedule = {true, true, false, true, false, true};
  private static final boolean[] firstInactiveSchedule = {true, false, true, false, true, true};

  private static boolean hubActive = false;

  public static void setupNTValues() {
    Robot.telemetry().log("Hub/HubActive", true);
    Robot.telemetry().log("Hub/TimeUntilShift", 10.0);
    Robot.telemetry().log("Hub/TimeUntilShiftHumanDisplay", "10.0");
  }

  /** Starts the timer. Call at the beginning of teleop. */
  public static void initialize() {
    timer.restart();
  }

  /** Updates the cached active state. Call from robotPeriodic. */
  public static void update() {
    if (!DriverStation.isTeleopEnabled() && !DriverStation.isTestEnabled()) {
      hubActive = DriverStation.isAutonomousEnabled();
      return;
    }

    boolean[] schedule = getSchedule();
    int shiftIndex = getShiftIndex(timer.get());
    hubActive = schedule[shiftIndex];
    Robot.telemetry().log("Hub/HubActive", hubActive);
    Robot.telemetry().log("Hub/TimeUntilShift", getSecondsUntilNextShift());
    Robot.telemetry()
        .log("Hub/TimeUntilShiftHumanDisplay", String.format("%02.1f", getSecondsUntilNextShift()));
  }

  /** Returns whether our hub is currently active. */
  public static boolean isHubActive() {
    return hubActive;
  }

  /**
   * Returns the number of seconds until the next shift boundary, or {@code Double.MAX_VALUE} if
   * past the last boundary.
   */
  public static double getSecondsUntilNextShift() {
    double time = timer.get();
    int nextBoundaryIndex = getShiftIndex(time) + 1;
    if (nextBoundaryIndex >= shiftBoundaries.length) {
      return Double.MAX_VALUE;
    }
    return shiftBoundaries[nextBoundaryIndex] - time;
  }

  // --- Internal helpers ---

  private static int getShiftIndex(double time) {
    for (int i = 0; i < shiftBoundaries.length - 1; i++) {
      if (time >= shiftBoundaries[i] && time < shiftBoundaries[i + 1]) {
        return i;
      }
    }
    // Past last boundary, assume endgame
    return shiftBoundaries.length - 2;
  }

  private static boolean[] getSchedule() {
    Alliance firstActive = getFirstActiveAlliance();
    Alliance ours = DriverStation.getAlliance().orElse(Alliance.Blue);
    return (firstActive == ours) ? firstActiveSchedule : firstInactiveSchedule;
  }

  static Alliance getFirstActiveAlliance() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Check FMS game data
    String message = DriverStation.getGameSpecificMessage();
    if (!message.isEmpty()) {
      char ch = message.charAt(0);
      if (ch == 'R') return Alliance.Blue;
      if (ch == 'B') return Alliance.Red;
    }

    // Default: opposite alliance goes first
    return (alliance == Alliance.Blue) ? Alliance.Red : Alliance.Blue;
  }
}
