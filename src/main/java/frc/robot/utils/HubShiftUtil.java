package frc.robot.utils;

import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.Timer;

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
    Logger.recordOutput("Hub/HubActive", true);
    Logger.recordOutput("Hub/TimeUntilShift", 10.0);
    Logger.recordOutput("Hub/TimeUntilShiftHumanDisplay", 10.0);
  }

  /** Starts the timer. Call at the beginning of teleop. */
  public static void initialize() {
    timer.restart();
  }

  /** Updates the cached active state. Call from robotPeriodic. */
  public static void update() {
    if (!RobotState.isTeleopEnabled()) {
      hubActive = RobotState.isAutonomousEnabled();
      return;
    }

    double time = timer.get();
    boolean[] schedule = getSchedule();
    int shiftIndex = getShiftIndex(time);
    hubActive = schedule[shiftIndex];

    int nextBoundaryIndex = shiftIndex + 1;
    double timeUntilShift =
        (nextBoundaryIndex < shiftBoundaries.length)
            ? shiftBoundaries[nextBoundaryIndex] - time
            : Double.MAX_VALUE;

    Logger.recordOutput("Hub/HubActive", hubActive);
    Logger.recordOutput("Hub/TimeUntilShift", timeUntilShift);
    Logger.recordOutput("Hub/TimeUntilShiftHumanDisplay", timeUntilShift);
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
    Alliance ours = MatchState.getAlliance().orElse(Alliance.BLUE);
    return (firstActive == ours) ? firstActiveSchedule : firstInactiveSchedule;
  }

  static Alliance getFirstActiveAlliance() {
    Alliance alliance = MatchState.getAlliance().orElse(Alliance.BLUE);

    // Check FMS game data
    String message = MatchState.getGameData().orElse("");
    if (!message.isEmpty()) {
      char ch = message.charAt(0);
      if (ch == 'R') return Alliance.BLUE;
      if (ch == 'B') return Alliance.RED;
    }

    // Default: opposite alliance goes first
    return (alliance == Alliance.BLUE) ? Alliance.RED : Alliance.BLUE;
  }
}
