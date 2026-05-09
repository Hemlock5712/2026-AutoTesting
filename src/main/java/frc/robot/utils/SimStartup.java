package frc.robot.utils;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

/**
 * Auto-starts the sim in auto or teleop mode without anyone clicking buttons. Used for headless
 * runs (e.g. CI). Set {@code -Pmode=auto} or {@code -Pmode=teleop} when building.
 */
public final class SimStartup {

  private SimStartup() {}

  public static void arm() {
    if (!RobotBase.isSimulation()) {
      return;
    }
    String mode = System.getProperty("frc.sim.startMode", "").toLowerCase();
    if (mode.isEmpty()) {
      return;
    }

    boolean autonomous;
    if (mode.equals("auto") || mode.equals("autonomous")) {
      autonomous = true;
    } else if (mode.equals("teleop")) {
      autonomous = false;
    } else {
      System.err.println("[SimStartup] Unknown frc.sim.startMode=" + mode + " — ignoring");
      return;
    }

    DriverStationSim.setAutonomous(autonomous);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
    DriverStationJNI.observeUserProgramStarting();

    System.out.println(
        "[SimStartup] Headless start: enabled=true autonomous="
            + autonomous
            + " (mode="
            + mode
            + ")");
  }
}
