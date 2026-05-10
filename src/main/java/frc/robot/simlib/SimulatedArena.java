package frc.robot.simlib;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.simlib.drivesims.AbstractDriveTrainSimulation;
import frc.robot.simlib.motorsims.SimulatedBattery;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.dyn4j.dynamics.Body;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.World;

/**
 * Trimmed-down arena focused on drive-train + battery sub-tick. The original maple-sim arena
 * carried game-piece, scoring, match-clock, intake-collision, and field-mirroring infrastructure
 * for FRC season-specific simulations; we vendored only the drive-sim slice and stripped the rest.
 *
 * <p>Public API kept (all callers in this project rely on these):
 *
 * <ul>
 *   <li>{@link #getInstance()} — single shared default instance
 *   <li>{@link #simulationPeriodic()} — call once per robot period
 *   <li>{@link #addDriveTrainSimulation(AbstractDriveTrainSimulation)}
 *   <li>{@link #addCustomSimulation(Simulatable)}
 *   <li>{@link #getSimulationDt()}, {@link #getSimulationSubTicksIn1Period()}
 *   <li>{@link #overrideSimulationTimings(Time, int)}
 * </ul>
 */
public class SimulatedArena {
  /**
   * Whether to allow the simulation to run a real robot This feature is HIGHLY RECOMMENDED to be
   * turned OFF
   */
  public static boolean ALLOW_CREATION_ON_REAL_ROBOT = false;

  private static SimulatedArena instance = null;

  /**
   * Gets/creates the default simulation world. The trimmed arena has no obstacles, no game pieces,
   * and no scoring — it only ticks {@link AbstractDriveTrainSimulation}s, {@link Simulatable}
   * customs, and the {@link SimulatedBattery}.
   */
  public static SimulatedArena getInstance() {
    if (RobotBase.isReal() && (!ALLOW_CREATION_ON_REAL_ROBOT))
      throw new IllegalStateException(
          "MapleSim is running on a real robot! (If you would actually want that, set "
              + "SimulatedArena.ALLOW_CREATION_ON_REAL_ROBOT to true).");
    if (instance == null) instance = new SimulatedArena();
    return instance;
  }

  /** Overrides the singleton returned by {@link #getInstance()}. */
  public static void overrideInstance(SimulatedArena newInstance) {
    instance = newInstance;
  }

  /** The number of sub-ticks the simulator will run in each robot period. */
  private static int SIMULATION_SUB_TICKS_IN_1_PERIOD = 5;

  public static int getSimulationSubTicksIn1Period() {
    return SIMULATION_SUB_TICKS_IN_1_PERIOD;
  }

  /** The period length of each sub-tick, in seconds. */
  private static Time SIMULATION_DT =
      Seconds.of(TimedRobot.kDefaultPeriod / SIMULATION_SUB_TICKS_IN_1_PERIOD);

  public static Time getSimulationDt() {
    return SIMULATION_DT;
  }

  /**
   * Overrides the timing configuration of the simulation. Defaults are 5 sub-ticks per 20 ms robot
   * period.
   */
  public static synchronized void overrideSimulationTimings(
      Time robotPeriod, int simulationSubTicksPerPeriod) {
    SIMULATION_SUB_TICKS_IN_1_PERIOD = simulationSubTicksPerPeriod;
    SIMULATION_DT = robotPeriod.div(SIMULATION_SUB_TICKS_IN_1_PERIOD);
  }

  protected final World<Body> physicsWorld;
  protected final Set<AbstractDriveTrainSimulation> driveTrainSimulations;
  protected final List<Simulatable> customSimulations;

  protected SimulatedArena() {
    this.physicsWorld = new World<>();
    this.physicsWorld.setGravity(PhysicsWorld.ZERO_GRAVITY);
    this.driveTrainSimulations = new HashSet<>();
    this.customSimulations = new ArrayList<>();
  }

  /** Custom per-sub-tick callback. */
  public interface Simulatable {
    /**
     * Called in {@link #simulationSubTick(int)}.
     *
     * @param subTickNum the number of this sub-tick (counting from 0 in each robot period)
     */
    void simulationSubTick(int subTickNum);
  }

  /** Registers a custom simulation to tick once per sub-tick. */
  public synchronized void addCustomSimulation(Simulatable simulatable) {
    this.customSimulations.add(simulatable);
  }

  /** Registers a drivetrain sim and adds it to the physics world. */
  public synchronized void addDriveTrainSimulation(
      AbstractDriveTrainSimulation driveTrainSimulation) {
    this.physicsWorld.addBody(driveTrainSimulation);
    this.driveTrainSimulations.add(driveTrainSimulation);
  }

  /** Removes all bodies from the physics world. */
  public synchronized void shutDown() {
    this.physicsWorld.removeAllBodies();
  }

  /**
   * Update the simulation world.
   *
   * <p>This method should be called ONCE in {@link TimedRobot#simulationPeriodic()} (or {@code
   * LoggedRobot.simulationPeriodic()} if using AdvantageKit).
   */
  public synchronized void simulationPeriodic() {
    synchronized (SimulatedArena.class) {
      final long t0 = System.nanoTime();
      for (int i = 0; i < SIMULATION_SUB_TICKS_IN_1_PERIOD; i++) simulationSubTick(i);
      SmartDashboard.putNumber(
          "MapleArenaSimulation/Dyn4jEngineCPUTimeMS", (System.nanoTime() - t0) / 1.0e6);
    }
  }

  protected void simulationSubTick(int subTickNum) {
    SimulatedBattery.simulationSubTick();
    driveTrainSimulations.forEach(AbstractDriveTrainSimulation::simulationSubTick);
    this.physicsWorld.step(1, SIMULATION_DT.in(Seconds));
    customSimulations.forEach(sim -> sim.simulationSubTick(subTickNum));
  }
}
