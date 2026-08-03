// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Superstructure.FeedMode;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.HubShiftUtil;
import frc.robot.utils.Tunables;
import java.lang.reflect.Field;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.IterativeRobotBase;
import org.wpilib.system.RobotController;
import org.wpilib.system.Watchdog;

public class Robot extends LoggedRobot {

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // OCV estimation bounds
  private static final double MIN_OCV = 6.0; // Brownout threshold

  // Rumble timing thresholds (seconds before hub shift)
  private static final double RUMBLE_START_THRESHOLD = 1.0;
  private static final double RUMBLE_END_THRESHOLD = 0.5;

  private static final double loopOverrunWarning = 0.2;

  public Robot() {
    Logger.recordMetadata("2026Robot", "2026-AutoTesting");
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter("/home/systemcore/logs"));
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      // Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
    }
    Logger.start();

    m_robotContainer = new RobotContainer();
    RobotController.setBrownoutVoltage(MIN_OCV);

    HubShiftUtil.setupNTValues();

    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(loopOverrunWarning);
    } catch (Exception e) {
      DriverStationErrors.reportWarning("Failed to disable loop overrun warnings", false);
    }
  }

  @Override
  public void robotPeriodic() {
    long start = System.nanoTime();

    m_robotContainer.getSuperstructure().update();
    long afterSuper = System.nanoTime();

    Scheduler.getDefault().run();
    long afterScheduler = System.nanoTime();

    Tunables.update();

    Logger.recordOutput("Timing/SuperstructureMs", (afterSuper - start) / 1e6);
    Logger.recordOutput("Timing/CommandSchedulerMs", (afterScheduler - afterSuper) / 1e6);
    Logger.recordOutput("Timing/TotalMs", (System.nanoTime() - start) / 1e6);
  }

  @Override
  public void disabledInit() {
    m_robotContainer.setRumble(0.0);
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.updateAutoSelection();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    FieldInfo.resetAllianceCache();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      Scheduler.getDefault().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      Scheduler.getDefault().cancel(m_autonomousCommand);
    }

    FieldInfo.resetAllianceCache();
    HubShiftUtil.initialize();

    int station = MatchState.getLocation().orElse(1);
    if (station <= 2) {
      m_robotContainer.getSuperstructure().setTeleopFeedMode(FeedMode.FORCE_LEFT);
    } else {
      m_robotContainer.getSuperstructure().setTeleopFeedMode(FeedMode.FORCE_RIGHT);
    }

    if (RobotState.isFMSAttached()) {
      Scheduler.getDefault().schedule(m_robotContainer.fmsInitCommand());
    } else {
      Scheduler.getDefault().schedule(m_robotContainer.stopCommand());
    }
  }

  @Override
  public void teleopPeriodic() {
    HubShiftUtil.update();

    // Rumble controller when a shift change is approaching
    double secondsUntilShift = HubShiftUtil.getSecondsUntilNextShift();
    m_robotContainer.setRumble(
        (secondsUntilShift <= RUMBLE_START_THRESHOLD && secondsUntilShift > RUMBLE_END_THRESHOLD)
            ? 1.0
            : 0.0);
  }

  @Override
  public void teleopExit() {
    m_robotContainer.setRumble(0.0);
  }

  @Override
  public void utilityInit() {
    Scheduler.getDefault().cancelAll();
  }

  @Override
  public void utilityPeriodic() {}

  @Override
  public void utilityExit() {}

  public static boolean isHubActive() {
    return HubShiftUtil.isHubActive();
  }
}
