// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.SimStartup;
import frc.robot.utils.Tunables;
import java.lang.reflect.Field;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  private static final double MIN_OCV = 6.0;
  private static final double loopOverrunWarning = 0.2;

  public Robot() {
    Logger.recordMetadata("ProjectName", "FRC-Robot-Template");
    Logger.recordMetadata("Mode", Constants.getMode().toString());

    switch (Constants.getMode()) {
      case REAL -> {
        Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
        Logger.addDataReceiver(new NT4Publisher());
      }
      case SIM -> {
        Logger.addDataReceiver(new WPILOGWriter("logs"));
        Logger.addDataReceiver(new NT4Publisher());
      }
      case REPLAY -> {
        // Read sensor data from the log and replay as fast as the CPU can run.
        setUseTiming(false);
        String logPath = System.getProperty("frc.replay.input", "logs/replay-input.wpilog");
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(logPath.replace(".wpilog", "_replay.wpilog")));
      }
    }
    Logger.start();

    m_robotContainer = new RobotContainer();
    RobotController.setBrownoutVoltage(MIN_OCV);

    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(loopOverrunWarning);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings", false);
    }
    CommandScheduler.getInstance().setPeriod(loopOverrunWarning);

    SimStartup.arm();
  }

  @Override
  public void robotPeriodic() {
    long start = System.nanoTime();

    CommandScheduler.getInstance().run();
    long afterScheduler = System.nanoTime();

    Tunables.update();

    Logger.recordOutput("Timing/CommandSchedulerMs", (afterScheduler - start) / 1e6);
    Logger.recordOutput("Timing/TotalMs", (System.nanoTime() - start) / 1e6);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    FieldInfo.resetAllianceCache();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    FieldInfo.resetAllianceCache();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
