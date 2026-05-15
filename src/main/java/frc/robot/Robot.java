// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.SimStartup;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  /**
   * Brownout-shutdown floor in volts. WPILib's default (6.75 V) trips early under heavy transient
   * draws like shooter spin-up or simultaneous swerve acceleration on multiple modules. Lowering to
   * 6.0 V keeps motor outputs alive through those sags. Raise if the chassis browns out and resets
   * during a match; lower further (5.5 V or so) only if you've measured the battery and confirmed
   * the FPGA stays up.
   */
  private static final double BROWNOUT_VOLTAGE = 6.0;

  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

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
    RobotController.setBrownoutVoltage(BROWNOUT_VOLTAGE);
    SimStartup.arm();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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

  @Override
  public void simulationPeriodic() {
    m_robotContainer.updateSimulation();
  }
}
