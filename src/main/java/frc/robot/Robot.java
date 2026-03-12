// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.HubShiftUtil;
import frc.robot.utils.Tunables;

@Logged
public class Robot extends TimedRobot {

  @NotLogged private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // OCV estimation bounds
  private static final double MIN_OCV = 6.0; // Brownout threshold

  // Rumble timing thresholds (seconds before hub shift)
  private static final double RUMBLE_START_THRESHOLD = 1.0;
  private static final double RUMBLE_END_THRESHOLD = 0.5;

  private boolean hasMatchStarted = false;

  public Robot() {
    m_robotContainer = new RobotContainer();
    RobotController.setBrownoutVoltage(MIN_OCV);
    DataLogManager.start();
    Epilogue.configure(
        config ->
            config.backend =
                EpilogueBackend.multi(new NTEpilogueBackend(NetworkTableInstance.getDefault())));
    Epilogue.bind(this);
  }

  @Override
  public void robotPeriodic() {
    m_robotContainer.getSuperstructure().update();
    CommandScheduler.getInstance().run();
    Tunables.update();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.setRumble(0.0);
  }

  @Override
  public void disabledPeriodic() {
    if (!hasMatchStarted) {
      m_robotContainer.checkStartingPosition();
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    hasMatchStarted = true;
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
    hasMatchStarted = true;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    HubShiftUtil.initialize();

    if (DriverStation.isFMSAttached()) {
      CommandScheduler.getInstance().schedule(m_robotContainer.fmsInitCommand());
    } else {
      CommandScheduler.getInstance().schedule(m_robotContainer.stopCommand());
    }
  }

  @Override
  public void teleopPeriodic() {
    HubShiftUtil.update();

    // Rumble controller when a shift change is 5 seconds away
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
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public static EpilogueBackend telemetry() {
    return Epilogue.getConfig().backend;
  }

  public static boolean isHubActive() {
    return HubShiftUtil.isHubActive();
  }
}
