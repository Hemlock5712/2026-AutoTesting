// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
//
// Adapted from the AdvantageKit talonfx_swerve template, MapleSim variant.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.generated.TunerConstants;
import frc.robot.simlib.drivesims.SwerveModuleSimulation;
import frc.robot.simlib.motorsims.SimulatedMotorController;
import frc.robot.util.PhoenixUtil;
import java.util.Arrays;

/**
 * Physics sim implementation of module IO backed by maple-sim's {@link SwerveModuleSimulation}. Sim
 * is always voltage-controlled. Odometry samples come from the sim's cached sub-tick state.
 */
public class ModuleIOSim implements ModuleIO {
  // TunerConstants doesn't carry sim-only gains, so they live here.
  private static final double DRIVE_KS = 0.03;
  // kV chosen so that FF = 12V at the rated top wheel speed (kSpeedAt12Volts / wheelRadius).
  // The earlier hard-coded 0.91035 V*s/rev gave FF ≈ 14.3 V at top speed — capped at 12 V by the
  // battery clamp, but more importantly the FF over-drove the motor at every intermediate speed.
  // During hard deceleration that meant the applied voltage stayed above back-EMF at the
  // setpoint, so the motor still produced forward torque (only the small KP*error term opposed
  // it) and the chassis lagged its commanded brake by ~1 m/s — visible as ~0.4 m of overshoot
  // at the end of paths. Computing kV from kSpeedAt12Volts brings FF into agreement with
  // back-EMF at every speed, so FF alone produces braking torque whenever the wheel is faster
  // than its setpoint.
  private static final double DRIVE_KV =
      12.0
          / (TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
              / TunerConstants.FrontLeft.WheelRadius);
  // Bumped from 0.05 to 0.4 (matching real-robot driveGains.kP) so the velocity loop has enough
  // authority to clamp tracking lag during hard brake commands. With the old gain, peak lag during
  // the path-end deceleration was ~1 m/s (most of the end-of-path overshoot).
  private static final double DRIVE_KP = 0.4;
  private static final double TURN_KP = 8.0;

  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private final PIDController driveController = new PIDController(DRIVE_KP, 0.0, 0.0);
  private final PIDController turnController = new PIDController(TURN_KP, 0.0, 0.0);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    this.driveMotor =
        moduleSimulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(TunerConstants.FrontLeft.SlipCurrent));
    this.turnMotor = moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(20));
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts
              + driveController.calculate(
                  moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts =
          turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians());
    } else {
      turnController.reset();
    }

    driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
    turnMotor.requestVoltage(Volts.of(turnAppliedVolts));

    inputs.driveConnected = true;
    inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));

    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnPosition = moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnVelocityRadPerSec =
        moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));

    inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositionsRad =
        Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
    inputs.odometryTurnPositions = moduleSimulation.getCachedSteerAbsolutePositions();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
