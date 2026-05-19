// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
//
// Adapted from the AdvantageKit talonfx_swerve template.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private double lastTargetSpeed = 0.0;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(
      ModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Disconnected turn encoder on module " + Integer.toString(index) + ".",
            AlertType.kError);
  }

  /** Reads sensor values from the hardware. Must hold {@link Drive#odometryLock} when calling. */
  public void updateIo() {
    io.updateInputs(inputs);
  }

  /** Logs the sensor values and computes positions for each odometry sample. */
  public void postIoPeriodic() {
    Logger.processInputs("Drive/Module" + index, inputs);

    int sampleCount = inputs.odometryTimestamps.length;
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
  }

  /**
   * Drive at the given speed/angle. {@code dt} sets the MotionMagicVelocityVoltage accel limit.
   * The {@code optimize} + {@code cosineScale} against the measured angle compensate for steer
   * servo lag between ticks; stripping them regresses Out-path tracking 113mm → 176mm.
   */
  public void runSetpoint(SwerveModuleState state, double dt) {
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    double deltaV = Math.abs(state.speedMetersPerSecond - lastTargetSpeed);
    double accelLimitMetersPerSecSq = Math.max(deltaV / dt, 1.0);
    lastTargetSpeed = state.speedMetersPerSecond;

    double velocityRadPerSec = state.speedMetersPerSecond / constants.WheelRadius;
    double accelLimitRadPerSecSq = accelLimitMetersPerSecSq / constants.WheelRadius;
    io.setDriveVelocity(velocityRadPerSec, accelLimitRadPerSecSq);
    io.setTurnPosition(state.angle);
  }

  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
    // Reset the interpolator's reference so a future runSetpoint after re-engagement starts from
    // zero, not from the pre-stop commanded speed.
    lastTargetSpeed = 0.0;
  }

  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  public double getPositionMeters() {
    return inputs.drivePositionRad * constants.WheelRadius;
  }

  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * constants.WheelRadius;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }
}
