// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
//
// Adapted from the AdvantageKit talonfx_swerve template.

package frc.robot.subsystems.drive;

import static frc.robot.utils.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  // Hardware objects
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  // Voltage control requests. Steer uses MotionMagicExpo (kV/kA shape is baked into the config
  // at construction). Drive uses MotionMagicVelocityVoltage with a per-call Acceleration that
  // smoothly interpolates between consecutive setpoints (computed in Module.runSetpoint).
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicExpoVoltage positionVoltageRequest = new MotionMagicExpoVoltage(0.0);
  private final MotionMagicVelocityVoltage velocityVoltageRequest =
      new MotionMagicVelocityVoltage(0.0);

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final MotionMagicExpoTorqueCurrentFOC positionTorqueCurrentRequest =
      new MotionMagicExpoTorqueCurrentFOC(0.0);
  private final MotionMagicVelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);

  // Timestamp inputs from Phoenix thread
  private final Queue<Double> timestampQueue;

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnEncoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ModuleIOTalonFX(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.constants = constants;
    driveTalon = new TalonFX(constants.DriveMotorId, TunerConstants.kCANBus);
    turnTalon = new TalonFX(constants.SteerMotorId, TunerConstants.kCANBus);
    cancoder = new CANcoder(constants.EncoderId, TunerConstants.kCANBus);

    // Configure drive motor
    var driveConfig = constants.DriveMotorInitialConfigs;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(
        "DriveTalon-" + constants.DriveMotorId + " config",
        5,
        () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(
        "DriveTalon-" + constants.DriveMotorId + " setPosition",
        5,
        () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor. Pull from constants.SteerMotorInitialConfigs so the azimuth stator-
    // current limit (and any other initial config) carries through; the AKit upstream template
    // started this with `new TalonFXConfiguration()`, which silently dropped those settings.
    var turnConfig = constants.SteerMotorInitialConfigs;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = constants.SteerMotorGains;
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    turnConfig.Feedback.FeedbackSensorSource =
        switch (constants.FeedbackSource) {
          case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
          default ->
              throw new RuntimeException(
                  "Turn feedback source not supported by ModuleIOTalonFX. Either change the source"
                      + " in TunerConstants or extend ModuleIOTalonFX.");
        };
    turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicAcceleration =
        turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(
        "TurnTalon-" + constants.SteerMotorId + " config",
        5,
        () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

    // Configure CANCoder
    CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
    cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    tryUntilOk(
        "CANcoder-" + constants.EncoderId + " config",
        5,
        () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25));

    // Create timestamp queue
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    // Create drive status signals
    drivePosition = driveTalon.getPosition();
    drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(drivePosition.clone());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    // Create turn status signals. The fused steer position lives on turnTalon.getPosition()
    // (Talon firmware fuses rotor + cancoder absolute internally). We also subscribe to the
    // cancoder absolute position for the disconnect alert and as an AKit-logged diagnostic.
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnPosition.clone());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    // Configure periodic frames. The cancoder absolute-position broadcast is what the Talon's
    // FusedCANcoder fusion uses for absolute references — match it to the rotor odometry rate
    // so every odometry sample is fully fused. Going higher than ODOMETRY_FREQUENCY would be
    // wasted CAN bandwidth (the rotor side caps fusion fresh-rate).
    BaseStatusSignal.setUpdateFrequencyForAll(
        Drive.ODOMETRY_FREQUENCY, drivePosition, turnPosition, turnAbsolutePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon, cancoder);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    var turnStatus =
        BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);
    var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

    // Azimuth coupling compensation. The drive shaft is dragged along by the steer mechanism
    // at CouplingGearRatio drive-rotor turns per steer-mechanism turn. drivePosition is
    // post-SensorToMechanismRatio (wheel rotations), so the phantom wheel motion to subtract
    // is steer_mech_rad * (CouplingGearRatio / DriveMotorGearRatio). CTRE's SwerveDrivetrain
    // class does this internally; we're not using it, so we do it here.
    final double couplingFactor = constants.CouplingGearRatio / constants.DriveMotorGearRatio;

    double rawDriveRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    double rawTurnMechRad = Units.rotationsToRadians(turnPosition.getValueAsDouble());
    double rawDriveVelRad = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    double rawTurnVelRad = Units.rotationsToRadians(turnVelocity.getValueAsDouble());

    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = rawDriveRad - rawTurnMechRad * couplingFactor;
    inputs.driveVelocityRadPerSec = rawDriveVelRad - rawTurnVelRad * couplingFactor;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = rawTurnVelRad;
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

    // Drain the high-rate queues in parallel so each odometry sample gets the matching steer
    // angle for its own coupling correction. The queues are populated together by the Phoenix
    // odometry thread, but defensively truncate to the shortest length.
    Double[] driveSamples = drivePositionQueue.toArray(new Double[0]);
    Double[] turnSamples = turnPositionQueue.toArray(new Double[0]);
    Double[] timestampSamples = timestampQueue.toArray(new Double[0]);
    int n = Math.min(driveSamples.length, Math.min(turnSamples.length, timestampSamples.length));
    double[] correctedDriveRad = new double[n];
    Rotation2d[] turnRotations = new Rotation2d[n];
    double[] timestamps = new double[n];
    for (int i = 0; i < n; i++) {
      double driveRad = Units.rotationsToRadians(driveSamples[i]);
      double turnMechRad = Units.rotationsToRadians(turnSamples[i]);
      correctedDriveRad[i] = driveRad - turnMechRad * couplingFactor;
      turnRotations[i] = Rotation2d.fromRotations(turnSamples[i]);
      timestamps[i] = timestampSamples[i];
    }
    inputs.odometryTimestamps = timestamps;
    inputs.odometryDrivePositionsRad = correctedDriveRad;
    inputs.odometryTurnPositions = turnRotations;
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec, double accelLimitRadPerSecSq) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    double accelRotPerSecSq = Units.radiansToRotations(accelLimitRadPerSecSq);
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage ->
              velocityVoltageRequest
                  .withVelocity(velocityRotPerSec)
                  .withAcceleration(accelRotPerSecSq);
          case TorqueCurrentFOC ->
              velocityTorqueCurrentRequest
                  .withVelocity(velocityRotPerSec)
                  .withAcceleration(accelRotPerSecSq);
        });
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
          case TorqueCurrentFOC ->
              positionTorqueCurrentRequest.withPosition(rotation.getRotations());
        });
  }
}
