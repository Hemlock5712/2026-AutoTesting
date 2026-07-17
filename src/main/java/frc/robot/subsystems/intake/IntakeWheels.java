package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TalonFXUtil;
import frc.robot.utils.Tunables;
import frc.robot.utils.Tunables.TunableDouble;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeWheels extends SubsystemBase {

  private enum AntiJamState {
    NORMAL,
    MONITORING,
    JAM_RECOVERY
  }

  private static final String[] ANTI_JAM_STATE_NAMES =
      java.util.Arrays.stream(AntiJamState.values()).map(Enum::name).toArray(String[]::new);

  private static final boolean ANTI_JAM_ENABLED = true;

  private final TalonFX wheel = new TalonFX(23, CANBus.roboRIO());
  private final StatusSignal<AngularVelocity> wheelVelSignal = wheel.getVelocity();
  private final StatusSignal<Current> wheelStatorSignal = wheel.getStatorCurrent();

  private TalonFXConfiguration wheelConfig = new TalonFXConfiguration();

  Alert motorConfigAlert = new Alert("Intake Wheel Motor Configuration Failed", AlertType.kError);

  VelocityVoltage voltageOut = new VelocityVoltage(0);

  private AntiJamState antiJamState = AntiJamState.NORMAL;
  private final Timer jamTimer = new Timer();
  private final Timer recoveryTimer = new Timer();
  private final TunableDouble minDriveRequestRPS =
      Tunables.value("IntakeWheels/AntiJam/MinDriveRequestRPS", 1.0);
  private final TunableDouble jamCurrentThresholdAmps =
      Tunables.value("IntakeWheels/AntiJam/JamCurrentThresholdAmps", 50.0);
  private final TunableDouble jamVelocityThresholdRPS =
      Tunables.value("IntakeWheels/AntiJam/JamVelocityThresholdRPS", 8.0);
  private final TunableDouble jamConfirmTimeSeconds =
      Tunables.value("IntakeWheels/AntiJam/JamConfirmTimeSeconds", 0.15);
  private final TunableDouble jamRecoveryTimeSeconds =
      Tunables.value("IntakeWheels/AntiJam/JamRecoveryTimeSeconds", 0.25);
  private double requestedVelocityRPS = 0.0;

  public IntakeWheels() {
    wheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    wheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    wheelConfig.Feedback.SensorToMechanismRatio = 2.33;

    wheelConfig.Slot0.kS = 0.3; // Static friction
    wheelConfig.Slot0.kP = 0.1; // Proportional gain (speed of correction)
    wheelConfig.Slot0.kV = 0.288; // Velocity feedforward

    boolean success = TalonFXUtil.applyConfigWithRetries(wheel, wheelConfig);
    motorConfigAlert.set(!success);

    wheelVelSignal.setUpdateFrequency(50);
    wheelStatorSignal.setUpdateFrequency(50);

    wheel.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    long _t = System.nanoTime();
    BaseStatusSignal.refreshAll(wheelVelSignal, wheelStatorSignal);
    updateAntiJam();
    Logger.recordOutput("IntakeWheels/AntiJamState", ANTI_JAM_STATE_NAMES[antiJamState.ordinal()]);
    Logger.recordOutput("IntakeWheels/JamDetected", antiJamState == AntiJamState.JAM_RECOVERY);
    Logger.recordOutput("Timing/IntakeWheelsMs", (System.nanoTime() - _t) / 1e6);
  }

  public Command runIntakeAuto() {
    return runOnce(() -> setVelocityRPS(15));
  }

  public Command runIntake() {
    return runOnce(() -> setVelocityRPS(25));
  }

  public Command reverseIntake() {
    return runOnce(() -> setVelocityRPS(-20));
  }

  public Command runFast() {
    return runOnce(() -> setVelocityRPS(37));
  }

  public Command stopWheel() {
    return runOnce(this::stop);
  }

  @AutoLogOutput
  public double getVelocity() {
    return wheelVelSignal.getValueAsDouble();
  }

  @AutoLogOutput
  public double getVelocityTarget() {
    return requestedVelocityRPS;
  }

  @AutoLogOutput
  public double getStatorCurrentAmps() {
    return wheelStatorSignal.getValueAsDouble();
  }

  private void setVelocityRPS(double velocityRPS) {
    requestedVelocityRPS = velocityRPS;
    resetAntiJam();
    wheel.setControl(voltageOut.withVelocity(velocityRPS));
  }

  private void stop() {
    requestedVelocityRPS = 0.0;
    resetAntiJam();
    wheel.setControl(voltageOut.withVelocity(0));
  }

  private void updateAntiJam() {
    if (!ANTI_JAM_ENABLED) {
      if (antiJamState != AntiJamState.NORMAL) {
        resetAntiJam();
      }
      return;
    }

    switch (antiJamState) {
      case NORMAL:
        if (isPotentiallyJammed()) {
          antiJamState = AntiJamState.MONITORING;
          jamTimer.restart();
        }
        break;
      case MONITORING:
        if (!isPotentiallyJammed()) {
          antiJamState = AntiJamState.NORMAL;
          jamTimer.stop();
          jamTimer.reset();
          return;
        }

        if (jamTimer.hasElapsed(jamConfirmTimeSeconds.get())) {
          antiJamState = AntiJamState.JAM_RECOVERY;
          jamTimer.stop();
          jamTimer.reset();
          recoveryTimer.restart();
          wheel.setControl(voltageOut.withVelocity(0));
        }
        break;
      case JAM_RECOVERY:
        if (Math.abs(requestedVelocityRPS) < minDriveRequestRPS.get()) {
          resetAntiJam();
          wheel.setControl(voltageOut.withVelocity(0));
          return;
        }

        if (recoveryTimer.hasElapsed(jamRecoveryTimeSeconds.get())) {
          resetAntiJam();
          wheel.setControl(voltageOut.withVelocity(requestedVelocityRPS));
        }
        break;
    }
  }

  private boolean isPotentiallyJammed() {
    return Math.abs(requestedVelocityRPS) >= minDriveRequestRPS.get()
        && Math.abs(getVelocity()) < jamVelocityThresholdRPS.get()
        && getStatorCurrentAmps() > jamCurrentThresholdAmps.get();
  }

  private void resetAntiJam() {
    antiJamState = AntiJamState.NORMAL;
    jamTimer.stop();
    jamTimer.reset();
    recoveryTimer.stop();
    recoveryTimer.reset();
  }
}
