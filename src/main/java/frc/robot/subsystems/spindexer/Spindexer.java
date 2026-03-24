package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;

@Logged(strategy = Strategy.OPT_IN)
public class Spindexer extends SubsystemBase {
  private static final double VELOCITY_TOLERANCE = 0.2;

  private static final double BACK_VELOCITY = -12;
  private static final double FORWARD_SPINDEXER_VEL = 12;
  private static final double FORWARD_KICKER_VEL = 10;
  private static final double PREP_FEED_KICKER_VEL =
      0; // Old value: 10. Test to see if this makes less balls fly everywhere

  protected final TalonFX spindexer = new TalonFX(20, TunerConstants.kCANBus);

  protected final TalonFX kicker = new TalonFX(21, TunerConstants.kCANBus);

  private final VelocityTorqueCurrentFOC spindexerVelocityOut = new VelocityTorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC kickerVelocityOut = new VelocityTorqueCurrentFOC(0);

  protected TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();

  protected TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

  private StatusSignal<AngularVelocity> kickerVelocity = kicker.getVelocity();
  private final StatusSignal<Current> spindexerCurrentDraw = spindexer.getSupplyCurrent();

  Alert motorConfigAlert = new Alert("Spindexer Motor Configuration Failed", AlertType.kError);

  Alert kickerMotorConfigAlert = new Alert("Kicker Motor Configuration Failed", AlertType.kError);

  public Spindexer() {
    applyConfigs();
  }

  private void setVelocity(double spindexerVel, double kickerVel) {
    spindexer.setControl(spindexerVelocityOut.withVelocity(spindexerVel));
    kicker.setControl(kickerVelocityOut.withVelocity(kickerVel));
  }

  public Command backCommand() {
    return runOnce(() -> setVelocity(BACK_VELOCITY, BACK_VELOCITY));
  }

  public Command forwardCommand() {
    return runOnce(() -> setVelocity(FORWARD_SPINDEXER_VEL, FORWARD_KICKER_VEL));
  }

  public Command prepFeed() {
    return runOnce(() -> setVelocity(0, PREP_FEED_KICKER_VEL));
  }

  private void stop() {
    spindexer.stopMotor();
    kicker.stopMotor();
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  @Logged
  public boolean isAtTarget() {
    return spindexer.getVelocity().isNear(spindexerVelocityOut.Velocity, VELOCITY_TOLERANCE);
  }

  public void applyConfigs() {
    spindexerConfig.Slot0.kS = 5; // Static friction compensation
    spindexerConfig.Slot0.kP = 20; // Proportional gain
    spindexerConfig.Slot0.kD = 0.0; // Derivative gain (damping to reduce overshoot)
    spindexerConfig.Slot0.kV = 0.55;
    // MotionMagic settings - with SensorToMechanismRatio set, units are mechanism
    // rotations
    // Cruise velocity: max SPINDEXER speed during motion profile (RPS)
    // Acceleration: how quickly the SPINDEXER speeds up/slows down (RPS²)
    spindexerConfig.MotionMagic.MotionMagicCruiseVelocity = 0; // RPS
    spindexerConfig.MotionMagic.MotionMagicAcceleration = 0; // RPS²

    spindexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    spindexerConfig.Feedback.SensorToMechanismRatio = 9.0;

    kickerConfig.Slot0.kS = 1.5; // Static friction compensation
    kickerConfig.Slot0.kP = 20; // Proportional gain
    kickerConfig.Slot0.kD = 0; // Derivative gain (damping to reduce overshoot)
    // MotionMagic settings - with SensorToMechanismRatio set, units are mechanism
    kickerConfig.Slot0.kV = 0.4;
    // rotations
    // Cruise velocity: max SPINDEXER speed during motion profile (RPS)
    // Acceleration: how quickly the SPINDEXER speeds up/slows down (RPS²)
    kickerConfig.MotionMagic.MotionMagicCruiseVelocity = 0; // RPS
    kickerConfig.MotionMagic.MotionMagicAcceleration = 0; // RPS²

    kickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    kickerConfig.Feedback.SensorToMechanismRatio = 7.0;

    boolean success = TalonFXUtil.applyConfigWithRetries(spindexer, spindexerConfig);
    motorConfigAlert.set(!success);
    success = TalonFXUtil.applyConfigWithRetries(kicker, kickerConfig);
    kickerMotorConfigAlert.set(!success);

    spindexer.optimizeBusUtilization();
    kicker.optimizeBusUtilization();
  }

  @Logged
  public AngularVelocity getTargetKickerVelocity() {
    return kickerVelocityOut.getVelocityMeasure();
  }

  @Logged
  public AngularVelocity getKickerVelocity() {
    return kickerVelocity.getValue();
  }

  @Logged
  public Current getSpindexerCurrentDraw() {
    return spindexerCurrentDraw.getValue();
  }

  @Override
  public void periodic() {
    StatusSignal.refreshAll(kickerVelocity, spindexerCurrentDraw);
  }
}
