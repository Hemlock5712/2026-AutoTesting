package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cortex.motor.TalonFXUtil;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

@Logged
public class Turret extends SubsystemBase {

  protected final TalonFX leader = new TalonFX(41, CANBus.roboRIO());

  private final MotionMagicExpoTorqueCurrentFOC angleOut = new MotionMagicExpoTorqueCurrentFOC(0);

  private final double overallGearRatio = 110.0 / 25.0 * 7.0;
  Alert motorConfigAlert = new Alert("Turret Motor Configuration Failed", AlertType.kError);

  private final StatusSignal<Angle> angleSignal = leader.getPosition();
  private final StatusSignal<AngularVelocity> velocitySignal = leader.getVelocity();

  @Logged
  private Angle turretAngle = Radians.of(0);

  @Logged
  private Angle targetAngle = Radians.of(0);

  private CommandSwerveDrivetrain drivetrain;

  public Turret(CommandSwerveDrivetrain drivetrain) {

    this.drivetrain = drivetrain;
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = overallGearRatio;

    // PID gains
    config.Slot0.kS = 1.0; // Static friction compensation
    config.Slot0.kP = 20; // Proportional gain
    config.Slot0.kD = 0; // Derivative gain (damping to reduce overshoot)

    // MotionMagic settings - limits how fast the turret can move
    // Values are in sensor (motor) rotations per second
    // For ~1 rotation/sec at mechanism (57 deg/sec), motor needs: 1.0 * gearRatio =
    // 30.8 RPS
    config.MotionMagic.MotionMagicCruiseVelocity = 30.0; // motor rotations/sec (~1 mechanism rot/sec)
    // Acceleration: how fast it can speed up (motor rotations per second squared)
    config.MotionMagic.MotionMagicAcceleration = 60.0; // motor rotations/sec²

    boolean success = TalonFXUtil.applyConfigWithRetries(leader, config);
    motorConfigAlert.set(success);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    angleSignal.refresh();
    turretAngle = angleSignal.getValue();
    targetAngle = angleOut.getPositionMeasure();
  }

  public Command trackHubCommand() {
    return run(() -> {
      Pose2d robotPose = drivetrain.getPose();

      // Get target position (static hub position - Superstructure handles shoot-while-moving)
      Translation2d targetPosition = FieldConstants.HUB_POSITION.get();

      // Calculate the angle to the target in field coordinates
      double angleToTargetField = Math.atan2(
          targetPosition.getY() - robotPose.getY(),
          targetPosition.getX() - robotPose.getX());

      // Get the robot's current heading
      double robotHeading = robotPose.getRotation().getRadians();

      // Calculate turret angle relative to robot forward (oppose robot rotation)
      double turretAngleRadians = angleToTargetField - robotHeading;

      // Normalize angle to [-pi, pi]
      turretAngleRadians = Math.atan2(Math.sin(turretAngleRadians), Math.cos(turretAngleRadians));

      setAngle(Radians.of(turretAngleRadians));
    });
  }

  public void setAngle(Angle angle) {
    targetAngle = angle;
    leader.setControl(angleOut.withPosition(angle));
  }

  public Angle getAngle() {
    // return angleSignal.getValue();
    return targetAngle;
  }

  public void stop() {
    leader.stopMotor();
  }
}