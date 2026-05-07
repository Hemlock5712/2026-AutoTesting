package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Coordinates IntakeArm and IntakeWheels together.
 *
 * <p>This is NOT a subsystem — it's a coordinator (same pattern as Superstructure). It provides
 * composed commands when the arm and wheels need to move together, and delegates to individual
 * subsystems for independent control.
 */
public class IntakeCoordinator {

  private final IntakeArm arm;
  private final IntakeWheels wheels;

  public IntakeCoordinator() {
    arm = RobotBase.isSimulation() ? new IntakeArmSIM() : new IntakeArm();
    wheels = new IntakeWheels();
  }

  // ==================== Delegated Arm Commands ====================

  public Command intakeDown() {
    return arm.intakeDown();
  }

  public Command intakeUp() {
    return arm.intakeUp();
  }

  public Command stopArm() {
    return arm.stopArm();
  }

  public Command stopBoth() {
    return stopArm().alongWith(stopWheel());
  }

  // ==================== Delegated Wheel Commands ====================

  public Command runIntake() {
    return wheels.runIntake();
  }

  public Command reverseIntake() {
    return wheels.reverseIntake();
  }

  public Command stopWheel() {
    return wheels.stopWheel();
  }

  public Command straightUp() {
    return wheels.stopWheel().andThen(arm.straightUp());
  }

  // ==================== Coordinated Commands ====================

  /** Deploy the arm down and start spinning the wheels in parallel (different subsystems). */
  public Command deployAndRun() {
    return Commands.parallel(arm.intakeDown(), wheels.runIntake());
  }

  public Command deployAndRunAUTO() {
    return Commands.parallel(arm.intakeDownAUTO(), wheels.runIntakeAuto());
  }

  public Command upAndRun() {
    return Commands.parallel(arm.intakeUp(), wheels.runFast());
  }

  public Command downAndRunFast() {
    return Commands.parallel(arm.intakeDown(), wheels.runFast());
  }

  public Command slowUpAndRun() {
    return Commands.parallel(arm.intakeUpSlow(), wheels.runFast());
  }

  /** Stop the wheels and retract the arm up in parallel (different subsystems). */
  public Command stopAndRetract() {
    return Commands.parallel(wheels.stopWheel(), arm.intakeUp());
  }

  // ==================== State Queries ====================

  @AutoLogOutput
  public boolean isAtTarget() {
    return arm.isAtTarget();
  }

  @AutoLogOutput
  public boolean isAtBumpHeight() {
    return arm.isAtBumpHeight();
  }

  @AutoLogOutput
  public double getTargetPositionRotations() {
    return arm.getTargetPosition();
  }

  @AutoLogOutput
  public double getPositionRotations() {
    return arm.getPositionRotations();
  }

  @AutoLogOutput
  public double getVelocity() {
    return wheels.getVelocity();
  }

  @AutoLogOutput
  public double getVelocityTarget() {
    return wheels.getVelocityTarget();
  }

  @AutoLogOutput
  public boolean isIntakeFast() {
    return wheels.getVelocity() > 35;
  }

  // ==================== Direct Access ====================

  public IntakeArm getArm() {
    return arm;
  }

  public IntakeWheels getWheels() {
    return wheels;
  }
}
