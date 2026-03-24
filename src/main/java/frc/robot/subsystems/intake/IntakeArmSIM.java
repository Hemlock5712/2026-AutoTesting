package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.utils.MechanismUtil;
import frc.robot.utils.TalonFXUtil;

public class IntakeArmSIM extends IntakeArm {

  private static final double GEAR_RATIO = 25.0;
  private static final double ARM_LENGTH = 1.0;
  private static final double ARM_MASS_KG = 5.0;
  private static final double MIN_ANGLE_RAD = Units.rotationsToRadians(-1);
  private static final double MAX_ANGLE_RAD = Units.rotationsToRadians(1);
  private static final double STARTING_ANGLE_RAD = 0.0;
  private static final double SIM_PERIOD_SECONDS = 0.020;
  private static final double ARM_VISUAL_LENGTH = 200.0;
  private static final Angle INTAKE_STRUCTURE_VISUAL_OFFSET = Degrees.of(110);

  private final DCMotor dcMotor = DCMotor.getKrakenX60(1);
  private final SingleJointedArmSim armSim;
  private final MechanismUtil.ArmMechanism armMechanism;

  private final Translation3d frontArmPivotPoint = new Translation3d(.2921, 0, .18432);
  private final Translation3d backArmPivotPoint = new Translation3d(.24179, 0, .19685);
  private final Distance frontArmLength = Inches.of(9.471);
  private final Distance backArmLength = Inches.of(7.758);
  private final Distance distanceBetweenPivots = Inches.of(4);
  private final Angle frontArmBaseOffset = Degrees.of(112);

  @Logged(name = "Intake Mechanism3D")
  public Pose3d[] turretPose =
      new Pose3d[] {
        new Pose3d(frontArmPivotPoint, Rotation3d.kZero),
        new Pose3d(backArmPivotPoint, Rotation3d.kZero),
        new Pose3d()
      };

  private Translation3d lastBackArmAttachmentPoint;

  public IntakeArmSIM() {
    super();

    config.Feedback.RotorToSensorRatio = GEAR_RATIO;
    config.Slot0.kG = 0.0;
    config.Slot0.kS = 0.0;
    config.Slot0.kP = 160;
    config.Slot0.kD = 30;
    config.MotionMagic.MotionMagicCruiseVelocity = 1;
    config.MotionMagic.MotionMagicAcceleration = 4;
    TalonFXUtil.applyConfigWithRetries(arm, config);

    armSim =
        new SingleJointedArmSim(
            dcMotor,
            GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(ARM_LENGTH, ARM_MASS_KG),
            ARM_LENGTH,
            MIN_ANGLE_RAD,
            MAX_ANGLE_RAD,
            false,
            STARTING_ANGLE_RAD);

    armMechanism = new MechanismUtil.ArmMechanism("Arm", ARM_VISUAL_LENGTH);
    SmartDashboard.putData("Arm Sim", armMechanism.getMechanism());
  }

  @Override
  public void simulationPeriodic() {
    armSim.setInput(arm.getMotorVoltage().getValueAsDouble());
    armSim.update(SIM_PERIOD_SECONDS);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

    double encoderPosition = Radians.of(armSim.getAngleRads()).in(Rotations);
    double encoderVelocity =
        RadiansPerSecond.of(armSim.getVelocityRadPerSec()).in(RotationsPerSecond);

    armEncoder.getSimState().setRawPosition(encoderPosition);
    armEncoder.getSimState().setVelocity(encoderVelocity);

    double motorPosition = encoderPosition * GEAR_RATIO;
    double motorVelocity = encoderVelocity * GEAR_RATIO;
    arm.getSimState().setRawRotorPosition(motorPosition);
    arm.getSimState().setRotorVelocity(motorVelocity);

    updateVisualization();

    Robot.telemetry().log("Arm Sim Current (A)", armSim.getCurrentDrawAmps());
  }

  private void updateVisualization() {
    double currentAngleDeg = getPosition().in(Degrees);
    armMechanism.update(currentAngleDeg, isAtTarget());
    updateFourBarVisualization();
  }

  /**
   * Updates the intake four-bar visualization by solving the passive rear link from the current
   * front arm position. The intake structure pose is anchored at the front intake-side pivot and
   * points toward the rear intake-side pivot.
   */
  private void updateFourBarVisualization() {
    double frontArmAngleRad = getTargetPosition().in(Radians);
    Translation3d frontArmAttachmentPoint =
        projectLinkEndpoint(frontArmPivotPoint, frontArmLength.in(Meters), frontArmAngleRad);
    Translation3d backArmAttachmentPoint = solveBackArmAttachmentPoint(frontArmAttachmentPoint);
    lastBackArmAttachmentPoint = backArmAttachmentPoint;

    Pose3d frontArmPose = createLinkPose(frontArmPivotPoint, frontArmAngleRad);
    Pose3d backArmPose =
        createLinkPose(
            backArmPivotPoint, angleBetweenPoints(backArmPivotPoint, backArmAttachmentPoint));
    Pose3d intakeStructurePose =
        createLinkPose(
            frontArmAttachmentPoint,
            angleBetweenPoints(frontArmAttachmentPoint, backArmAttachmentPoint)
                + INTAKE_STRUCTURE_VISUAL_OFFSET.in(Radians));

    turretPose[0] = frontArmPose;
    turretPose[1] = backArmPose;
    turretPose[2] = intakeStructurePose;
  }

  private Pose3d createLinkPose(Translation3d pivotPoint, double angleRad) {
    return new Pose3d(pivotPoint, new Rotation3d(0, -angleRad, 0));
  }

  private Translation3d projectLinkEndpoint(
      Translation3d pivotPoint, double linkLengthMeters, double angleRad) {
    return new Translation3d(
        pivotPoint.getX() + linkLengthMeters * Math.cos(angleRad),
        pivotPoint.getY(),
        pivotPoint.getZ() + linkLengthMeters * Math.sin(angleRad));
  }

  private Translation3d solveBackArmAttachmentPoint(Translation3d frontArmAttachmentPoint) {
    double intakePivotSpacingMeters = distanceBetweenPivots.in(Meters);
    double backArmLengthMeters = backArmLength.in(Meters);

    double deltaX = backArmPivotPoint.getX() - frontArmAttachmentPoint.getX();
    double deltaZ = backArmPivotPoint.getZ() - frontArmAttachmentPoint.getZ();
    double pivotDistanceMeters = Math.hypot(deltaX, deltaZ);

    if (!isFourBarSolvable(pivotDistanceMeters, intakePivotSpacingMeters, backArmLengthMeters)) {
      return createFallbackBackArmAttachment(frontArmAttachmentPoint, backArmLengthMeters);
    }

    double distanceAlongChord =
        (square(intakePivotSpacingMeters)
                - square(backArmLengthMeters)
                + square(pivotDistanceMeters))
            / (2.0 * pivotDistanceMeters);
    double perpendicularOffsetMeters =
        Math.sqrt(Math.max(0.0, square(intakePivotSpacingMeters) - square(distanceAlongChord)));

    double unitX = deltaX / pivotDistanceMeters;
    double unitZ = deltaZ / pivotDistanceMeters;

    double chordMidpointX = frontArmAttachmentPoint.getX() + distanceAlongChord * unitX;
    double chordMidpointZ = frontArmAttachmentPoint.getZ() + distanceAlongChord * unitZ;

    Translation3d firstCandidate =
        new Translation3d(
            chordMidpointX - perpendicularOffsetMeters * unitZ,
            frontArmAttachmentPoint.getY(),
            chordMidpointZ + perpendicularOffsetMeters * unitX);
    Translation3d secondCandidate =
        new Translation3d(
            chordMidpointX + perpendicularOffsetMeters * unitZ,
            frontArmAttachmentPoint.getY(),
            chordMidpointZ - perpendicularOffsetMeters * unitX);

    return chooseBackArmAttachmentPoint(frontArmAttachmentPoint, firstCandidate, secondCandidate);
  }

  private boolean isFourBarSolvable(
      double pivotDistanceMeters, double intakePivotSpacingMeters, double backArmLengthMeters) {
    if (pivotDistanceMeters < 1e-9) {
      return false;
    }

    return pivotDistanceMeters <= intakePivotSpacingMeters + backArmLengthMeters
        && pivotDistanceMeters >= Math.abs(intakePivotSpacingMeters - backArmLengthMeters);
  }

  private Translation3d createFallbackBackArmAttachment(
      Translation3d frontArmAttachmentPoint, double backArmLengthMeters) {
    double fallbackAngleRad = angleBetweenPoints(backArmPivotPoint, frontArmAttachmentPoint);
    return projectLinkEndpoint(backArmPivotPoint, backArmLengthMeters, fallbackAngleRad);
  }

  private Translation3d chooseBackArmAttachmentPoint(
      Translation3d frontArmAttachmentPoint,
      Translation3d firstCandidate,
      Translation3d secondCandidate) {
    if (lastBackArmAttachmentPoint != null) {
      double firstError = firstCandidate.getDistance(lastBackArmAttachmentPoint);
      double secondError = secondCandidate.getDistance(lastBackArmAttachmentPoint);
      return firstError <= secondError ? firstCandidate : secondCandidate;
    }

    // Start on the open branch by keeping the passive joint on the same side of the chord from
    // the front intake pivot to the fixed back pivot as the driven front pivot.
    double preferredSide =
        signedTriangleArea(frontArmAttachmentPoint, backArmPivotPoint, frontArmPivotPoint);
    double firstSide =
        signedTriangleArea(frontArmAttachmentPoint, backArmPivotPoint, firstCandidate);
    double secondSide =
        signedTriangleArea(frontArmAttachmentPoint, backArmPivotPoint, secondCandidate);

    return Math.abs(firstSide - preferredSide) <= Math.abs(secondSide - preferredSide)
        ? firstCandidate
        : secondCandidate;
  }

  private double angleBetweenPoints(Translation3d startPoint, Translation3d endPoint) {
    return Math.atan2(endPoint.getZ() - startPoint.getZ(), endPoint.getX() - startPoint.getX());
  }

  private double signedTriangleArea(
      Translation3d firstPoint, Translation3d secondPoint, Translation3d thirdPoint) {
    return (secondPoint.getX() - firstPoint.getX()) * (thirdPoint.getZ() - firstPoint.getZ())
        - (secondPoint.getZ() - firstPoint.getZ()) * (thirdPoint.getX() - firstPoint.getX());
  }

  private double square(double value) {
    return value * value;
  }
}
