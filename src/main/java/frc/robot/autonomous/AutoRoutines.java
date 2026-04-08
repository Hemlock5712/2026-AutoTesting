package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeCoordinator;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.geometry.ExtPose;

public class AutoRoutines {

  private final AutoCommands autoCommands;
  private final Superstructure superstructure;
  private final IntakeCoordinator intakeCoordinator;

  private static final double RIGHT_TRENCH_CENTER = 0.639445;
  private static final double LEFT_TRENCH_CENTER = FieldInfo.width().in(Meters) - 0.639445;
  private static final double BUMPERS_ON_LINE = 4.378;

  public AutoRoutines(
      AutoCommands autoCommands,
      Superstructure superstructure,
      IntakeCoordinator intakeCoordinator) {
    this.autoCommands = autoCommands;
    this.superstructure = superstructure;
    this.intakeCoordinator = intakeCoordinator;
  }

  public static Rotation2d snapToNearest90Degrees(Rotation2d rotation) {
    double degrees = rotation.getDegrees();
    double snapped = Math.round(degrees / 90.0) * 90.0;
    return Rotation2d.fromDegrees(snapped);
  }

  public static Rotation2d snapToNearest180Degrees(Rotation2d rotation) {
    double degrees = rotation.getDegrees();
    double snapped = Math.round(degrees / 180.0) * 180.0;
    return Rotation2d.fromDegrees(snapped);
  }

  public Command sequentialScoringAuto() {
    return Commands.sequence(
        Commands.print("=== Sequential Scoring Auto ==="),
        autoCommands.resetPose(() -> Pose2d.kZero),
        autoCommands
            .pathPlan()
            .from(Pose2d.kZero)
            .through(new ExtPose(3.0, 0, Rotation2d.kZero))
            .through(new ExtPose(3.0, 3.0, Rotation2d.kZero))
            .to(new ExtPose(0, 0, Rotation2d.kZero))
            .build());
  }

  // ==================== Right-Side Auto Routines ====================

  public Command rightAutoPass() {
    return Commands.sequence(
        autoCommands.rightAutoSetup(),
        // Drive to midline, through midline center, back while passing
        autoCommands
            .pathPlan()
            .from(new ExtPose(BUMPERS_ON_LINE, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(90)))
            .through(new ExtPose(5.965, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(90)))
            .through(new ExtPose(8.1, 1.036, Rotation2d.fromDegrees(90)))
            .through(
                new ExtPose(8.1, FieldInfo.width().in(Meters) / 2.0, Rotation2d.fromDegrees(90)))
            .withMaxSpeed(1)
            .withCommand(
                () ->
                    superstructure.passToLocation(new Translation2d(Inches.of(48), Inches.of(32))))
            .to(new ExtPose(6.5, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)))
            .withMaxSpeed(2)
            .onPassingX(6.0, intakeCoordinator::deployAndRunAUTO)
            .build(),
        superstructure.stopShoot(),
        // Drive under trench to outpost
        superstructure.hubShoot(),
        autoCommands
            .pathPlan()
            .through(new ExtPose(4.378, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180)))
            .to(new ExtPose(0.914, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)))
            .withMaxSpeed(2)
            .build(),
        intakeCoordinator.upAndRun());
  }

  public Command rightAuto() {
    return Commands.sequence(
        autoCommands.rightAutoSetup(),
        // Drive to midline, through balls, return under trench to outpost
        autoCommands
            .pathPlan()
            .from(new ExtPose(BUMPERS_ON_LINE, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(90)))
            .through(new ExtPose(5.965, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(90)))
            .through(new ExtPose(8.1, 1.036, Rotation2d.fromDegrees(90)))
            .through(new ExtPose(8.1, 2.766, Rotation2d.fromDegrees(110)))
            .withMaxSpeed(1.5)
            .withCommand(superstructure::hubShoot)
            .through(new ExtPose(5.959, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)))
            .through(new ExtPose(4.378, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180)))
            .to(new ExtPose(0.744, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)))
            .withMaxSpeed(2)
            .onPassingX(6.0, intakeCoordinator::deployAndRun)
            .build(),
        intakeCoordinator.upAndRun());
  }

  public Command rightShortAuto() {
    return Commands.sequence(
        autoCommands.rightAutoSetup(),
        // Drive to midline, collect balls, return to outpost
        autoCommands
            .pathPlan()
            .from(new ExtPose(BUMPERS_ON_LINE, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(90)))
            .through(new ExtPose(5.965, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(90)))
            .through(new ExtPose(7.6, 1.036, Rotation2d.fromDegrees(90)))
            .withCommand(intakeCoordinator::deployAndRun)
            .through(new ExtPose(7.6, 2.766, Rotation2d.fromDegrees(90)))
            .withMaxSpeed(1)
            .through(new ExtPose(5.959, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)))
            .through(new ExtPose(4.378, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180)))
            .to(new ExtPose(0.744, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)))
            .withMaxSpeed(2)
            .withCommand(superstructure::hubShoot)
            .build(),
        Commands.waitSeconds(3),
        autoCommands
            .pathPlan()
            .to(new ExtPose(3.5, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180)))
            .withMaxSpeed(2)
            .build(),
        superstructure.hubShoot());
  }

  public Command rightShortExtendedAuto() {
    return Commands.sequence(
        autoCommands.rightAutoSetup(),
        // Drive to midline with intake, stop at trench return for heading alignment
        autoCommands
            .pathPlan()
            .from(new ExtPose(BUMPERS_ON_LINE, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(90)))
            .through(new ExtPose(5.965, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(90)))
            .through(new ExtPose(7.6, 1.036, Rotation2d.fromDegrees(90)))
            .withCommand(intakeCoordinator::deployAndRun)
            .through(new ExtPose(7.6, 2.766, Rotation2d.fromDegrees(90)))
            .withMaxSpeed(1)
            .to(new ExtPose(5.959, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)))
            .build(),
        // Under trench to outpost, wait, drive back
        superstructure.hubShoot(),
        autoCommands
            .pathPlan()
            .through(new ExtPose(4.378, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180)))
            .to(new ExtPose(0.744, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180)))
            .withMaxSpeed(2)
            .build(),
        Commands.waitSeconds(2),
        autoCommands
            .pathPlan()
            .through(new ExtPose(3.5, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180)))
            .withMaxSpeed(2)
            .build(),
        superstructure.stopShoot(),
        // Second pass: out and back through the trench
        autoCommands
            .pathPlan()
            .through(new ExtPose(5.965, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180)))
            .through(new ExtPose(5.965, 4.0, Rotation2d.fromDegrees(90)))
            .withMaxSpeed(2)
            .through(new ExtPose(5.959, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)))
            .to(new ExtPose(4.0, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180)))
            .build(),
        superstructure.hubShoot());
  }

  // ==================== Left-Side Auto Routines ====================

  public Command leftShortSideAuto() {
    return leftAutoCore(8.481);
  }

  private Command leftAutoCore(double midlineX) {
    return Commands.sequence(
        autoCommands.leftAutoSetup(),
        // Drive to midline, through balls, return under trench
        autoCommands
            .pathPlan()
            .from(new ExtPose(BUMPERS_ON_LINE, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-90)))
            .through(new ExtPose(5.965, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-90)))
            .through(new ExtPose(midlineX, 7.20, Rotation2d.fromDegrees(-90)))
            .through(new ExtPose(8.481, 5.263, Rotation2d.fromDegrees(-110)))
            .withMaxSpeed(2)
            .through(new ExtPose(6.2, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(0)))
            .to(new ExtPose(3.8, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(0)))
            .onPassingX(6.0, intakeCoordinator::deployAndRun)
            .build(),
        // Shoot for 3 seconds
        superstructure.hubShoot(),
        Commands.waitSeconds(3),
        superstructure.stopShoot(),
        intakeCoordinator.deployAndRun(),
        // Second pass: drive back through trench and collect
        autoCommands
            .pathPlan()
            .through(new ExtPose(5.965, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(0)))
            .through(new ExtPose(7.8, 4.0, Rotation2d.fromDegrees(-80)))
            .withMaxSpeed(2)
            .through(new ExtPose(6.2, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(0)))
            .to(new ExtPose(3.8, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(0)))
            .build(),
        // Final shoot
        Commands.parallel(
            Commands.sequence(
                Commands.waitSeconds(3), intakeCoordinator.upAndRun(), Commands.waitSeconds(2)),
            superstructure.hubShoot()));
  }

  // ==================== Trench Traverse ====================

  public Command throughRightTrench(boolean centerToZone) {
    return Commands.either(
        autoCommands
            .pathPlan()
            .through(new ExtPose(5.959, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)))
            .withMaxSpeed(3)
            .withCommand(superstructure::stopShoot)
            .through(new ExtPose(3.517, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180)))
            .withMaxSpeed(4)
            .build(),
        autoCommands
            .pathPlan()
            .through(new ExtPose(3.517, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(180)))
            .withMaxSpeed(4)
            .through(new ExtPose(5.959, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)))
            .withMaxSpeed(3)
            .withCommand(superstructure::stopShoot)
            .build(),
        () -> (centerToZone));
  }

  public Command throughLeftTrench(boolean centerToZone) {
    return Commands.either(
        autoCommands
            .pathPlan()
            .through(new ExtPose(5.959, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)))
            .withMaxSpeed(3)
            .withCommand(superstructure::stopShoot)
            .through(new ExtPose(3.517, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(180)))
            .withMaxSpeed(4)
            .build(),
        autoCommands
            .pathPlan()
            .through(new ExtPose(3.517, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(180)))
            .withMaxSpeed(4)
            .through(new ExtPose(5.959, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)))
            .withMaxSpeed(3)
            .withCommand(superstructure::stopShoot)
            .build(),
        () -> (centerToZone));
  }

  // ==================== Feed Autos ====================

  public Command leftAutoFeed(double midlineX) {
    double fieldHalfY = FieldInfo.width().div(2).plus(Feet.of(2)).in(Meters);

    return Commands.sequence(
        autoCommands.leftAutoSetup(),
        // Drive to midline, through balls, under trench, through outpost
        autoCommands
            .pathPlan()
            .from(new ExtPose(BUMPERS_ON_LINE, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-90)))
            .through(new ExtPose(midlineX, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-90)))
            .through(new ExtPose(midlineX, fieldHalfY, Rotation2d.fromDegrees(-100)))
            .withMaxSpeed(1.5)
            .through(
                new ExtPose(
                    midlineX - Feet.of(2).in(Meters), fieldHalfY, Rotation2d.fromDegrees(-235)))
            .withMaxSpeed(1.5)
            .through(
                new ExtPose(
                    midlineX - Feet.of(2).in(Meters),
                    LEFT_TRENCH_CENTER,
                    Rotation2d.fromDegrees(-180)))
            .withMaxSpeed(2)
            .through(new ExtPose(3.8, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)))
            .withCommand(superstructure::spinUpShooter)
            .through(new ExtPose(0.76, 7.0, Rotation2d.fromDegrees(-120)))
            .withMaxSpeed(1.25)
            .withCommand(superstructure::shoot)
            .through(new ExtPose(0.76, 5.284, Rotation2d.fromDegrees(-120)))
            .withMaxSpeed(1)
            .withCommand(superstructure::shoot)
            .through(new ExtPose(3.5, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(0)))
            .withMaxSpeed(1)
            .withCommand(superstructure::shoot)
            .onPassingX(6.0, intakeCoordinator::deployAndRunAUTO)
            .build(),
        intakeCoordinator.slowUpAndRun());
  }

  public Command leftAutoFeedActual(double midlineX) {
    double fieldHalfY = FieldInfo.width().div(2).in(Meters);

    return Commands.sequence(
        autoCommands.leftAutoSetup(),
        // Drive to midline, through balls while feed-shooting
        autoCommands
            .pathPlan()
            .from(new ExtPose(BUMPERS_ON_LINE, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-90)))
            .through(new ExtPose(midlineX, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-90)))
            .withMaxSpeed(5)
            .withCommand(superstructure::spinUpShooter)
            .through(new ExtPose(midlineX, fieldHalfY, Rotation2d.fromDegrees(-90)))
            .withMaxSpeed(1.25)
            .withCommand(superstructure::feedShoot)
            .through(
                new ExtPose(
                    midlineX - Feet.of(2).in(Meters), fieldHalfY, Rotation2d.fromDegrees(-260)))
            .withMaxSpeed(1.25)
            .through(
                new ExtPose(midlineX - Feet.of(2).in(Meters), 6.8, Rotation2d.fromDegrees(-260)))
            .withMaxSpeed(1.25)
            .through(
                new ExtPose(midlineX - Feet.of(3.5).in(Meters), 6.8, Rotation2d.fromDegrees(-100)))
            .withMaxSpeed(1.25)
            .through(
                new ExtPose(
                    midlineX - Feet.of(4).in(Meters), fieldHalfY, Rotation2d.fromDegrees(-90)))
            .withMaxSpeed(1.25)
            .onPassingX(6.0, intakeCoordinator::deployAndRunAUTO)
            .build(),
        superstructure.stopShoot(),
        // Return through trench with shooting at each segment
        autoCommands
            .pathPlan()
            .through(new ExtPose(6.35, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)))
            // .withMaxSpeed(4)
            .through(new ExtPose(3.8, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)))
            // .withMaxSpeed(4)
            .withPositionTolerance(Inches.of(4))
            .withCommand(superstructure::spinUpShooter)
            .through(new ExtPose(2.2, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)))
            .withMaxSpeed(2)
            .withPositionTolerance(Inches.of(4))
            .withCommand(superstructure::shoot)
            .to(new ExtPose(0.9, LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(-180)))
            .withMaxSpeed(0.5)
            .withPositionTolerance(Inches.of(4))
            .withCommand(superstructure::shoot)
            .build(),
        intakeCoordinator.slowUpAndRun());
  }

  // ==================== Right-Side Feed Auto ====================

  public Command rightAutoJustFeed() {
    return Commands.sequence(
        autoCommands.rightAutoSetupFaceForward(),
        // To midline, into neutral zone: spin up, deploy, feed — all on-path (no sequential block)
        autoCommands
            .pathPlan()
            .from(new ExtPose(BUMPERS_ON_LINE, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(0)))
            .through(new ExtPose(6, RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(0)))
            // .withMaxSpeed(5)
            .withCommand(superstructure::spinUpShooter)
            .through(
                new ExtPose(6.3, FieldInfo.width().div(2).in(Meters), Rotation2d.fromDegrees(30)))
            .withCommand(intakeCoordinator::deployAndRun)
            .through(
                new ExtPose(8, FieldInfo.width().div(2).in(Meters), Rotation2d.fromDegrees(-45)))
            .withCommand(superstructure::feedShoot)
            .build(),
        neutralZoneLoop().repeatedly());
  }

  private Command neutralZoneLoop() {
    return autoCommands
        .pathPlan()
        .through(new ExtPose(8.3, 1.68, Rotation2d.fromDegrees(-95)))
        .through(new ExtPose(8 - Feet.of(3).in(Meters), 1.68, Rotation2d.fromDegrees(95)))
        .through(new ExtPose(8 - Feet.of(3).in(Meters), 6.41, Rotation2d.fromDegrees(85)))
        .through(new ExtPose(8.3, 6.41, Rotation2d.fromDegrees(-85)))
        .withGlobalMaxSpeed(1.5)
        .whileFollowingPath(superstructure::feedShoot)
        .build();
  }
}
