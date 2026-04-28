package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.shooter.ShooterLookup;

/** Drives shooter lookup table test poses from the second controller. */
public final class LookupTestController {
  private final double[] distances = ShooterLookup.getHubTestDistances();
  private int selectedIndex = 0;
  private Pose2d selectedBluePose = Pose2d.kZero;
  private boolean poseValid = false;

  public LookupTestController() {
    updateSelectedPose();
  }

  public void selectNext() {
    selectedIndex = Math.min(selectedIndex + 1, distances.length - 1);
    updateSelectedPose();
  }

  public void selectPrevious() {
    selectedIndex = Math.max(selectedIndex - 1, 0);
    updateSelectedPose();
  }

  public Pose2d getAlliancePose() {
    return FieldInfo.flip(selectedBluePose);
  }

  public boolean isPoseValid() {
    return poseValid;
  }

  private void updateSelectedPose() {
    double distance = distances[selectedIndex];
    selectedBluePose = calculateBluePose(distance);
    poseValid = isInsideField(selectedBluePose);
    publish(distance);

    if (!poseValid) {
      DriverStation.reportWarning(
          "Lookup test pose is outside field bounds: " + selectedBluePose, false);
    }
  }

  private Pose2d calculateBluePose(double distance) {
    Translation2d hubToOrigin = Translation2d.kZero.minus(FieldInfo.HUB_POSITION);
    Translation2d direction = hubToOrigin.div(hubToOrigin.getNorm());
    Translation2d turretPosition = FieldInfo.HUB_POSITION.plus(direction.times(distance));
    Translation2d robotPosition =
        turretPosition.minus(Superstructure.TURRET_TRANSFORM.getTranslation());
    return new Pose2d(robotPosition, Rotation2d.kZero);
  }

  private boolean isInsideField(Pose2d pose) {
    return pose.getX() >= 0.0
        && pose.getX() <= FieldInfo.lengthMeters()
        && pose.getY() >= 0.0
        && pose.getY() <= FieldInfo.widthMeters();
  }

  private void publish(double distance) {
    SmartDashboard.putNumber("LookupTest/DistanceMeters", distance);
    SmartDashboard.putNumber("LookupTest/HoodDeg", ShooterLookup.getHoodMap().get(distance));
    SmartDashboard.putNumber(
        "LookupTest/FlywheelRPS", ShooterLookup.getFlywheelMap().get(distance));
    SmartDashboard.putNumber("LookupTest/TimeOfFlightSec", ShooterLookup.getToFMap().get(distance));
    SmartDashboard.putString(
        "LookupTest/DrivePose",
        poseValid ? getAlliancePose().toString() : "INVALID " + selectedBluePose);
  }
}
