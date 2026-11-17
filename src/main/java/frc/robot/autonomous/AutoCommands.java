package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToPoint;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.FieldInfo;
import frc.robot.utils.geometry.ExtPose;
import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathSample;
import frc.team5712.path.core.Pose;
import frc.team5712.path.follow.FollowerConfig;
import frc.team5712.path.follow.PathFollower;
import frc.team5712.path.wpilib.Conversions;
import frc.team5712.path.wpilib.FollowPathCommandV2;
import frc.team5712.path.wpilib.PathId;
import frc.team5712.path.wpilib.PathLibrary;
import frc.team5712.path.wpilib.Pathfinding;
import java.util.function.Supplier;

/**
 * Reusable command builders for autonomous routines.
 *
 * <p>Paths come from the 5712 path library (Myelin, paths.frc5712.com): every {@code
 * deploy/paths/*.path.json} is built once here, at robot init, and named with the editor's
 * generated {@code PathEditorNames.Paths} constants. A path is drawn for blue and flipped for red
 * when it starts (a half turn about the field's centre), so routines don't build both.
 */
public class AutoCommands {

  /** How red paths are flipped: the 2026 field is rotationally symmetric. */
  public static final FieldFrame RED_FRAME = FieldFrame.RED_HALF_TURN;

  /** Heading tolerance when a routine asks for a looser finish (the old follower checked none). */
  private static final double LOOSE_HEADING_TOLERANCE_RAD = Math.toRadians(10);

  private final CommandSwerveDrivetrain drivetrain;
  private final PathLibrary paths;
  private final FollowerConfig followerConfig;
  private final Pathfinding pathfinding;

  private final SwerveRequest.ApplyFieldSpeeds pathRequest =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  public AutoCommands(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.paths = PathLibrary.loadAll();
    this.followerConfig = FollowerConfig.defaults(paths.robot());
    this.pathfinding =
        new Pathfinding(
            paths,
            followerConfig,
            drivetrain::getPose,
            drivetrain::getFieldSpeeds,
            this::driveFieldSpeeds,
            RED_FRAME);
  }

  private void driveFieldSpeeds(ChassisSpeeds fieldSpeeds) {
    drivetrain.setControl(pathRequest.withSpeeds(fieldSpeeds));
  }

  /** Every loaded path, plus the field and pathfinder, for anything not wrapped here. */
  public PathLibrary paths() {
    return paths;
  }

  /** On-the-fly paths around field.json's obstacles ({@code pathfinding().toV2(...)} etc.). */
  public Pathfinding pathfinding() {
    return pathfinding;
  }

  public DriveToPoint driveTo(Supplier<Pose2d> pose) {
    return new DriveToPoint(drivetrain, pose);
  }

  // ==================== Path following ====================

  /** Follows a library path with the default finish (2 cm, 1°, settled). Flipped on red. */
  public FollowPathCommandV2 followPath(PathId path) {
    return pathfinding.followV2(drivetrain, path);
  }

  /**
   * Follows a library path, finishing once within {@code positionToleranceMeters} of the end (and
   * 10° of its heading, settled), for routines that would rather move on than settle exactly.
   */
  public FollowPathCommandV2 followPath(PathId path, double positionToleranceMeters) {
    FollowerConfig loose =
        followerConfig.toBuilder()
            .positionToleranceMeters(positionToleranceMeters)
            .headingToleranceRadians(LOOSE_HEADING_TOLERANCE_RAD)
            .build();
    return new FollowPathCommandV2(
            path,
            drivetrain,
            new PathFollower(paths.get(path), loose),
            drivetrain::getPose,
            drivetrain::getFieldSpeeds,
            this::driveFieldSpeeds,
            RED_FRAME)
        .withEvents(paths);
  }

  /** Where {@code path} starts, for the alliance the driver station reports now (2026 frame). */
  public Pose2d startPose(PathId path) {
    return poseAt(path, 0.0);
  }

  /** Where {@code path} ends, for the alliance the driver station reports now (2026 frame). */
  public Pose2d endPose(PathId path) {
    Path p = paths.get(path);
    return poseAt(path, p.lengthMeters());
  }

  private Pose2d poseAt(PathId path, double s) {
    PathSample sample = paths.get(path).sample(s);
    Pose blue = new Pose(sample.xMeters(), sample.yMeters(), sample.headingRadians());
    return Conversions.toPose2d(Conversions.allianceFrame(RED_FRAME).fromBlue(blue));
  }

  public Command resetPose(Supplier<Pose2d> pose) {
    return drivetrain.runOnce(() -> drivetrain.resetPose(pose.get()));
  }

  /** Resets odometry to the start of {@code path} (alliance read when it runs). */
  public Command resetPoseToStart(PathId path) {
    return resetPose(() -> startPose(path));
  }

  // ==================== Time-Triggered Actions ====================

  /**
   * Returns a command that waits until the robot passes a given X position, then runs a command.
   * The threshold is specified in blue-alliance coordinates and is automatically flipped for red
   * alliance using FieldInfo.flipX().
   *
   * @param blueAllianceX X position threshold in blue-alliance coordinates
   * @param commandToRun Command to run once the robot passes the threshold
   * @return Command that triggers based on robot X position
   */
  public Command runWhenPastX(double blueAllianceX, Command commandToRun) {
    return Commands.sequence(
        Commands.waitUntil(() -> FieldInfo.flipX(drivetrain.getPose().getX()) > blueAllianceX),
        commandToRun);
  }

  public Command leftAutoSetup() {
    return resetPose(
        () ->
            new ExtPose(4.378, FieldInfo.width().in(Meters) - 0.639445, Rotation2d.fromDegrees(-90))
                .get());
  }
}
