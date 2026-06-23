package frc.robot.autonomous;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.FollowPath;
import frc.robot.utils.path.PathData;
import frc.robot.utils.path.Paths;
import frc.robot.utils.path.Paths.AlliancePath;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;

class AutoRoutineFactoryTest {
  @Test
  void resetPoseToStartUsesBluePathWhenAllianceIsNotRed() {
    AtomicReference<Pose2d> resetPose = new AtomicReference<>();
    AutoRoutineFactory factory =
        new AutoRoutineFactory(
            (path, configure) -> new InstantCommand(),
            (path, actions) -> new InstantCommand(),
            poseSupplier -> resetCommand(resetPose, poseSupplier),
            () -> false);

    factory.resetPoseToStart(Paths.LEFT_SIDE_AFTER_DEPOT_CLEANUP).initialize();

    assertEquals(Paths.LEFT_SIDE_AFTER_DEPOT_CLEANUP.getStartingPose(), resetPose.get());
  }

  @Test
  void resetPoseToStartUsesRedPathWhenAllianceIsRed() {
    AtomicReference<Pose2d> resetPose = new AtomicReference<>();
    AutoRoutineFactory factory =
        new AutoRoutineFactory(
            (path, configure) -> new InstantCommand(),
            (path, actions) -> new InstantCommand(),
            poseSupplier -> resetCommand(resetPose, poseSupplier),
            () -> true);

    factory.resetPoseToStart(Paths.LEFT_SIDE_AFTER_DEPOT_CLEANUP).initialize();

    assertEquals(
        Paths.LEFT_SIDE_AFTER_DEPOT_CLEANUP.mirrorForRedAlliance().getStartingPose(),
        resetPose.get());
  }

  @Test
  void resetPoseToStartSelectsAllianceWhenCommandRuns() {
    AtomicReference<Pose2d> resetPose = new AtomicReference<>();
    AtomicBoolean isRedAlliance = new AtomicBoolean(false);
    AutoRoutineFactory factory =
        new AutoRoutineFactory(
            (path, configure) -> new InstantCommand(),
            (path, actions) -> new InstantCommand(),
            poseSupplier -> resetCommand(resetPose, poseSupplier),
            isRedAlliance::get);

    Command command = factory.resetPoseToStart(Paths.LEFT_SIDE_AFTER_DEPOT_CLEANUP);
    isRedAlliance.set(true);
    command.initialize();

    assertEquals(
        Paths.LEFT_SIDE_AFTER_DEPOT_CLEANUP.mirrorForRedAlliance().getStartingPose(),
        resetPose.get());
  }

  @Test
  void pathAppliesConfigurationToBothAllianceVariants() {
    List<PathData> configuredPaths = new ArrayList<>();
    AtomicInteger configureCalls = new AtomicInteger();
    Consumer<FollowPath> configure = ignored -> configureCalls.incrementAndGet();
    AutoRoutineFactory factory =
        new AutoRoutineFactory(
            (path, pathConfigure) -> {
              assertSame(configure, pathConfigure);
              pathConfigure.accept(null);
              configuredPaths.add(path);
              return new InstantCommand();
            },
            (path, actions) -> new InstantCommand(),
            poseSupplier -> new InstantCommand(),
            () -> false);

    factory.path(Paths.LEFT_SIDE_AFTER_DEPOT_CLEANUP, configure);

    assertEquals(2, configuredPaths.size());
    assertEquals(2, configureCalls.get());
    assertEquals(
        Paths.LEFT_SIDE_AFTER_DEPOT_CLEANUP.mirrorForRedAlliance(), configuredPaths.get(0));
    assertSame(Paths.LEFT_SIDE_AFTER_DEPOT_CLEANUP, configuredPaths.get(1));
  }

  @Test
  void pathSelectsAllianceWhenCommandRuns() {
    AtomicReference<PathData> selectedPath = new AtomicReference<>();
    AtomicBoolean isRedAlliance = new AtomicBoolean(false);
    AutoRoutineFactory factory =
        new AutoRoutineFactory(
            (path, configure) -> new InstantCommand(() -> selectedPath.set(path)),
            (path, actions) -> new InstantCommand(),
            poseSupplier -> new InstantCommand(),
            isRedAlliance::get);

    Command command = factory.path(Paths.LEFT_SIDE_AFTER_DEPOT_CLEANUP);
    isRedAlliance.set(true);
    command.initialize();

    assertEquals(Paths.LEFT_SIDE_AFTER_DEPOT_CLEANUP.mirrorForRedAlliance(), selectedPath.get());
  }

  @Test
  void pathWithActionsSelectsAllianceWhenCommandRuns() {
    AtomicReference<PathData> selectedPath = new AtomicReference<>();
    AtomicBoolean isRedAlliance = new AtomicBoolean(false);
    AutoRoutineFactory factory =
        new AutoRoutineFactory(
            (path, configure) -> new InstantCommand(),
            (path, actions) -> new InstantCommand(() -> selectedPath.set(path)),
            poseSupplier -> new InstantCommand(),
            isRedAlliance::get);

    Command command = factory.pathWithActions(Paths.LEFT_SIDE_AFTER_DEPOT_CLEANUP, List.of());
    isRedAlliance.set(true);
    command.initialize();

    assertEquals(Paths.LEFT_SIDE_AFTER_DEPOT_CLEANUP.mirrorForRedAlliance(), selectedPath.get());
  }

  @Test
  void alliancePathIsCachedByPathIdentity() {
    PathData path = Paths.LEFT_SIDE_AFTER_DEPOT_CLEANUP;
    AutoRoutineFactory factory =
        new AutoRoutineFactory(
            (pathData, configure) -> new InstantCommand(),
            (pathData, actions) -> new InstantCommand(),
            poseSupplier -> new InstantCommand(),
            () -> false);

    AlliancePath first = factory.alliancePath(path);
    AlliancePath second = factory.alliancePath(path);

    assertSame(first, second);
    assertSame(path, first.blue());
    assertNotSame(path, first.red());
  }

  private static Command resetCommand(
      AtomicReference<Pose2d> resetPose, Supplier<Pose2d> poseSupplier) {
    return new InstantCommand(() -> resetPose.set(poseSupplier.get()));
  }
}
