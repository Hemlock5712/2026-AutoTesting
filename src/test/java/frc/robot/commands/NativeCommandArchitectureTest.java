package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.regex.Pattern;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.function.Executable;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;

class NativeCommandArchitectureTest {
  private static final List<Class<? extends Command>> COMMAND_CLASSES =
      List.of(
          AxisLockDrive.class,
          DriveToPoint.class,
          FollowPath.class,
          GamePieceDrive.class,
          JamProtectedShoot.class,
          OrbitDrive.class,
          TurretDrive.class);

  private static final Path COMMAND_SOURCE_DIRECTORY =
      Path.of("src", "main", "java", "frc", "robot", "commands");
  private static final String LEGACY_ADAPTER_TYPE = "CommandLifecycle" + "Adapter";
  private static final Pattern RUN_LOOP = Pattern.compile("while\\s*\\(\\s*true\\s*\\)");
  private static final Pattern COROUTINE_YIELD =
      Pattern.compile("coroutine\\s*\\.\\s*yield\\s*\\(\\s*\\)\\s*;");
  private static final Pattern LEGACY_LIFECYCLE_METHOD =
      Pattern.compile(
          "(?m)^\\s*(?:public|protected)\\s+(?:final\\s+)?(?:void\\s+"
              + "(?:initialize|execute|end)|boolean\\s+isFinished)\\s*\\(");

  @Test
  void allConvertedCommandsDeclareTheNativeCommandContractDirectly() {
    assertEquals(7, COMMAND_CLASSES.size());

    assertAll(
        COMMAND_CLASSES.stream()
            .map(
                commandClass ->
                    (Executable)
                        () -> {
                          String context = commandClass.getSimpleName();
                          assertEquals(
                              Object.class,
                              commandClass.getSuperclass(),
                              context + " must not inherit a command base class");
                          assertTrue(
                              Arrays.asList(commandClass.getInterfaces()).contains(Command.class),
                              context + " must directly implement Commands v3 Command");

                          assertDeclaredPublicMethod(
                              commandClass, "run", void.class, Coroutine.class);
                          assertDeclaredPublicMethod(commandClass, "onCancel", void.class);
                          assertDeclaredPublicMethod(commandClass, "name", String.class);
                          assertDeclaredPublicMethod(commandClass, "requirements", Set.class);
                        }));
  }

  @Test
  void convertedRunLoopsYieldAndCommandPackageHasNoAdapterResidue() throws IOException {
    assertTrue(
        Files.isDirectory(COMMAND_SOURCE_DIRECTORY),
        "command source directory must resolve from the Gradle project root");

    for (Class<? extends Command> commandClass : COMMAND_CLASSES) {
      Path sourceFile = COMMAND_SOURCE_DIRECTORY.resolve(commandClass.getSimpleName() + ".java");
      String source = Files.readString(sourceFile, StandardCharsets.UTF_8);
      String context = commandClass.getSimpleName();

      assertEquals(
          1,
          RUN_LOOP.matcher(source).results().count(),
          context + " must have one continuing coroutine run loop");
      assertEquals(
          1,
          COROUTINE_YIELD.matcher(source).results().count(),
          context + " must yield once from its continuing run loop");
      assertFalse(
          LEGACY_LIFECYCLE_METHOD.matcher(source).find(),
          context + " must not retain legacy lifecycle methods");
    }

    try (Stream<Path> packageSources = Files.list(COMMAND_SOURCE_DIRECTORY)) {
      assertAll(
          packageSources
              .filter(path -> path.getFileName().toString().endsWith(".java"))
              .map(
                  path ->
                      (Executable)
                          () -> {
                            String source = Files.readString(path, StandardCharsets.UTF_8);
                            assertFalse(
                                path.getFileName().toString().equals(LEGACY_ADAPTER_TYPE + ".java"),
                                "legacy adapter source must be deleted");
                            assertFalse(
                                source.contains(LEGACY_ADAPTER_TYPE),
                                path.getFileName() + " must not reference the legacy adapter");
                          }));
    }
  }

  private static void assertDeclaredPublicMethod(
      Class<?> commandClass, String methodName, Class<?> returnType, Class<?>... parameterTypes)
      throws NoSuchMethodException {
    Method method = commandClass.getDeclaredMethod(methodName, parameterTypes);
    assertTrue(
        Modifier.isPublic(method.getModifiers()),
        commandClass.getSimpleName() + "." + methodName + " must be public");
    assertEquals(
        returnType,
        method.getReturnType(),
        commandClass.getSimpleName() + "." + methodName + " has the wrong return type");
  }
}
