package frc.robot.utils;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.GenericHID.RumbleType;

public class RumbleUtils {
  public static Command shortRumble(CommandGamepad joystick) {
    return mediumStrengthRumble(joystick)
        .withTimeout(Seconds.of(0.25))
        .andThen(stopRumble(joystick))
        .withAutomaticName();
  }

  public static Command mediumRumble(CommandGamepad joystick) {
    return mediumStrengthRumble(joystick)
        .withTimeout(Seconds.of(0.5))
        .andThen(stopRumble(joystick))
        .withAutomaticName();
  }

  public static Command longRumble(CommandGamepad joystick) {
    return mediumStrengthRumble(joystick)
        .withTimeout(Seconds.of(1))
        .andThen(stopRumble(joystick))
        .withAutomaticName();
  }

  @SuppressWarnings("unused")
  private static Command highStrengthRumble(CommandGamepad joystick) {
    return Commands.run(() -> setRumble(joystick, 1.0)).withTimeout(Seconds.of(2));
  }

  private static Command mediumStrengthRumble(CommandGamepad joystick) {
    return Commands.run(() -> setRumble(joystick, 0.75)).withTimeout(Seconds.of(2));
  }

  @SuppressWarnings("unused")
  private static Command lowStrengthRumble(CommandGamepad joystick) {
    return Commands.run(() -> setRumble(joystick, 0.5)).withTimeout(Seconds.of(2));
  }

  private static Command stopRumble(CommandGamepad joystick) {
    return Commands.run(() -> setRumble(joystick, 0.0)).withTimeout(Seconds.of(2));
  }

  private static void setRumble(CommandGamepad joystick, double value) {
    joystick.getHID().setRumble(RumbleType.LEFT_RUMBLE, value);
    joystick.getHID().setRumble(RumbleType.RIGHT_RUMBLE, value);
  }
}
