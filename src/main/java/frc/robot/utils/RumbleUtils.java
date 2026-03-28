package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleUtils {
  public static Command shortRumble(CommandXboxController joystick) {
    return mediumStrengthRumble(joystick).withTimeout(0.25).andThen(stopRumble(joystick));
  }

  public static Command mediumRumble(CommandXboxController joystick) {
    return mediumStrengthRumble(joystick).withTimeout(0.5).andThen(stopRumble(joystick));
  }

  public static Command longRumble(CommandXboxController joystick) {
    return mediumStrengthRumble(joystick).withTimeout(1).andThen(stopRumble(joystick));
  }

  @SuppressWarnings("unused")
  private static Command highStrengthRumble(CommandXboxController joystick) {
    return Commands.run(() -> joystick.setRumble(RumbleType.kBothRumble, 1.0)).withTimeout(2);
  }

  private static Command mediumStrengthRumble(CommandXboxController joystick) {
    return Commands.run(() -> joystick.setRumble(RumbleType.kBothRumble, 0.75)).withTimeout(2);
  }

  @SuppressWarnings("unused")
  private static Command lowStrengthRumble(CommandXboxController joystick) {
    return Commands.run(() -> joystick.setRumble(RumbleType.kBothRumble, 0.5)).withTimeout(2);
  }

  private static Command stopRumble(CommandXboxController joystick) {
    return Commands.run(() -> joystick.setRumble(RumbleType.kBothRumble, 0.0)).withTimeout(2);
  }
}
