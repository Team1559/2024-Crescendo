package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.led.Leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;

public class LightsCommands {

  /** Private to prevent instantiation. */
  private LightsCommands() {
  }

  /**
   * Set LEDs to Alliance color
   * 
   * @param subsystem LEDs being set
   * @return
   */
  public static Command setToAllianceColor(Leds subsystem) {
    return new InstantCommand(() -> {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        subsystem.setStaticColor(Color.kBlue);
      } else {
        subsystem.setStaticColor(Color.kRed);
      }
    }, subsystem);
  }

  /**
   * Blink the LEDs to specified Color and then return to Alliance color
   * 
   * @param subsystem
   * @param color
   * @return
   */
  public static Command blinkCommand(Leds subsystem, Color color) {
    //@formatter:off
    return new SequentialCommandGroup(
      subsystem.setStaticColorCommand(color),
      new WaitCommand(.5),
      setToAllianceColor(subsystem));
    //@formatter:on
  }
}
