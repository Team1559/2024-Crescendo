package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.led.Leds;
import frc.robot.subsystems.shooter.ColorSensor;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;

public class ShooterCommands {

  /** Makes Class non-instantiable */
  private ShooterCommands() {
  }

  // TODO: Add Reverse Commands. (Spins Flywheel & Feeder)

  public static Command shootCommand(Flywheel flywheel, Feeder feeder, Leds LEDs, ColorSensor sensor) {
  //@formatter:off
    return new SequentialCommandGroup(
      spinUpFlywheelCommand(flywheel),
      feeder.startCommand(),
      LightsCommands.blinkCommand(LEDs, Color.kOrange),
      sensor.waitForNoObjectCommand(),
      new WaitCommand(.25),
      feeder.stopCommand(),
      flywheel.stopFlywheelCommand()
    );
    //@formatter:on
  }

  public static Command spinUpFlywheelCommand(Flywheel flywheel) {
    //@formatter:off
    return new SequentialCommandGroup(
      flywheel.startFlywheelCommand(), 
      new WaitCommand(0.5)
      );
    //@formatter:on
  }
}
