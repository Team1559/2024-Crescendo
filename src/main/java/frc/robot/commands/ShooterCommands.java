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

  public static Command reverseShooterCommand(Flywheel flywheel, Feeder feeder, Leds leds) {
    Command reverseShooterCommand = new Command() {
      @Override
      public void execute() {
        flywheel.reverseFlywheel();
        feeder.reverse();
        leds.setDynamicPattern(new Color[] { Color.kRed, Color.kRed, Color.kRed, Color.kRed, Color.kRed, Color.kBlack,
            Color.kBlack, Color.kBlack, Color.kBlack }, true);
      }

      @Override
      public void end(boolean interrupted) {
        flywheel.stopFlywheel();
        feeder.stop();
        leds.setAllianceColor();
      }
    };
    reverseShooterCommand.addRequirements(flywheel, feeder, leds);
    return reverseShooterCommand;
  }

  public static Command shootCommand(Flywheel flywheel, Feeder feeder, Leds leds, ColorSensor colorSensor) {
    //@formatter:off
    return new SequentialCommandGroup(
      spinUpFlywheelCommand(flywheel),
      feeder.startCommand(),
      leds.setColorCommand(Color.kMediumSpringGreen),
      colorSensor.waitForNoObjectCommand(),
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
