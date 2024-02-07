package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.led.LightsSubsystem;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;

public class ShooterCommands {
  /** Makes Class non-instantiable */
  private ShooterCommands() {
  }

  // TODO
  public static Command shootCommand(Flywheel flywheel, Feeder feeder, LightsSubsystem LEDs) {
    return new SequentialCommandGroup(
        flywheel.startFlywheelCommand(12), new WaitCommand(0.5),
        feeder.startCommand(), LightsCommands.setColor(LEDs, Color.kDarkViolet),
        new WaitCommand(1), flywheel.stopFlywheelCommand(), feeder.stopCommand(),
        LightsCommands.setToAllianceColor(LEDs)

    // -Turn on FLyhwheels first.
    // -Give wheels time to come up to speed.
    // -Turn on feed motor so that the note gets pushed forward into the flywheel.
    // *Start LED pattern for launch.*
    // -Give time for note to launch.
    // -After note launches, turn off the motors and LEDs.
    );
  }
}
