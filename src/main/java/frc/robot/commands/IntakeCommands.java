package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.led.Leds;
import frc.robot.subsystems.shooter.ColorSensor;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Intake;

public class IntakeCommands {

  /** Makes Class non-instantiable */
  private IntakeCommands() {
  }

  public static Command defaultIntakeCommand(Intake intake, ColorSensor sensor) {
    return Commands.run(() -> {
      if (sensor.isObjectDetected()) {
        intake.stop();
      } else {
        intake.start();
      }
    }, intake);
  }

  public static Command defaultFeederCommand(Feeder feeder, ColorSensor sensor) {
    return Commands.run(() -> {
      if (sensor.isObjectDetected()) {
        feeder.stop();
      } else {
        feeder.start();
      }
    }, feeder);
  }

  public static Command stopIntakeFeederCommand(Intake intake, Feeder feeder) {
    return new InstantCommand(() -> {
      intake.stop();
      feeder.stop();
    }, intake, feeder);
  }
}
