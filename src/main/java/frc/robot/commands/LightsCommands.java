package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LightsSubsystem;
import edu.wpi.first.wpilibj.util.Color;;

/** An example command that uses an example subsystem. */
public class LightsCommands {
  // ---Private to prevent instantiation ---
  private LightsCommands() {
  }

  public static Command setColor(LightsSubsystem subsystem, Color color) {
    Command command = new Command() {
      @Override
      public void initialize() {
        subsystem.setStaticColor(color);
      }

      @Override
      public void execute() {
        
      }

      @Override
      public boolean isFinished() {
        return true;
      }

      @Override
      public void end(boolean interrupted) {
      }
    };
    return command;
  }
  public static Command changeBrightness(LightsSubsystem subsystem, boolean isDimming) {
    Command command = new Command() {
      @Override
      public void initialize() {

      }

      @Override
      public void execute() {
        subsystem.changeBrightness(isDimming);
      }

      @Override
      public boolean isFinished() {
        return true;
      }

      @Override
      public void end(boolean interrupted) {
      }
    };
    return command;
  }
}
