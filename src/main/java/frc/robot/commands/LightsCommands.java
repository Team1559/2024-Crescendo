package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LightsSubsystem;

import edu.wpi.first.wpilibj.util.Color;;

public class LightsCommands {

  /** Private to prevent instantiation. */
  private LightsCommands() {}

  public static Command changeBrightness(LightsSubsystem subsystem, boolean isDimming) {
    return new Command() {
      @Override
      public void execute() {
        subsystem.changeBrightness(isDimming);
      }
      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  public static Command setColor(LightsSubsystem subsystem, Color color) {
    return new Command() {
      @Override
      public void execute() {
        subsystem.setStaticColor(color);
      }
      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  public static Command setDynamicPattern(LightsSubsystem subsystem, Color[] pattern, boolean isDynamicPatternFowards) {
    return new Command() {
      @Override
      public void execute() {
        subsystem.setDynamicPattern(pattern, isDynamicPatternFowards);
      }
      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  public static Command setStaticPattern(LightsSubsystem subsystem, Color[] pattern) {
    return new Command() {
      @Override
      public void execute() {
        subsystem.setStaticPattern(pattern);
      }
      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }
}
