package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.led.LightsSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;

public class LightsCommands {

  /** Private to prevent instantiation. */
  private LightsCommands() {
  }

  /**
   * Dims/Brightens the lights
   * 
   * @param subsystem LED subsystem being changed
   * @param isDimming are lights being dimmed or brightened
   * @return
   */
  public static Command changeBrightness(LightsSubsystem subsystem, boolean isDimming) {
    return new InstantCommand(() -> subsystem.changeBrightness(isDimming), subsystem);
  }

  /**
   * Set color of the LEDs
   * 
   * @param subsystem LEDs being modified
   * @param color     Color LEDs are being set to
   * @return
   */
  public static Command setColor(LightsSubsystem subsystem, Color color) {
    Command command = new Command() {
      @Override
      public void initialize() {
        subsystem.setStaticColor(color);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
    command.addRequirements(subsystem);
    return command;
  }

  /**
   * Set the lights to a scrolling pattern
   * 
   * @param subsystem               LEDs being set
   * @param pattern                 Pattern the LEDs are being set to
   * @param isDynamicPatternFowards is the pattern scrolling fowards or backwards
   * @return
   */
  public static Command setDynamicPattern(LightsSubsystem subsystem, Color[] pattern, boolean isDynamicPatternFowards) {
    Command command = new Command() {
      @Override
      public void initialize() {
        subsystem.setDynamicPattern(pattern, isDynamicPatternFowards);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
    command.addRequirements(subsystem);
    return command;
  }

  /**
   * Sets a static patttern to the LEDs
   * 
   * @param subsystem LEDs being set
   * @param pattern   Pattern being set to the LEDs
   * @return
   */
  public static Command setStaticPattern(LightsSubsystem subsystem, Color[] pattern) {
    Command command = new Command() {
      @Override
      public void initialize() {
        subsystem.setStaticPattern(pattern);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
    command.addRequirements(subsystem);
    return command;
  }

  /**
   * Set LEDs to Alliance color
   * 
   * @param subsystem LEDs being set
   * @return
   */
  public static Command setToAllianceColor(LightsSubsystem subsystem) {
    Command command = new Command() {
      @Override
      public void initialize() {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
          subsystem.setStaticColor(Color.kAliceBlue);
        } else {
          subsystem.setStaticColor(Color.kRed);
        }

      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
    command.addRequirements(subsystem);
    return command;
  }

  /**
   * Blink the LEDs to specified Color and then return to Alliance color
   * 
   * @param subsystem
   * @param color
   * @return
   */
  public static Command blinkCommand(LightsSubsystem subsystem, Color color) {
    return new SequentialCommandGroup(setColor(subsystem, color), new WaitCommand(.5), setToAllianceColor(subsystem));
  }
}
