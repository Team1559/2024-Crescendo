package frc.robot.commands;

import java.time.Duration;
import java.time.LocalTime;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.Leds;

public class LedCommands {

  /** Private to prevent instantiation. */
  private LedCommands() {
  }

  /**
   * Blink the LEDs to specified Color and then return to Alliance color
   * 
   * @param leds  leds being set
   * @param color color being blinked to
   * @return
   */
  public static Command blinkCommand(Leds leds, Color color) {
    Duration WAIT_TIME = Duration.ofMillis(500);
    Command blinkCommand = new Command() {
      LocalTime startTime;

      @Override
      public void initialize() {
        leds.setStaticColor(color);
        startTime = LocalTime.now();
      }

      @Override
      public boolean isFinished() {
        Duration timeWaited = Duration.between(startTime, LocalTime.now());
        return timeWaited.compareTo(WAIT_TIME) >= 0;
      }

      @Override
      public void end(boolean interrupted) {
        leds.setAllianceColor(); // TODO - Layering colors
      }
    };
    blinkCommand.addRequirements(leds);
    return blinkCommand;
  }
}
