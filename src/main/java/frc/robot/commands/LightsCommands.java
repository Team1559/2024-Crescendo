package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LightsSubsystem;
import edu.wpi.first.wpilibj.util.Color;;

/** An example command that uses an example subsystem. */
public class LightsCommands extends Command {
  
  private final LightsSubsystem lightSubsystem;
  private final Color color;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LightsCommands(LightsSubsystem lightSubsystem, Color color) {
    this.lightSubsystem = lightSubsystem;
    this.color = color;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lightSubsystem);
  }

  @Override
  public void initialize() {
    lightSubsystem.setStaticColor(color);
  }

  @Override
  public void execute(){
   
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}
