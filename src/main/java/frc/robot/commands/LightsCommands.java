// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LightsSubsystem;
import frc.robot.util.LightsManager;

import java.text.CollationElementIterator;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.util.Color;;

/** An example command that uses an example subsystem. */
public class LightsCommands extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LightsSubsystem m_subsystem;
  private Color color;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LightsCommands(LightsSubsystem subsystem, Color color) {
    m_subsystem = subsystem;
    this.color = color;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("A");
    m_subsystem.setStaticColor(color);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.clear();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
