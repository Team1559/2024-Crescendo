package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Flywheel;

public class ShooterCommands {
    /** Makes Class non-instantiable */
    private ShooterCommands() {
    }

    public static Command enableShooter(Flywheel subsystem) {
        Command command = new Command() {
            @Override
            public void execute() {
              subsystem.setFlywheelSpeed(0);
            }
            @Override
            public boolean isFinished() {
              return true;
            }
          };
        command.addRequirements(subsystem);
        return command;
    }
    public static Command disableShooter(Flywheel subsystem) {
        Command command = new Command() {
            @Override
            public void execute() {
              subsystem.disableShooter();
            }
            @Override
            public boolean isFinished() {
              return true;
            }
          };
        command.addRequirements(subsystem);
        return command;
    }
}
