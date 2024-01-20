package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.base.DriveBase;

import java.util.function.DoubleSupplier;

public class DriveCommands {

  /** Makes this class non-instantiable.*/
  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and
   * angular velocities).
   */
  public static Command joystickDrive(DriveBase drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude = MathUtil.applyDeadband(
              Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
              Constants.JOYSTICK_DEADBAND);
          Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.JOYSTICK_DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
              .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
              .getTranslation();

          if (Constants.FIELD_RELATIVE) {
            // Convert to field relative speeds & send command
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    omega * drive.getMaxAngularSpeedRadPerSec(),
                    drive.getRotation()));
          } else {
            drive.runVelocity(
                new ChassisSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    omega * drive.getMaxAngularSpeedRadPerSec()));
          }
        },
        drive);
  }

  /**
   * This method will create a command to spin the robot the specified number of degrees at the specified velocity.
   * 
   * @param driveBase The robot to spin.
   * @param degrees The number of degrees to spin (must be a positive number).
   * @param velocity The velocity to spin at. (must be a positive number).
   * @return The created command.
   */
  public static Command spinCommand(DriveBase driveBase, double degrees, double velocity) {

    if(degrees <= 0) {
      throw new RuntimeException("Spin Degrees is not a posotive number: " + degrees);
    }
    if(velocity <= 0) {
      throw new RuntimeException("Spin Velocity is not a posotive number: " + velocity);
    }

    Command spinCommand = new Command() {

      @Override
      public void initialize() {

      }

      @Override
      public void execute() {

      }

      @Override
      public void end(boolean interrupted) {

      }

      @Override
      public boolean isFinished() {
        return false;
      }
    };

    spinCommand.addRequirements(driveBase);
    
    return spinCommand;
  }
}