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

  /** Makes this class non-instantiable. */
  private DriveCommands() {
  }

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
   * This method will create a command to spin the robot the specified amount at a
   * given speed. The robot
   * will always take the shortest path.
   * 
   * @param driveBase     The robot to spin.
   * @param Roationamount The amount the robot rotates.
   * @param speed         The speed to spin at. (must be a positive number greater
   *                      than 0).
   * @return The created command.
   */
  public static Command spinCommand(DriveBase driveBase, Rotation2d rotationAmount, double speed) {

    if (speed <= 0) {
      throw new RuntimeException("Robot cannot spin because velocity is negative or zero:  " + speed);
    }
    System.out.println("Spin robot: " + rotationAmount + "and" + speed);

    Command spinCommand = new Command() {

      private Rotation2d startingRotation, targetRotation;

      @Override
      public void initialize() {
        startingRotation = driveBase.getRotation();
        targetRotation = startingRotation.plus(rotationAmount);
      }

      @Override
      public void execute() {
        Rotation2d current = driveBase.getRotation();
        double delta = targetRotation.minus(current).getDegrees();
        double omega = Math.copySign(speed, delta);
        driveBase.runVelocity(new ChassisSpeeds(0, 0, omega));
      }

      @Override
      public boolean isFinished() {
        Rotation2d current = driveBase.getRotation();
        double delta = targetRotation.minus(current).getDegrees();
        return Math.abs(delta) < 1;
      }

      @Override
      public void end(boolean interrupted) {
        driveBase.stop();
      }
    };

    spinCommand.addRequirements(driveBase);

    return spinCommand;
  }
}