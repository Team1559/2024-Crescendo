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
   * This method will create a command to spin the robot the specified number of
   * degrees at the specified velocity.
   * 
   * @param driveBase The robot to spin.
   * @param degrees   The number of degrees to spin (must be a positive number).
   * @param velocity  The velocity to spin at. (must be a positive number).
   * @return The created command.
   */
  public static Command spinCommand(DriveBase driveBase, double degrees, double velocity) {

    if (degrees > 180 || degrees < -180) {
      throw new RuntimeException("Spin Degrees has exceeded 180 or -180: " + degrees);
    }
    if (velocity == 0) {
      throw new RuntimeException("Robot cannot spin because velocity is:  " + velocity);
    }
    System.out.println("Spin robot: " + degrees + "and" + velocity);

    Command spinCommand = new Command() {

      private boolean crossedResetAngle = false;
      private Rotation2d startingRotation, targetRotation;
      private boolean isStartSignPositive; 
      @Override
      public void initialize() {
        startingRotation = driveBase.getRotation();
        isStartSignPositive = startingRotation.getDegrees() >= 0;
        targetRotation = startingRotation.plus(Rotation2d.fromDegrees(degrees));
        System.out.println("Spin robot after intialize.");
      }

      @Override
      public void execute() {
        System.out.println("Spin robot execute ");
        driveBase.runVelocity(new ChassisSpeeds(0, 0, velocity));

      }

      @Override
      public boolean isFinished() {
        double startRotationInDegrees = startingRotation.getDegrees() + 180;
        double targetRotationInDegrees = targetRotation.getDegrees() + 180;
        double currentRotationInDegrees = driveBase.getRotation().getDegrees() + 180;
        System.out.println(currentRotationInDegrees + " : " + targetRotationInDegrees);
        
        //if start + distance is beyond degrees of circle we crossed the barrier
        if (startRotationInDegrees < targetRotationInDegrees) {
          return currentRotationInDegrees >= targetRotationInDegrees
              || currentRotationInDegrees < startRotationInDegrees - 1;
        } else {
          if (crossedResetAngle) {
            return currentRotationInDegrees >= targetRotationInDegrees;
          } else {
            crossedResetAngle = (driveBase.getRotation().getDegrees() >= 0) != isStartSignPositive;
            return false;
          }
        }
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