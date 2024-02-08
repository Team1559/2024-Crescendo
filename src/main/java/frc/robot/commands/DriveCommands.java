package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.base.DriveBase;

public class DriveCommands {

  /** Makes this class non-instantiable. */
  private DriveCommands() {
  }

  public static Command autoAimAndManuallyDriveCommand(DriveBase driveBase,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> target) {

    Command aimingDrive = new Command() {

      PIDController pid = new PIDController(Constants.MAX_ANGULAR_SPEED / 50, 0, 0); // TODO: Tune.

      @Override
      public void initialize() {
        pid.setSetpoint(0);
        pid.setTolerance(1);
        pid.enableContinuousInput(-180, 180);
      }

      @Override
      public void execute() {

        // Apply deadband.
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
            Constants.JOYSTICK_DEADBAND);

        // Square values.
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Calcaulate new linear velocity.
        Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

        // Calculate omega velocity.
        double omega = pid
            .calculate(driveBase.getRotationToTarget(target.get()).plus(Rotation2d.fromDegrees(180)).getDegrees());
        omega = MathUtil.clamp(omega, -.1, .1); // TODO: Set Min & Max.

        // Scale Velocities to between 0 and Max.
        double scaledXVelocity = linearVelocity.getX() * driveBase.getMaxLinearSpeedMetersPerSec(),
            scaledYVelocity = linearVelocity.getY() * driveBase.getMaxLinearSpeedMetersPerSec(),
            // TODO: We are Clamping above, do we really also want to "scale" here?
            scaledOmegaVelocity = omega * driveBase.getMaxAngularSpeedRadPerSec();

        // Run Velocities.
        if (Constants.FIELD_RELATIVE) {
          driveBase.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(scaledXVelocity, scaledYVelocity,
              scaledOmegaVelocity, driveBase.getRotation()));
        } else {
          driveBase.runVelocity(new ChassisSpeeds(scaledXVelocity, scaledYVelocity, scaledOmegaVelocity));
        }

      }
    };
    aimingDrive.addRequirements(driveBase);
    return aimingDrive;
  }

  public static Command manualDriveDefaultCommand(DriveBase driveBase,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    return Commands.run(
        () -> {
          // Apply deadband.
          double linearMagnitude = MathUtil.applyDeadband(Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
              Constants.JOYSTICK_DEADBAND);
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.JOYSTICK_DEADBAND);

          // Square values.
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity.
          Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
              .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

          // Scale Velocities to between 0 and Max.
          double scaledXVelocity = linearVelocity.getX() * driveBase.getMaxLinearSpeedMetersPerSec(),
              scaledYVelocity = linearVelocity.getY() * driveBase.getMaxLinearSpeedMetersPerSec(),
              scaledOmegaVelocity = omega * driveBase.getMaxAngularSpeedRadPerSec();

          // Run Velocities.
          if (Constants.FIELD_RELATIVE) {
            driveBase.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(scaledXVelocity, scaledYVelocity,
                scaledOmegaVelocity, driveBase.getRotation()));
          } else {
            driveBase.runVelocity(new ChassisSpeeds(scaledXVelocity, scaledYVelocity, scaledOmegaVelocity));
          }
        },
        driveBase);
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

    Command spinCommand = new Command() {

      private Rotation2d startingRotation, targetRotation;

      @Override
      public void initialize() {
        startingRotation = driveBase.getRotation();
        // System.out.println("Rotation start: " + startingRotation);
        targetRotation = startingRotation.plus(rotationAmount);
        // System.out.println("Rotation amount: " + rotationAmount);
        // System.out.println("Target Rotation: " + targetRotation);
      }

      @Override
      public void execute() {
        Rotation2d current = driveBase.getRotation();
        double delta = targetRotation.minus(current).getDegrees();
        double rampOmega = Math.max(Math.min(Math.abs(delta) / 50, 1.0), .01);
        double omega = Math.copySign(speed, delta) * rampOmega;
        Logger.recordOutput("Spin/delta", delta);
        Logger.recordOutput("Spin/rampOmega", rampOmega);
        Logger.recordOutput("Spin/omega", omega);
        driveBase.runVelocity(new ChassisSpeeds(0, 0, omega));
      }

      @Override
      public boolean isFinished() {
        Rotation2d current = driveBase.getRotation();
        double delta = targetRotation.minus(current).getDegrees();
        return Math.abs(delta) < .5;
      }

      @Override
      public void end(boolean interrupted) {
        driveBase.stop();
      }
    };

    spinCommand.addRequirements(driveBase);

    return spinCommand;
  }

  // TODO: Create method to duplaicate turnToTargetCommand functionality using a
  // SwerveControllerCommand.
  // public static SwerveControllerCommand
  // turnToTargetSwerveControllerCommand(DriveBase driveBase, Translation2d
  // target, double speed)

  // TODO: Create method to duplaicate turnToTargetCommand functionality using a
  // PIDCommand.
  // public static PIDCommand turnToTargetPidCommand(DriveBase driveBase,
  // Translation2d target, double speed)

  public static Command turnToTargetCommand(DriveBase driveBase, Translation2d target, double speed) {

    Command spinCommand = new Command() {

      private Command spinCommand;

      @Override
      public void initialize() {
        Rotation2d rotation = driveBase.getRotationToTarget(target);
        spinCommand = spinCommand(driveBase, rotation, speed);
        spinCommand.initialize();
      }

      @Override
      public void execute() {
        spinCommand.execute();
      }

      @Override
      public boolean isFinished() {
        return spinCommand.isFinished();
      }

      @Override
      public void end(boolean interrupted) {
        spinCommand.end(interrupted);
      }
    };

    spinCommand.addRequirements(driveBase);

    return spinCommand;
  }

}