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
import frc.robot.constants.NewConstants;
import frc.robot.subsystems.base.DriveBase;
import frc.robot.subsystems.shooter.Flywheel;

public class DriveCommands {

    /** Makes this class non-instantiable. */
    private DriveCommands() {
    }

    // TODO: Deduplicate code between this and the manualDriveDefaultCommand method.
    public static Command autoAimAndManuallyDriveCommand(DriveBase driveBase,
            Flywheel flywheel,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Translation2d> target) {

        Command aimingDrive = new Command() {

            PIDController pid;

            @Override
            public void initialize() {
                pid = new PIDController(Constants.MAX_ANGULAR_SPEED_IN_RADS_PER_SECONDS / 90 /* degrees */, 0, 0);
                pid.setSetpoint(0); // Degrees from target.
                pid.setTolerance(1/* degree(s) */);
                pid.enableContinuousInput(-180, 180); // Degrees.
                flywheel.start();
            }

            @Override
            public void execute() {

                // Apply deadband.
                double linearMagnitude = MathUtil.applyDeadband(
                        Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                        Constants.JOYSTICK_DEADBAND);

                // Square values, to accelerate mor gradually.
                linearMagnitude = linearMagnitude * linearMagnitude;

                // Calcaulate new linear velocity.
                Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

                // Calculate omega velocity.
                double degreesToTarget = driveBase.getRotationToTarget(target.get()).plus(Rotation2d.fromDegrees(180))
                        .getDegrees();
                /*
                 * Range:
                 * - If kd = 0: minimumInput * kp - ki <-> maximumInput * kp + ki.
                 * - If kd != 0: -Double.MAX_VALUE <-> Double.MAX_VALUE.
                 */
                // We are inverting the direction because degreesToTarget is our "correction",
                // but the PIDController wants our "position".
                double omega = pid.calculate(-degreesToTarget);

                omega = MathUtil.clamp(omega,
                        -Constants.MAX_ANGULAR_SPEED_IN_RADS_PER_SECONDS,
                        Constants.MAX_ANGULAR_SPEED_IN_RADS_PER_SECONDS);

                // Scale Velocities to between 0 and Max.
                double scaledXVelocity = linearVelocity.getX() * Constants.MAX_LINEAR_SPEED_IN_METERS_PER_SECOND,
                        scaledYVelocity = linearVelocity.getY() * Constants.MAX_LINEAR_SPEED_IN_METERS_PER_SECOND;

                // Run Velocities.
                if (NewConstants.get().isDrivingModeFieldRelative()) {
                    driveBase.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(scaledXVelocity, scaledYVelocity,
                            omega, driveBase.getRotation()));
                } else {
                    driveBase.runVelocity(new ChassisSpeeds(scaledXVelocity, scaledYVelocity, omega));
                }

                // Log Calculated Values.
                Logger.recordOutput("DriveCommands/autoAimAndManuallyDriveCommand/degreesToTarget", degreesToTarget);
                Logger.recordOutput("DriveCommands/autoAimAndManuallyDriveCommand/vxMetersPerSecond", scaledXVelocity);
                Logger.recordOutput("DriveCommands/autoAimAndManuallyDriveCommand/vyMetersPerSecond", scaledYVelocity);
                Logger.recordOutput("DriveCommands/autoAimAndManuallyDriveCommand/omegaRadiansPerSecond", omega);
            }

            @Override
            public boolean isFinished() {
                // Never stop, because this command will be used as a While True command.
                return false;
            }

            @Override
            public void end(boolean interrupted) {
                // No need to tell the motors to stop, because the default command will kick in.
                // TODO: Create a default command for the Flywheel, that will stop the flywheel
                // if it has not been given a command for a X second(s).
                pid.close();
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
                    double linearMagnitude = MathUtil.applyDeadband(
                            Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                            Constants.JOYSTICK_DEADBAND);
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.JOYSTICK_DEADBAND);

                    // Square values, to accelerate mor gradually.
                    linearMagnitude = linearMagnitude * linearMagnitude;
                    omega = Math.copySign(omega * omega, omega);

                    // Calcaulate new linear velocity.
                    Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                    Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

                    // Scale Velocities to between 0 and Max.
                    double scaledXVelocity = linearVelocity.getX() * Constants.MAX_LINEAR_SPEED_IN_METERS_PER_SECOND,
                            scaledYVelocity = linearVelocity.getY() * Constants.MAX_LINEAR_SPEED_IN_METERS_PER_SECOND,
                            scaledOmegaVelocity = omega * Constants.MAX_ANGULAR_SPEED_IN_RADS_PER_SECONDS;
                    ;

                    // Run Velocities.
                    if (NewConstants.get().isDrivingModeFieldRelative()) {
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

            private Rotation2d targetRotation;

            @Override
            public void initialize() {
                Rotation2d startingRotation = driveBase.getRotation();
                targetRotation = startingRotation.plus(rotationAmount);
            }

            @Override
            public void execute() {

                Rotation2d current = driveBase.getRotation();
                double delta = targetRotation.minus(current).getDegrees();

                double rampOmega = Math.max(Math.min(Math.abs(delta) / 50 /* degrees */, 1.0), .01);
                double omega = Math.copySign(speed, delta) * rampOmega;

                driveBase.runVelocity(new ChassisSpeeds(0, 0, omega));

                Logger.recordOutput("DriveCommands/spinCommand/delta", delta);
                Logger.recordOutput("DriveCommands/spinCommand/rampOmega", rampOmega);
                Logger.recordOutput("DriveCommands/spinCommand/omega", omega);
            }

            @Override
            public boolean isFinished() {
                Rotation2d current = driveBase.getRotation();
                double delta = targetRotation.minus(current).getDegrees();
                return Math.abs(delta) < .5 /* degrees */;
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
                // Rotating plus 180 degrees to postion the back of the robot to the target.
                Rotation2d rotation = driveBase.getRotationToTarget(target).plus(Rotation2d.fromDegrees(180));
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