package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveBase;
import frc.robot.subsystems.shooter.Aimer;
import frc.robot.subsystems.shooter.Flywheel;

public class DriveCommands {

    /** Makes Class non-instantiable */
    private DriveCommands() {
    }

    // ========================= Default Commands =========================

    public static Command manualDriveDefaultCommand(SwerveBase swerveBase, DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {

        return Commands.run(
                () -> {
                    double linearMagnitude = SwerveBase.calculateLinearMagnitude(xSupplier, ySupplier);
                    double omega = MathUtil.applyDeadband(-omegaSupplier.getAsDouble(),
                            Constants.getJoystickDeadband());
                    omega = Math.copySign(omega * omega, omega);

                    // Calculate new linear velocity.
                    Rotation2d linearDirection = SwerveBase.calculateLinearDirection(xSupplier, ySupplier);
                    Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

                    // Scale Velocities to between 0 and Max.
                    Measure<Velocity<Distance>> scaledXVelocity = Constants.getMaxLinearSpeed()
                            .times(linearVelocity.getX());
                    Measure<Velocity<Distance>> scaledYVelocity = Constants.getMaxLinearSpeed()
                            .times(linearVelocity.getY());
                    Measure<Velocity<Angle>> scaledOmegaVelocity = Constants.getMaxAngularSpeed().times(omega);

                    // Run Velocities.
                    if (Constants.isDrivingModeFieldRelative()) {
                        swerveBase.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(scaledXVelocity, scaledYVelocity,
                                scaledOmegaVelocity, swerveBase.getRotation()));
                    } else {
                        swerveBase
                                .runVelocity(new ChassisSpeeds(scaledXVelocity, scaledYVelocity, scaledOmegaVelocity));
                    }
                },
                swerveBase);
    }

    // ========================= Other Commands =========================

    public static Command autoAimAndManuallyDriveCommand(SwerveBase swerveBase, Flywheel flywheel, Aimer aimer,
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Translation3d> target) {

        Command aimingDrive = new Command() {

            PIDController pid;

            @Override
            public void initialize() {
                // TODO: Use same units when calculating KP.
                pid = new PIDController(Constants.getMaxAngularSpeed().in(RadiansPerSecond) / 90 /* degrees */, 0, 0);
                pid.setSetpoint(0); // Degrees from target.
                pid.setTolerance(1/* degree(s) */);
                pid.enableContinuousInput(-180, 180); // Degrees.
                if (Constants.hasFlywheelSubsystem())
                    flywheel.start();
            }

            @Override
            public void execute() {
                double linearMagnitude = SwerveBase.calculateLinearMagnitude(xSupplier, ySupplier);
                // Calcaulate new linear velocity.
                Rotation2d linearDirection = SwerveBase.calculateLinearDirection(xSupplier, ySupplier);
                Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

                // Calculate omega velocity.
                double degreesToTarget = swerveBase.getRotationToTarget(target.get().toTranslation2d())
                        .plus(Rotation2d.fromDegrees(180))
                        .getDegrees();
                /*
                 * Range:
                 * - If kd = 0: minimumInput * kp - ki <-> maximumInput * kp + ki.
                 * - If kd != 0: -Double.MAX_VALUE <-> Double.MAX_VALUE.
                 */
                // We are inverting the direction because degreesToTarget is our "correction",
                // but the PIDController wants our "position".
                double omega = pid.calculate(-degreesToTarget);

                omega = MathUtil.clamp(omega, // TODO: Use same units when calculating KP.
                        -Constants.getMaxAngularSpeed().in(RadiansPerSecond),
                        Constants.getMaxAngularSpeed().in(RadiansPerSecond));

                // Scale Velocities to between 0 and Max.
                double scaledXVelocity = linearVelocity.getX() * Constants.getMaxLinearSpeed().in(MetersPerSecond),
                        scaledYVelocity = linearVelocity.getY() * Constants.getMaxLinearSpeed().in(MetersPerSecond);

                // Run Velocities.
                if (Constants.isDrivingModeFieldRelative()) {
                    swerveBase.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(scaledXVelocity, scaledYVelocity,
                            omega, swerveBase.getRotation()));
                } else {
                    swerveBase.runVelocity(new ChassisSpeeds(scaledXVelocity, scaledYVelocity, omega));
                }

                aimer.aimAtTarget(target.get(), swerveBase.getEstimatedPosition().getTranslation());

                // TODO: Add Turning LEDs to Green, when close enough to shoot.

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
                // if it has not been given a command for a X second(s).
                pid.close();
            }

        };
        aimingDrive.addRequirements(swerveBase);
        if (Constants.hasFlywheelSubsystem())
            aimingDrive.addRequirements(flywheel);
        return aimingDrive;
    }

    /**
     * This method will create a command to spin the robot the specified amount at a
     * given speed. The robot
     * will always take the shortest path.
     * 
     * @param driveBase      The robot to spin.
     * @param rotationAmount The amount the robot rotates.
     * @param speed          The speed to spin at. (must be a positive number
     *                       greater
     *                       than 0).
     * @return The created command.
     */
    public static Command spinCommand(SwerveBase swerveBase, Rotation2d rotationAmount, double speed) {

        if (speed <= 0) {
            throw new RuntimeException("Robot cannot spin because velocity is negative or zero:  " + speed);
        }

        Command spinCommand = new Command() {

            private Rotation2d targetRotation;

            @Override
            public void initialize() {
                Rotation2d startingRotation = swerveBase.getRotation();
                targetRotation = startingRotation.plus(rotationAmount);
            }

            @Override
            public void execute() {

                Rotation2d current = swerveBase.getRotation();
                double delta = targetRotation.minus(current).getDegrees();

                double rampOmega = Math.max(Math.min(Math.abs(delta) / 50 /* degrees */, 1.0), .01);
                double omega = Math.copySign(speed, delta) * rampOmega;

                swerveBase.runVelocity(new ChassisSpeeds(0, 0, omega));

                Logger.recordOutput("DriveCommands/spinCommand/delta", delta);
                Logger.recordOutput("DriveCommands/spinCommand/rampOmega", rampOmega);
                Logger.recordOutput("DriveCommands/spinCommand/omega", omega);
            }

            @Override
            public boolean isFinished() {
                Rotation2d current = swerveBase.getRotation();
                double delta = targetRotation.minus(current).getDegrees();
                return Math.abs(delta) < .5 /* degrees */;
            }

            @Override
            public void end(boolean interrupted) {
                swerveBase.stop();
            }
        };

        spinCommand.addRequirements(swerveBase);

        return spinCommand;
    }

    public static Command turnToTargetCommand(SwerveBase swerveBase, Supplier<Translation3d> target, double speed) {

        Command spinCommand = new Command() {

            private Command spinCommand;

            @Override
            public void initialize() {
                // Rotating plus 180 degrees to position the back of the robot to the target.
                Rotation2d rotation = swerveBase.getRotationToTarget(target.get().toTranslation2d())
                        .plus(Rotation2d.fromDegrees(180));
                spinCommand = spinCommand(swerveBase, rotation, speed);
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

        spinCommand.addRequirements(swerveBase);

        return spinCommand;
    }
}
