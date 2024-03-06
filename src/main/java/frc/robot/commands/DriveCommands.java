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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveBase;
import frc.robot.subsystems.shooter.Aimer;
import frc.robot.subsystems.shooter.Flywheel;

public class DriveCommands {

    /** Makes this class non-instantiable. */
    private DriveCommands() {
    }

    public static class AutoAimDriveCommand extends Command {
        private final SwerveBase driveBase;
        private final Flywheel flywheel;
        private final Aimer aimer;

        private final DoubleSupplier xVelocity;
        private final DoubleSupplier yVelocity;
        private final Supplier<Translation3d> target;

        private final PIDController pid;

        public AutoAimDriveCommand(SwerveBase driveBase, Flywheel flywheel, Aimer aimer, DoubleSupplier xVelocity,
                DoubleSupplier yVelocity,
                Supplier<Translation3d> target) {
            this.driveBase = driveBase;
            this.flywheel = flywheel;
            this.aimer = aimer;

            this.xVelocity = xVelocity;
            this.yVelocity = yVelocity;
            this.target = target;

            pid = new PIDController(Constants.getMaxAngularSpeed().in(RadiansPerSecond) / 90, 0, 0);
            pid.setTolerance(1);
            pid.enableContinuousInput(-180, 180);
        }

        @Override
        public void initialize() {
            pid.setSetpoint(0);

            if (flywheel != null) {
                flywheel.start();
            }
        }

        @Override
        public void execute() {
            double linearMagnitude = SwerveBase.calculateLinearMagnitude(xVelocity, yVelocity);
            // Calcaulate new linear velocity.
            Rotation2d linearDirection = SwerveBase.calculateLinearDirection(xVelocity, yVelocity);
            Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

            // Calculate omega velocity.
            double degreesToTarget = driveBase.getRotationToTarget(target.get().toTranslation2d())
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
                driveBase.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(scaledXVelocity, scaledYVelocity,
                        omega, driveBase.getRotation()));
            } else {
                driveBase.runVelocity(new ChassisSpeeds(scaledXVelocity, scaledYVelocity, omega));
            }

            aimer.aimAtTarget(target.get(), driveBase.getEstimatedPosition().getTranslation());

            // TODO: Add Turning LEDs to Green, when close enough to shoot.

            // Log Calculated Values.
            Logger.recordOutput("DriveCommands/autoAimAndManuallyDriveCommand/degreesToTarget", degreesToTarget);
            Logger.recordOutput("DriveCommands/autoAimAndManuallyDriveCommand/vxMetersPerSecond", scaledXVelocity);
            Logger.recordOutput("DriveCommands/autoAimAndManuallyDriveCommand/vyMetersPerSecond", scaledYVelocity);
            Logger.recordOutput("DriveCommands/autoAimAndManuallyDriveCommand/omegaRadiansPerSecond", omega);
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            pid.close();
        }
    }

    public static Command autoAimAndManuallyDriveCommand(SwerveBase driveBase,
            Flywheel flywheel,
            Aimer aimer,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Translation3d> target) {

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
                double degreesToTarget = driveBase.getRotationToTarget(target.get().toTranslation2d())
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
                    driveBase.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(scaledXVelocity, scaledYVelocity,
                            omega, driveBase.getRotation()));
                } else {
                    driveBase.runVelocity(new ChassisSpeeds(scaledXVelocity, scaledYVelocity, omega));
                }

                aimer.aimAtTarget(target.get(), driveBase.getEstimatedPosition().getTranslation());

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
        aimingDrive.addRequirements(driveBase);
        if (Constants.hasFlywheelSubsystem())
            aimingDrive.addRequirements(flywheel);
        return aimingDrive;
    }

}