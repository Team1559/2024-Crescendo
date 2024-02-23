package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.constants.AbstractConstants.CONSTANTS;

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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.base.DriveBase;
import frc.robot.subsystems.shooter.Aimer;
import frc.robot.subsystems.shooter.Flywheel;

public class DriveCommands {

    /** Makes this class non-instantiable. */
    private DriveCommands() {
    }

    // TODO: Deduplicate code between this and the manualDriveDefaultCommand method.
    public static Command autoAimAndManuallyDriveCommand(DriveBase driveBase,
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
                pid = new PIDController(CONSTANTS.getMaxAngularSpeed().in(RadiansPerSecond) / 90 /* degrees */, 0, 0);
                pid.setSetpoint(0); // Degrees from target.
                pid.setTolerance(1/* degree(s) */);
                pid.enableContinuousInput(-180, 180); // Degrees.
                if (CONSTANTS.hasFlywheelSubsystem())
                    flywheel.start();
            }

            @Override
            public void execute() {

                // Apply deadband.
                double linearMagnitude = MathUtil.applyDeadband(
                        Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                        CONSTANTS.getJoystickDeadband());

                // Square values, to accelerate mor gradually.
                linearMagnitude = linearMagnitude * linearMagnitude;

                // Calcaulate new linear velocity.
                Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
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
                        -CONSTANTS.getMaxAngularSpeed().in(RadiansPerSecond),
                        CONSTANTS.getMaxAngularSpeed().in(RadiansPerSecond));

                // Scale Velocities to between 0 and Max.
                double scaledXVelocity = linearVelocity.getX() * CONSTANTS.getMaxLinearSpeed().in(MetersPerSecond),
                        scaledYVelocity = linearVelocity.getY() * CONSTANTS.getMaxLinearSpeed().in(MetersPerSecond);

                // Run Velocities.
                if (CONSTANTS.isDrivingModeFieldRelative()) {
                    driveBase.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(scaledXVelocity, scaledYVelocity,
                            omega, driveBase.getRotation()));
                } else {
                    driveBase.runVelocity(new ChassisSpeeds(scaledXVelocity, scaledYVelocity, omega));
                }

                aimer.aimAtTarget(target.get(), driveBase.getPose().getTranslation());

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
        if (CONSTANTS.hasFlywheelSubsystem())
            aimingDrive.addRequirements(flywheel);
        return aimingDrive;
    }

    public static Command manualDriveDefaultCommand(DriveBase driveBase,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {

        return Commands.run(
                () -> {

                    // Apply dead-band.
                    double linearMagnitude = MathUtil.applyDeadband(
                            Math.hypot(
                                    CONSTANTS.getAlliance() == Alliance.Blue ? -xSupplier.getAsDouble()
                                            : xSupplier.getAsDouble(),
                                    CONSTANTS.getAlliance() == Alliance.Blue ? -ySupplier.getAsDouble()
                                            : ySupplier.getAsDouble()), // Inverts the controllers if the alliance is
                                                                        // red
                            CONSTANTS.getJoystickDeadband());
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), CONSTANTS.getJoystickDeadband());

                    // Square values, to accelerate more gradually.
                    linearMagnitude = linearMagnitude * linearMagnitude;
                    omega = Math.copySign(omega * omega, omega);

                    // Calcaulate new linear velocity.
                    Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                    Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

                    // Scale Velocities to between 0 and Max.
                    Measure<Velocity<Distance>> scaledXVelocity = CONSTANTS.getMaxLinearSpeed()
                            .times(linearVelocity.getX());
                    Measure<Velocity<Distance>> scaledYVelocity = CONSTANTS.getMaxLinearSpeed()
                            .times(linearVelocity.getY());
                    Measure<Velocity<Angle>> scaledOmegaVelocity = CONSTANTS.getMaxAngularSpeed().times(omega);

                    // Run Velocities.
                    if (CONSTANTS.isDrivingModeFieldRelative()) {
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

    public static Command turnToTargetCommand(DriveBase driveBase, Supplier<Translation3d> target, double speed) {

        Command spinCommand = new Command() {

            private Command spinCommand;

            @Override
            public void initialize() {
                // Rotating plus 180 degrees to postion the back of the robot to the target.
                Rotation2d rotation = driveBase.getRotationToTarget(target.get().toTranslation2d())
                        .plus(Rotation2d.fromDegrees(180));
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