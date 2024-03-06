package frc.robot;

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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.io.motor.MotorIoNeo550Brushless;
import frc.robot.subsystems.drive.SwerveBase;
import frc.robot.subsystems.led.Leds;
import frc.robot.subsystems.shooter.Aimer;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Intake;
import frc.robot.subsystems.shooter.NoteSensor;

/**
 * Only used for commands that use multiple subsystems.
 * <p>
 * Single subsystem commands should be in their subsystem class.
 * </p>
 */
public class CompositeCommands {

    /** Makes Class non-instantiable */
    private CompositeCommands() {
    }

    // ========================= Default Commands =========================
    public static Command defaultIntakeCommand(Intake intake, NoteSensor sensor) {
        return Commands.run(() -> {
            if (sensor.isObjectDetected()) {
                intake.stop();
            } else {
                intake.start();
            }
        }, intake);
    }

    public static Command defaultFeederCommand(Feeder feeder, NoteSensor sensor) {
        return Commands.run(() -> {
            if (sensor.isObjectDetected()) {
                feeder.stop();
            } else {
                feeder.start();
            }
        }, feeder);
    }

    // ========================= Other Commands =========================

    public static Command autoAimAndManuallyDriveCommand(SwerveBase driveBase, Flywheel flywheel, Aimer aimer,
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

    public static Command intakeStartStopCommand(Intake intake, Feeder feeder) {
        return new StartEndCommand(
                () -> {
                    intake.start();
                    feeder.start();
                },
                () -> {
                    intake.stop();
                    feeder.stop();
                },
                intake, feeder);
    }

    public static Command reverseShooterAndIntakeCommand(Intake intake, Feeder feeder, Flywheel flywheel) {
        return new ParallelCommandGroup(new StartEndCommand(flywheel::reverse, flywheel::stop, flywheel),
                new StartEndCommand(feeder::reverse, feeder::stop, feeder),
                new StartEndCommand(intake::reverse, intake::stop, intake));
    }

    public static Command reverseShooterCommand(Flywheel flywheel, Feeder feeder, Leds leds) {
        Command reverseShooterCommand = new Command() {
            @Override
            public void execute() {
                flywheel.reverse();
                feeder.reverse();
                leds.setDynamicPattern(new Color[] { Color.kRed, Color.kRed, Color.kBlack, Color.kBlack }, true);
            }

            @Override
            public void end(boolean interrupted) {
                flywheel.stop();
                feeder.stop();
                leds.setAllianceColor();
            }
        };
        reverseShooterCommand.addRequirements(flywheel, feeder, leds);
        return reverseShooterCommand;
    }

    public static Command shootAutonomousCommand(Feeder feeder, Leds leds, NoteSensor noteSensor) {
        return new SequentialCommandGroup(
                feeder.startCommand(),
                leds.blinkCommand(Color.kOrange),
                noteSensor.waitForNoObjectCommand(),
                new WaitCommand(.25),
                feeder.stopCommand());
    }

    public static Command shootTeleopCommand(Feeder feeder, Flywheel flywheel, Intake intake, NoteSensor noteSensor,
            Leds leds) {

        ParallelRaceGroup group = new ParallelRaceGroup(new StartEndCommand(intake::start, intake::stop, intake),
                new StartEndCommand(() -> feeder.setVelocity(MotorIoNeo550Brushless.MAX_VELOCITY), feeder::stop,
                        feeder),
                leds.setColorCommand(Color.kPurple).repeatedly(),
                noteSensor.waitForNoObjectCommand(), new WaitCommand(5));

        // TODO: Spin up flywheelsm if not already spinning.
        return group;
    }

    public static Command stopIntakeFeederCommand(Intake intake, Feeder feeder, Leds leds) {
        return new InstantCommand(() -> {
            intake.stop();
            feeder.stop();
            leds.setDynamicPattern(new Color[] { Color.kRed, Color.kRed, Color.kBlack, Color.kBlack }, true);
        }, intake, feeder);
    }
}
