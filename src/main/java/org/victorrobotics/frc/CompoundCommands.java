package org.victorrobotics.frc;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.victorrobotics.frc.subsystems.abstract_interface.MotorSubsystem;
import org.victorrobotics.frc.subsystems.drive.SwerveBase;
import org.victorrobotics.frc.subsystems.led.Leds;
import org.victorrobotics.frc.subsystems.shooter.Aimer;
import org.victorrobotics.frc.subsystems.shooter.Feeder;
import org.victorrobotics.frc.subsystems.shooter.Flywheel;
import org.victorrobotics.frc.subsystems.shooter.Intake;
import org.victorrobotics.frc.subsystems.shooter.NoteSensor;
import org.victorrobotics.frc.util.CommandUtils;

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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/**
 * Only used for commands that use multiple subsystems.
 * <p>
 * Single subsystem commands should be in their subsystem class.
 * </p>
 */
public class CompoundCommands {

    /** Makes Class non-instantiable */
    private CompoundCommands() {
    }

    // ========================= Trigger Commands ==============================

    public static Command overheatedMotorShutdownCommand(MotorSubsystem motorSubsystem, Leds leds) {
        Command command = motorSubsystem.stopCommand()
                .alongWith(leds.setDynamicPatternCommand(Constants.getMotorOverheatEmergencyPattern(), false));

        return CommandUtils.addName(command);
    }

    // ========================= Other Commands =========================

    public static Command autoAimAndManuallyDriveCommand(SwerveBase swerveBase, Aimer aimer, Flywheel flywheel,
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
                    flywheel.forward();
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
                Logger.recordOutput("DriveCommands/autoAimAndManuallyDriveCommand/degreesToTarget",
                        degreesToTarget);
                Logger.recordOutput("DriveCommands/autoAimAndManuallyDriveCommand/vxMetersPerSecond",
                        scaledXVelocity);
                Logger.recordOutput("DriveCommands/autoAimAndManuallyDriveCommand/vyMetersPerSecond",
                        scaledYVelocity);
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
        return CommandUtils.addName(aimingDrive);
    }

    public static Command autoDelayedManualShotCommand(Aimer aimer, Feeder feeder, Intake intake,
            NoteSensor noteSensor) {

        Command command = new ParallelDeadlineGroup(
                new WaitCommand(12),
                aimer.setAngleCommand(Rotation2d.fromDegrees(36.7)).andThen(new WaitUntilCommand(aimer::isAtTarget))
                        .andThen(CompoundCommands.shootAutonomousCommand(feeder, intake, noteSensor)));

        return CommandUtils.addName(command);
    }

    public static Command autoShootCommand(SwerveBase swerveBase, Aimer aimer, Feeder feeder, Intake intake,
            NoteSensor noteSensor) {
        Command command = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        swerveBase.turnToTargetCommand(Constants::getSpeakerLocation, 4.5),
                        aimer.aimAtTargetCommand(Constants::getSpeakerLocation, swerveBase::getTranslation)
                                .andThen(aimer.waitUntilAtTargetCommand())),
                CompoundCommands.shootAutonomousCommand(feeder, intake, noteSensor));

        return CommandUtils.addName(command);
    }

    public static Command autoJustShootCommand(Aimer aimer, Feeder feeder, Intake intake, NoteSensor noteSensor) {

        Command command = aimer.setAngleCommand(Rotation2d.fromDegrees(36.7))
                .andThen(new WaitUntilCommand(aimer::isAtTarget))
                .andThen(CompoundCommands.shootAutonomousCommand(feeder, intake, noteSensor));

        return CommandUtils.addName(command);
    }

    public static Command intakeStartStopCommand(Feeder feeder, Intake intake) {
        Command command = new StartEndCommand(
                () -> {
                    intake.forward();
                    feeder.forward();
                },
                () -> {
                    intake.stop();
                    feeder.stop();
                },
                intake, feeder);

        return CommandUtils.addName(command);
    }

    public static Command reverseShooterAndIntakeCommand(Feeder feeder, Flywheel flywheel, Intake intake) {
        Command command = new ParallelCommandGroup(new StartEndCommand(flywheel::reverse, flywheel::stop, flywheel),
                new StartEndCommand(feeder::reverse, feeder::stop, feeder),
                new StartEndCommand(intake::reverse, intake::stop, intake));

        return CommandUtils.addName(command);
    }

    public static Command reverseShooterCommand(Feeder feeder, Flywheel flywheel, Leds leds) {
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

        return CommandUtils.addName(reverseShooterCommand);
    }

    public static Command runIntakeCommand(Feeder feeder, Flywheel flywheel, Intake intake) {

        Command command = new ParallelCommandGroup(CompoundCommands.intakeStartStopCommand(feeder, intake),
                flywheel.stopCommand());

        return CommandUtils.addName(command);
    }

    public static Command shootAutonomousCommand(Feeder feeder, Intake intake, NoteSensor noteSensor) {

        ParallelRaceGroup group = new ParallelRaceGroup(
                feeder.forwardThenStopCommand(),
                intake.forwardThenStopCommand(),
                // TODO: May want to wait a little after the note is no longer sensed.
                noteSensor.waitForNoObjectOnSwitchCommand(),
                new WaitCommand(5));

        return CommandUtils.addName(group);
    }

    public static Command shootTeleopCommand(Feeder feeder, Flywheel flywheel, Intake intake, NoteSensor noteSensor) {

        // TODO: Have this run until the Co-Pilot stops pushing the button.
        ParallelRaceGroup group = new ParallelRaceGroup(
                new StartEndCommand(intake::forward, intake::stop, intake),
                feeder.forwardMaxVelocityThenStopCommand(),
                noteSensor.waitForNoObjectOnSwitchCommand(),
                new WaitCommand(5));

        // TODO: Spin up flywheel if not already spinning.

        return CommandUtils.addName(group);
    }
}
