package frc.robot.util;

import java.time.Duration;
import java.time.LocalTime;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.base.DriveBase;

public class CalibrationUtils {

    public static Command calibrateSpin(DriveBase driveBase) {

        SequentialCommandGroup commands = new SequentialCommandGroup();

        for (int breakingAngleInDgrees = 45; breakingAngleInDgrees > 0; breakingAngleInDgrees -= 15) {
            for (int breakingExponent = 1; breakingExponent <= 3; breakingExponent++) {
                commands.addCommands(
                        calibrateSpinStep(driveBase, Rotation2d.fromDegrees(breakingAngleInDgrees), breakingExponent));
            }
        }

        return commands;
    }

    public static Command calibrateSpinStep(DriveBase driveBase, Rotation2d breakingAngleThreshold,
            int breakingExponent) {

        Command command = new Command() {

            private Rotation2d targetRotation;
            private LocalTime startTime;

            @Override
            public void initialize() {
                targetRotation = driveBase.getPose().getRotation().minus(Rotation2d.fromDegrees(180));
                startTime = LocalTime.now();
            }

            @Override
            public void execute() {
                double angularSpeedPercentage = getAngularSpeedPercentage(driveBase.getRotation(), targetRotation,
                        breakingAngleThreshold, breakingExponent);
                double omega = angularSpeedPercentage * Constants.MAX_ANGULAR_SPEED_IN_RADS_PER_SECONDS;

                driveBase.runVelocity(new ChassisSpeeds(0, 0, omega));
            }

            @Override
            public boolean isFinished() {
                Rotation2d current = driveBase.getRotation();
                double delta = targetRotation.minus(current).getDegrees();
                return Math.abs(delta) < 1 /* degrees */;
            }

            @Override
            public void end(boolean interrupted) {
                long durationInSeconds = Duration.between(startTime, LocalTime.now()).toSeconds();

                driveBase.stop();

                Logger.recordOutput("CalibrationUtils/calibrateSpin/breakingAngleThreshold", breakingAngleThreshold);
                Logger.recordOutput("CalibrationUtils/calibrateSpin/breakingExponent", breakingExponent);
                Logger.recordOutput("CalibrationUtils/calibrateSpin/duration", durationInSeconds);
            }
        };

        command.addRequirements(driveBase);

        return command;
    }

    private static double getAngularSpeedPercentage(Translation2d currentPosition, Translation2d targetPosition,
            Rotation2d breakingAngleThreshold, int breakingExponent) {

        Translation2d deltaPosition = targetPosition.minus(currentPosition);
        Rotation2d deltaRotation = deltaPosition.getAngle();

        Logger.recordOutput("CalibrationUtils/getAngularSpeedPercentage-Translation2d/currentPosition",
                currentPosition);
        Logger.recordOutput("CalibrationUtils/getAngularSpeedPercentage-Translation2d/targetPosition", targetPosition);
        Logger.recordOutput("CalibrationUtils/getAngularSpeedPercentage-Translation2d/deltaPosition", deltaPosition);
        Logger.recordOutput("CalibrationUtils/getAngularSpeedPercentage-Translation2d/deltaRotation", deltaRotation);

        return getAngularSpeedPercentage(Rotation2d.fromDegrees(0), deltaRotation, breakingAngleThreshold,
                breakingExponent);
    }

    private static double getAngularSpeedPercentage(Rotation2d currentRotation, Rotation2d targetRotation,
            Rotation2d breakingAngleThreshold, int breakingExponent) {

        if (breakingExponent < 1) {
            throw new IllegalArgumentException(
                    "breakingExponent must be greater than or equal to 1 but curretly is: " + breakingExponent + "!");
        }

        double breakingPathPercentLeft = -1;
        Rotation2d deltaRotation = targetRotation.minus(currentRotation);

        double angularSpeedPercentage = Math.copySign(1, deltaRotation.getDegrees());
        if (Math.abs(deltaRotation.getDegrees()) < breakingAngleThreshold.getDegrees()) {
            breakingPathPercentLeft = deltaRotation.getDegrees() / breakingAngleThreshold.getDegrees();
            angularSpeedPercentage = Math.pow(breakingPathPercentLeft, breakingExponent);
        }

        Logger.recordOutput("CalibrationUtils/getAngularSpeedPercentage-Rotation2d/currentRotation", currentRotation);
        Logger.recordOutput("CalibrationUtils/getAngularSpeedPercentage-Rotation2d/targetRotation", targetRotation);
        Logger.recordOutput("CalibrationUtils/getAngularSpeedPercentage-Rotation2d/breakingAngleThreshold",
                breakingAngleThreshold);
        Logger.recordOutput("CalibrationUtils/getAngularSpeedPercentage-Rotation2d/breakingExponent", breakingExponent);
        Logger.recordOutput("CalibrationUtils/getAngularSpeedPercentage-Rotation2d/angularSpeedPercentage",
                angularSpeedPercentage);
        Logger.recordOutput("CalibrationUtils/getAngularSpeedPercentage-Rotation2d/breakingPathPercentLeft",
                breakingPathPercentLeft);
        Logger.recordOutput("CalibrationUtils/getAngularSpeedPercentage-Rotation2d/angularSpeedPercentage",
                angularSpeedPercentage);

        return angularSpeedPercentage;
    }
}
