package org.victorrobotics.frc.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.victorrobotics.frc.Constants;
import org.victorrobotics.frc.io.gyro.GyroIo;
import org.victorrobotics.frc.io.gyro.GyroIoInputsAutoLogged;
import org.victorrobotics.frc.subsystems.drive.SwerveModule.WheelModuleIndex;
import org.victorrobotics.frc.subsystems.led.Leds;
import org.victorrobotics.frc.util.CommandUtils;
import org.victorrobotics.frc.util.LocalAdStarAk;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveBase extends SubsystemBase {

    // ========================= Class Level ===================================

    @AutoLog
    static class DriveBaseInputs {
        public Pose2d estimatedPosition;
        public Measure<Velocity<Distance>> estimatedSpeed;
    }

    /**
     * Calculates and squares the linear magnitude for the swerve drive
     * 
     * @param xSupplier Joystick x
     * @param ySupplier Joystick Y
     * @return linear magnitude
     */
    public static double calculateLinearMagnitude(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        double magnitude = Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble());
        double clampedMagnitude = MathUtil.applyDeadband(magnitude, Constants.getJoystickDeadband());
        // Square values, for more precision at slow speeds
        return Math.pow(clampedMagnitude, 2);
    }

    public static Rotation2d calculateLinearDirection(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        Rotation2d stickDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
        return Constants.getAlliance() == Alliance.Red
                ? stickDirection
                : stickDirection.plus(Rotation2d.fromDegrees(180));
    }

    // ========================= Object Level ===================================

    private final String LOG_PATH = "Drive/Base";

    private final GyroIo gyroIO;
    private final GyroIoInputsAutoLogged gyroInputs = new GyroIoInputsAutoLogged();
    private final SwerveModule[] modules = new SwerveModule[4];

    private final SwerveDriveKinematics kinematics;
    public final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    private Pose2d lastPosition;

    private final DriveBaseInputsAutoLogged inputs = new DriveBaseInputsAutoLogged();

    public SwerveBase(GyroIo gyroIo, SwerveModule flModule, SwerveModule frModule, SwerveModule blModule,
            SwerveModule brModule) {

        // -------------------- Instantiate Hardware --------------------
        this.gyroIO = gyroIo;
        modules[WheelModuleIndex.FRONT_LEFT.value] = flModule;
        modules[WheelModuleIndex.FRONT_RIGHT.value] = frModule;
        modules[WheelModuleIndex.BACK_LEFT.value] = blModule;
        modules[WheelModuleIndex.BACK_RIGHT.value] = brModule;

        // -------------------- Create Position Estimator --------------------
        kinematics = new SwerveDriveKinematics(new Translation2d[] {
                new Translation2d(Constants.getWheelDistanceFrontToBack().in(Meters) / 2.0,
                        Constants.getWheelDistanceLeftToRight().in(Meters) / 2.0),
                new Translation2d(Constants.getWheelDistanceFrontToBack().in(Meters) / 2.0,
                        -Constants.getWheelDistanceLeftToRight().in(Meters) / 2.0),
                new Translation2d(-Constants.getWheelDistanceFrontToBack().in(Meters) / 2.0,
                        Constants.getWheelDistanceLeftToRight().in(Meters) / 2.0),
                new Translation2d(-Constants.getWheelDistanceFrontToBack().in(Meters) / 2.0,
                        -Constants.getWheelDistanceLeftToRight().in(Meters) / 2.0)
        });

        updateModulePositions();

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics, gyroInputs.yawPosition, modulePositions,
                new Pose2d(0, 0, gyroInputs.yawPosition),
                VecBuilder.fill(0.01, 0.01, 0.01), // TODO: Why not use default number?
                VecBuilder.fill(1, 1, 1)); // placeholder, will be filled in by vision.

        lastPosition = getEstimatedPosition();

        // -------------------- Configure PathPlanner --------------------
        AutoBuilder.configureHolonomic(
                this::getEstimatedPosition,
                this::setPose,
                () -> kinematics.toChassisSpeeds(getModuleStates()),
                this::runVelocity,
                new HolonomicPathFollowerConfig(Constants.getMaxLinearSpeed().in(MetersPerSecond),
                        Constants.getDriveBaseWheelRadius().in(Meters), new ReplanningConfig()),
                Constants::shouldFlipPath,
                this);
        Pathfinding.setPathfinder(new LocalAdStarAk());
        PathPlannerLogging.setLogActivePathCallback(activePath -> Logger.recordOutput("PathPlanner/ActivePath",
                activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging
                .setLogTargetPoseCallback(targetPose -> Logger.recordOutput("PathPlanner/TargetPos", targetPose));

        // TODO: Figure out why the robot is not starting at 0,0.
        setPose(new Pose2d(new Translation2d(),
                Constants.getAlliance() == Alliance.Blue ? Rotation2d.fromDegrees(180) : new Rotation2d()));
    }

    @Override
    public void periodic() {

        // Stop moving when disabled.
        if (DriverStation.isDisabled()) {
            stop();
        }

        // Run Module periodic methods.
        for (SwerveModule module : modules) {
            module.periodic();
        }

        // ---------- Log Gyro Inputs ----------
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        // ---------- Update Position Estimator ----------
        updateModulePositions();
        poseEstimator.update(gyroInputs.yawPosition, modulePositions);

        // ---------- Log Drive Base Inputs ----------
        inputs.estimatedPosition = getEstimatedPosition();
        inputs.estimatedSpeed = Meters
                .of(lastPosition.getTranslation().getDistance(getEstimatedPosition().getTranslation()))
                .per(Seconds.of(1 / 50));
        lastPosition = getEstimatedPosition();
        Logger.processInputs(LOG_PATH, inputs);
    }

    // ========================= Functions =====================================

    public Pose2d getEstimatedPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    public Measure<Velocity<Distance>> getEstimatedSpeed() {
        return inputs.estimatedSpeed;
    }

    public Rotation2d getRotation() {
        return getEstimatedPosition().getRotation();
    }

    /**
     * This gets the rotation from the current position to the target position, and
     * it's trying to point the front of the robot to the target.
     *
     * @param target
     * @return
     */
    public Rotation2d getRotationToTarget(Translation2d target) {
        Pose2d currentPose = getEstimatedPosition();
        Translation2d deltaTranslation = target.minus(currentPose.getTranslation());
        Rotation2d deltaAngle = deltaTranslation.getAngle();
        return deltaAngle.minus(currentPose.getRotation());
    }

    public Translation2d getTranslation() {
        return getEstimatedPosition().getTranslation();
    }

    public boolean isTemperatureTooHigh() {
        for (SwerveModule module : modules) {
            if (module.isTemperatureTooHigh()) {
                return true;
            }
        }
        return false;
    }

    public void resetFieldOrientation() {
        poseEstimator.addVisionMeasurement(new Pose2d(getTranslation(), new Rotation2d()), Timer.getFPGATimestamp(),
                VecBuilder.fill(0, 0, 0));
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {

        SwerveModuleState[] setpointStates, optimizedSetpointStates;
        if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {

            optimizedSetpointStates = setpointStates = new SwerveModuleState[4];
            int i = -1;
            for (SwerveModule module : modules) {
                module.stop();
                setpointStates[++i] = module.getSwerveModuleState();
            }
        } else {

            // Calculate module setpoints.
            ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
            setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.getMaxLinearSpeed());

            // Send setpoints to modules.
            optimizedSetpointStates = new SwerveModuleState[4];
            for (int i = 0; i < modules.length; i++) {
                optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
            }
        }

        // Log setpoint states.
        Logger.recordOutput(LOG_PATH + "/Setpoints", setpointStates);
        Logger.recordOutput(LOG_PATH + "/SetpointsOptimized", optimizedSetpointStates);
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(gyroInputs.yawPosition, modulePositions, pose);
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    // ========================= Helper Methods ================================

    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getSwerveModuleState();
        }
        return states;
    }

    private void updateModulePositions() {
        for (int i = 0; i < modules.length; i++) {
            modulePositions[i] = modules[i].getSwerveModulePosition();
        }
    }

    // ========================= Default Commands ==============================

    public Command manualDriveDefaultCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {

        Command command = Commands.run(
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
                        runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(scaledXVelocity, scaledYVelocity,
                                scaledOmegaVelocity, getRotation()));
                    } else {
                        runVelocity(new ChassisSpeeds(scaledXVelocity, scaledYVelocity, scaledOmegaVelocity));
                    }
                },
                this);

        return CommandUtils.addName(command);
    }

    // ========================= Trigger Commands ==============================

    public Command overheatedMotorShutdownCommand(Leds leds) {
        Command command = stopCommand()
                .alongWith(leds.setDynamicPatternCommand(Constants.getMotorOverheatEmergencyPattern(), false));

        return CommandUtils.addName(getName(), command);
    }

    // ========================= Function Commands =============================

    public Command resetFieldOrientationCommand() {
        return CommandUtils.addName(getName(), new InstantCommand(this::resetFieldOrientation));
    }

    public Command runVelocityCommand(ChassisSpeeds speeds) {
        return CommandUtils.addName(getName(), new RunCommand(() -> runVelocity(speeds)));
    }

    /**
     * This method will create a command to spin the robot the specified amount at a
     * given speed. The robot
     * will always take the shortest path.
     * 
     * @param rotationAmount The amount the robot rotates.
     * @param speed          The speed to spin at. (must be a positive number
     *                       greater
     *                       than 0).
     * @return The created command.
     */
    public Command spinCommand(Rotation2d rotationAmount, double speed) {

        if (speed <= 0) {
            throw new RuntimeException("Robot cannot spin because velocity is negative or zero:  " + speed);
        }

        Command spinCommand = new Command() {

            private Rotation2d targetRotation;

            @Override
            public void initialize() {
                Rotation2d startingRotation = getRotation();
                targetRotation = startingRotation.plus(rotationAmount);
            }

            @Override
            public void execute() {

                Rotation2d current = getRotation();
                double delta = targetRotation.minus(current).getDegrees();

                double rampOmega = Math.max(Math.min(Math.abs(delta) / 50 /* degrees */, 1.0), .01);
                double omega = Math.copySign(speed, delta) * rampOmega;

                runVelocity(new ChassisSpeeds(0, 0, omega));

                Logger.recordOutput("DriveCommands/spinCommand/delta", delta);
                Logger.recordOutput("DriveCommands/spinCommand/rampOmega", rampOmega);
                Logger.recordOutput("DriveCommands/spinCommand/omega", omega);
            }

            @Override
            public boolean isFinished() {
                Rotation2d current = getRotation();
                double delta = targetRotation.minus(current).getDegrees();
                return Math.abs(delta) < .5 /* degrees */;
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };

        spinCommand.addRequirements(this);

        return CommandUtils.addName(getName(), spinCommand);
    }

    public Command stopCommand() {
        return CommandUtils.addName(getName(), new InstantCommand(this::stop));
    }

    public Command turnToTargetCommand(Supplier<Translation3d> target, double speed) {

        Command spinCommand = new Command() {

            private Command spinCommand;

            @Override
            public void initialize() {
                // Rotating plus 180 degrees to position the back of the robot to the target.
                Rotation2d rotation = getRotationToTarget(target.get().toTranslation2d())
                        .plus(Rotation2d.fromDegrees(180));
                spinCommand = spinCommand(rotation, speed);
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

        spinCommand.addRequirements(this);

        return CommandUtils.addName(getName(), spinCommand);
    }
}