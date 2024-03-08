package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.BetterLogger;

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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.gyro.GyroIo;
import frc.robot.io.gyro.GyroIoInputsAutoLogged;
import frc.robot.io.swerve_module.SwerveModuleIo;
import frc.robot.subsystems.drive.SwerveModule.WheelModuleIndex;
import frc.robot.util.LocalAdStarAk;

public class SwerveBase extends SubsystemBase {

    // ========================= Class Level ===================================

    @AutoLog
    static class DriveBaseInputs {
        public Pose2d estimatedPosition;
        public Measure<Velocity<Distance>> estimatedSpeed;
    }

    public static SwerveBase createSimOrReplaySwerveBase(GyroIo gyroIo, SwerveModuleIo swerveModuleIo) {
        switch (Constants.getCurrentOperatingMode()) {
            case SIMULATION:
            case LOG_REPLAY:
                break;
            default:
                throw new RuntimeException("Invalid Operating Mode: " + Constants.getCurrentOperatingMode() + "!");
        }

        return new SwerveBase(gyroIo, swerveModuleIo, swerveModuleIo.clone(), swerveModuleIo.clone(),
                swerveModuleIo.clone());
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

    public SwerveBase(GyroIo gyroIo, SwerveModuleIo flModuleI, SwerveModuleIo frModuleIo, SwerveModuleIo blModuleIo,
            SwerveModuleIo brModuleIo) {

        // -------------------- Instantiate Hardware --------------------
        this.gyroIO = gyroIo;
        modules[WheelModuleIndex.FRONT_LEFT.value] = new SwerveModule(flModuleI, WheelModuleIndex.FRONT_LEFT);
        modules[WheelModuleIndex.FRONT_RIGHT.value] = new SwerveModule(frModuleIo, WheelModuleIndex.FRONT_RIGHT);
        modules[WheelModuleIndex.BACK_LEFT.value] = new SwerveModule(blModuleIo, WheelModuleIndex.BACK_LEFT);
        modules[WheelModuleIndex.BACK_RIGHT.value] = new SwerveModule(brModuleIo, WheelModuleIndex.BACK_RIGHT);

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
        PathPlannerLogging.setLogActivePathCallback(activePath -> BetterLogger.recordOutput("PathPlanner/ActivePath",
                activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging
                .setLogTargetPoseCallback(targetPose -> BetterLogger.recordOutput("PathPlanner/TargetPos", targetPose));

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
        BetterLogger.processInputs("Drive/Gyro", gyroInputs);

        // ---------- Update Position Estimator ----------
        updateModulePositions();
        poseEstimator.update(gyroInputs.yawPosition, modulePositions);

        // ---------- Log Drive Base Inputs ----------
        inputs.estimatedPosition = getEstimatedPosition();
        inputs.estimatedSpeed = Meters
                .of(lastPosition.getTranslation().getDistance(getEstimatedPosition().getTranslation()))
                .per(Seconds.of(1 / 50));
        lastPosition = getEstimatedPosition();
        BetterLogger.processInputs(LOG_PATH, inputs);
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
        BetterLogger.recordOutput(LOG_PATH + "/Setpoints", setpointStates);
        BetterLogger.recordOutput(LOG_PATH + "/SetpointsOptimized", optimizedSetpointStates);
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

    // ========================= Function Commands =============================

    public Command resetFieldOrientationCommand() {
        return new InstantCommand(this::resetFieldOrientation);
    }

    public Command stopCommand() {
        return new InstantCommand(this::stop);
    }
}