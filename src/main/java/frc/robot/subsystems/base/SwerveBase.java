package frc.robot.subsystems.base;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.AbstractConstants.CONSTANTS;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

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
import frc.robot.io.gyro.GyroIo;
import frc.robot.io.gyro.GyroIoInputsAutoLogged;
import frc.robot.io.swerve_module.SwerveModuleIo;
import frc.robot.subsystems.base.SwerveModule.WheelModuleIndex;
import frc.robot.util.LocalAdStarAk;

public class SwerveBase extends SubsystemBase {

    @AutoLog
    static class DriveBaseInputs {
        public Pose2d estimatedPosition;
        public Measure<Velocity<Distance>> estimatedSpeed;
    }

    private final String LOG_PATH = "Drive/Base";

    private final GyroIo gyroIO;
    private final GyroIoInputsAutoLogged gyroInputs = new GyroIoInputsAutoLogged();
    private final SwerveModule[] modules = new SwerveModule[4];

    private final SwerveDriveKinematics kinematics;
    public final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    private Pose2d lastPosition;

    private DriveBaseInputsAutoLogged inputs = new DriveBaseInputsAutoLogged();

    public SwerveBase(GyroIo gyroIo,
            SwerveModuleIo flModuleI,
            SwerveModuleIo frModuleIo,
            SwerveModuleIo blModuleIo,
            SwerveModuleIo brModuleIo) {

        // -------------------- Instantiate Hardware --------------------
        this.gyroIO = gyroIo;
        modules[WheelModuleIndex.FRONT_LEFT.value] = new SwerveModule(flModuleI, WheelModuleIndex.FRONT_LEFT);
        modules[WheelModuleIndex.FRONT_RIGHT.value] = new SwerveModule(frModuleIo, WheelModuleIndex.FRONT_RIGHT);
        modules[WheelModuleIndex.BACK_LEFT.value] = new SwerveModule(blModuleIo, WheelModuleIndex.BACK_LEFT);
        modules[WheelModuleIndex.BACK_RIGHT.value] = new SwerveModule(brModuleIo, WheelModuleIndex.BACK_RIGHT);

        // -------------------- Create Position Estimator --------------------
        kinematics = new SwerveDriveKinematics(new Translation2d[] {
                new Translation2d(CONSTANTS.getWheelDistanceFrontToBack().in(Meters) / 2.0,
                        CONSTANTS.getWheelDistanceLeftToRight().in(Meters) / 2.0),
                new Translation2d(CONSTANTS.getWheelDistanceFrontToBack().in(Meters) / 2.0,
                        -CONSTANTS.getWheelDistanceLeftToRight().in(Meters) / 2.0),
                new Translation2d(-CONSTANTS.getWheelDistanceFrontToBack().in(Meters) / 2.0,
                        CONSTANTS.getWheelDistanceLeftToRight().in(Meters) / 2.0),
                new Translation2d(-CONSTANTS.getWheelDistanceFrontToBack().in(Meters) / 2.0,
                        -CONSTANTS.getWheelDistanceLeftToRight().in(Meters) / 2.0)
        });

        updateModulePositions();

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics, gyroInputs.yawPosition, modulePositions,
                new Pose2d(0, 0, gyroInputs.yawPosition),
                VecBuilder.fill(0.01, 0.01, 0.01), // TODO: Why not use default number?
                VecBuilder.fill(1, 1, 1)); // placeholder, will be filled in by vision

        lastPosition = getEstimatedPosition();

        // -------------------- Configure PathPlanner --------------------
        AutoBuilder.configureHolonomic(
                this::getEstimatedPosition,
                this::setPose,
                () -> kinematics.toChassisSpeeds(getModuleStates()),
                this::runVelocity,
                new HolonomicPathFollowerConfig(CONSTANTS.getMaxLinearSpeed().in(MetersPerSecond),
                        CONSTANTS.getDriveBaseWheelRadius().in(Meters), new ReplanningConfig()),
                CONSTANTS::shouldFlipPath,
                this);
        Pathfinding.setPathfinder(new LocalAdStarAk());
        PathPlannerLogging.setLogActivePathCallback(activePath -> Logger.recordOutput("PathPlanner/ActivePath",
                activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging
                .setLogTargetPoseCallback(targetPose -> Logger.recordOutput("PathPlanner/TargetPos", targetPose));

        // TODO: Figure out why the robot is not starting at 0,0.
        setPose(new Pose2d(new Translation2d(),
                CONSTANTS.getAlliance() == Alliance.Blue ? Rotation2d.fromDegrees(180) : new Rotation2d()));
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
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, CONSTANTS.getMaxLinearSpeed());

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

    // ========================= Commands =====================================

    public Command resetFieldOrientationCommand() {
        return new InstantCommand(this::resetFieldOrientation);
    }

    public Command stopCommand() {
        return new InstantCommand(this::stop);
    }
}