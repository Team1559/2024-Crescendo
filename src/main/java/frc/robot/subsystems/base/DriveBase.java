package frc.robot.subsystems.base;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.gyro.GyroIo;
import frc.robot.subsystems.gyro.GyroIoInputsAutoLogged;
import frc.robot.subsystems.swerve.IndexedSwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleIo;
import frc.robot.util.LocalADStarAK;

public class DriveBase extends SubsystemBase {
  private static final double DRIVE_BASE_RADIUS = Math.hypot(Constants.TRACK_WIDTH_X / 2.0, Constants.TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = Constants.MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  private final GyroIo gyroIO;
  private final GyroIoInputsAutoLogged gyroInputs = new GyroIoInputsAutoLogged();
  private final IndexedSwerveModule[] modules = new IndexedSwerveModule[4];

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Pose2d pose = new Pose2d();
  private Rotation2d lastGyroRotation = new Rotation2d();

  public enum WheelModuleIndex {
    /** 0 */
    FRONT_LEFT(0),
    /** 1 */
    FRONT_RIGHT(1),
    /** 2 */
    BACK_LEFT(2),
    /** 3 */
    BACK_RIGHT(3);

    public final int value;

    private WheelModuleIndex(int value) {
      this.value = value;
    }
  }

  public DriveBase( GyroIo gyroIO,
    SwerveModuleIo flModuleIO,
    SwerveModuleIo frModuleIO,
    SwerveModuleIo blModuleIO,
    SwerveModuleIo brModuleIO) {

    this.gyroIO = gyroIO;
    modules[WheelModuleIndex.FRONT_LEFT.value] = new IndexedSwerveModule(flModuleIO, WheelModuleIndex.FRONT_LEFT.value);
    modules[WheelModuleIndex.FRONT_RIGHT.value] = new IndexedSwerveModule(frModuleIO, WheelModuleIndex.FRONT_RIGHT.value);
    modules[WheelModuleIndex.BACK_LEFT.value] = new IndexedSwerveModule(blModuleIO, WheelModuleIndex.BACK_LEFT.value);
    modules[WheelModuleIndex.BACK_RIGHT.value] = new IndexedSwerveModule(brModuleIO, WheelModuleIndex.BACK_RIGHT.value);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(Constants.MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
         // Flips path if aliance is on red side.
        () -> Constants.FLIP_PATH_IF_ALLIANCE_IS_NOT_DEFAULT && DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() != Constants.DEFAULT_ALLIANCE,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        activePath -> Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]))
        );
    PathPlannerLogging.setLogTargetPoseCallback(
        targetPose -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose)
        );
  }

  @Override
  public void periodic() {

    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints []");
      Logger.recordOutput("SwerveStates/SetpointsOptimized []");
    }

    // Update odometry
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] = modules[i].getPositionDelta();
    }
    
    // The twist represents the motion of the robot since the last
    // loop cycle in x, y, and theta based only on the modules,
    // without the gyro. The gyro is always disconnected in simulation.
    var twist = kinematics.toTwist2d(wheelDeltas);
    if (gyroInputs.connected) {
      // If the gyro is connected, replace the theta component of the twist
      // with the change in angle since the last loop cycle.
      twist = new Twist2d(twist.dx, twist.dy, gyroInputs.yawPosition.minus(lastGyroRotation).getRadians());
      lastGyroRotation = gyroInputs.yawPosition;
    }
    // Apply the twist (change since last loop cycle) to the current pose
    pose = pose.exp(twist);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement.
   * The modules will
   * return to their normal orientations the next time a nonzero velocity is
   * requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all of the
   * modules.
   */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return pose;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return pose.getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    this.pose = pose;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return Constants.MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
        new Translation2d(Constants.TRACK_WIDTH_X / 2.0, Constants.TRACK_WIDTH_Y / 2.0),
        new Translation2d(Constants.TRACK_WIDTH_X / 2.0, -Constants.TRACK_WIDTH_Y / 2.0),
        new Translation2d(-Constants.TRACK_WIDTH_X / 2.0, Constants.TRACK_WIDTH_Y / 2.0),
        new Translation2d(-Constants.TRACK_WIDTH_X / 2.0, -Constants.TRACK_WIDTH_Y / 2.0)
    };
  }
}