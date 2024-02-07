package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.LightsCommands;
import frc.robot.subsystems.base.DriveBase;
import frc.robot.subsystems.base.DriveBase.WheelModuleIndex;
import frc.robot.subsystems.gyro.GyroIoPigeon2;
import frc.robot.subsystems.gyro.GyroIoSimAndReplay;
import frc.robot.subsystems.led.LightsSubsystem;
import frc.robot.subsystems.shooter.Aimer;
import frc.robot.subsystems.shooter.DualCanSparkMaxSubsystem;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.swerve.SwerveModuleIoReplay;
import frc.robot.subsystems.swerve.SwerveModuleIoSim;
import frc.robot.subsystems.swerve.SwerveModuleIoTalonFx;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIoLimelight;
import frc.robot.subsystems.vision.VisionIoSimAndReplay;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController controller = new CommandXboxController(0);
  private final LightsSubsystem lightsSubsystem = new LightsSubsystem();
  private final DriveBase driveBase;
  private final Vision vision;
  private final LoggedDashboardChooser<Command> autoChooser;
  private final DualCanSparkMaxSubsystem intake;
  private final DualCanSparkMaxSubsystem feeder;
  private final Aimer aimer;
  private final Flywheel flywheel;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // ---------- Initialize the Drive Base ----------
    switch (Constants.CURRENT_OPERATING_MODE) {

      case REAL_WORLD:
        // Real robot, instantiate hardware IO implementations
        driveBase = new DriveBase(
            new GyroIoPigeon2(),
            new SwerveModuleIoTalonFx(WheelModuleIndex.FRONT_LEFT),
            new SwerveModuleIoTalonFx(WheelModuleIndex.FRONT_RIGHT),
            new SwerveModuleIoTalonFx(WheelModuleIndex.BACK_LEFT),
            new SwerveModuleIoTalonFx(WheelModuleIndex.BACK_RIGHT));
        vision = new Vision(driveBase.getPoseEstimator(), new VisionIoLimelight(Constants.SHOOTER_CAMERA_NAME));
        break;

      case SIMULATION:
        // Sim robot, instantiate physics sim IO implementations
        driveBase = new DriveBase(
            new GyroIoSimAndReplay(),
            new SwerveModuleIoSim(),
            new SwerveModuleIoSim(),
            new SwerveModuleIoSim(),
            new SwerveModuleIoSim());
        vision = new Vision(driveBase.getPoseEstimator(), new VisionIoSimAndReplay());
        break;

      case LOG_REPLAY:
        // Replayed robot, disable IO implementations
        driveBase = new DriveBase(
            new GyroIoSimAndReplay(),
            new SwerveModuleIoReplay(),
            new SwerveModuleIoReplay(),
            new SwerveModuleIoReplay(),
            new SwerveModuleIoReplay());
        vision = new Vision(driveBase.getPoseEstimator(), new VisionIoSimAndReplay());
        break;
      default:
        throw new RuntimeException("Unknown Run Mode: " + Constants.CURRENT_OPERATING_MODE);
    }
    if (Constants.HAVE_INTAKE) {
      intake = new DualCanSparkMaxSubsystem("Intake", Constants.INTAKE_L_ID, Constants.INTAKE_R_ID,
          Constants.INTAKE_FORWARD_VOLTAGE, Constants.INTAKE_REVERSE_VOLTAGE);
    } else {
      intake = null;
    }
    if (Constants.HAVE_FEEDER) {
      feeder = new DualCanSparkMaxSubsystem("Feeder", Constants.FEEDER_L_ID, Constants.FEEDER_R_ID,
          Constants.FEEDER_FORWARD_VOLTAGE, Constants.FEEDER_REVERSE_VOLTAGE);
    } else {
      feeder = null;
    }
    if (Constants.HAVE_AIMER) {
      aimer = new Aimer();
    } else {
      aimer = null;
    }
    if (Constants.HAVE_FLYWHEEL) {
      flywheel = new Flywheel();
    } else {
      flywheel = null;
    }

    // ========================= Autonomous =========================
    // ---------- Create Named Commands for use by Pathe Planner ----------
    NamedCommands.registerCommand("Spin 180", DriveCommands.spinCommand(driveBase, Rotation2d.fromDegrees(180), 1));
    NamedCommands.registerCommand("StartIntake", new PrintCommand("StartIntake working"));
    NamedCommands.registerCommand("Turn to Speaker", new ConditionalCommand(
        // Turn to Blue Speaker.
        DriveCommands.turnToTargetCommand(driveBase,
            new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), 4.5),
        // Turn to Red Speaker.
        DriveCommands.turnToTargetCommand(driveBase,
            new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42)), 4.5),
        () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Blue));

    // ---------- Set-up Autonomous Choices ----------
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // ========================= Tele-Op =========================
    // ---------- Configure Joystick for Tele-Op ----------
    driveBase.setDefaultCommand(DriveCommands.joystickDrive(driveBase,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX()));

    // ---------- Configure D-PAD for Tele-Op ----------
    controller.povUp().whileTrue(Commands.run(() -> driveBase.runVelocity(new ChassisSpeeds(1, 0, 0)),
        driveBase));
    controller.povDown().whileTrue(Commands.run(() -> driveBase.runVelocity(new ChassisSpeeds(-1, 0, 0)),
        driveBase));
    controller.povRight().whileTrue(Commands.run(() -> driveBase.runVelocity(new ChassisSpeeds(0, -1, 0)),
        driveBase));
    controller.povLeft().whileTrue(Commands.run(() -> driveBase.runVelocity(new ChassisSpeeds(0, 1, 0)),
        driveBase));

    // ---------- Configure Light Buttons ----------
    controller.a().onTrue(LightsCommands.setStaticPattern(lightsSubsystem,
        new Color[] { Color.kDarkGreen, Color.kDarkGreen, Color.kBlack, Color.kBlack }));
    controller.b().onTrue(LightsCommands.setColor(lightsSubsystem, Color.kRed));
    controller.x().onTrue(LightsCommands.setDynamicPattern(lightsSubsystem,
        new Color[] { Color.kBlue, Color.kBlue, Color.kBlue, Color.kDarkBlue, Color.kDarkBlue, Color.kDarkBlue },
        true));
    controller.y().onTrue(LightsCommands.setDynamicPattern(lightsSubsystem,
        new Color[] {
            Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kYellow, Color.kBlack, Color.kBlack,
            Color.kBlack, Color.kBlack, Color.kBlack,
            Color.kOrange, Color.kOrange, Color.kOrange, Color.kOrange, Color.kOrange, Color.kBlack, Color.kBlack,
            Color.kBlack, Color.kBlack, Color.kBlack },
        false));
    controller.a().and(controller.b()).onTrue(LightsCommands.setColor(lightsSubsystem, Color.kBlack));
    controller.rightBumper().onTrue(LightsCommands.changeBrightness(lightsSubsystem, false));
    controller.leftBumper().onTrue(LightsCommands.changeBrightness(lightsSubsystem, true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
