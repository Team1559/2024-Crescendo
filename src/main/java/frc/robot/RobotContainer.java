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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.LightsCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.base.DriveBase;
import frc.robot.subsystems.base.DriveBase.WheelModuleIndex;
import frc.robot.subsystems.gyro.GyroIoPigeon2;
import frc.robot.subsystems.gyro.GyroIoSimAndReplay;
import frc.robot.subsystems.led.Leds;
import frc.robot.subsystems.shooter.Aimer;
import frc.robot.subsystems.shooter.ColorSensor;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Intake;
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
  private final Leds lightsSubsystem = new Leds();
  private final DriveBase driveBase;
  private final Vision vision;
  private final LoggedDashboardChooser<Command> autoChooser;
  private final Intake intake;
  private final Feeder feeder;
  private final Aimer aimer;
  private final Flywheel flywheel;
  private final ColorSensor sensor;

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
        vision = new Vision(driveBase.getPoseEstimator(), new VisionIoLimelight(Constants.CAMERA_1_NAME));
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

    // ---------- Initialize Subsystems ----------
    if (Constants.HAVE_INTAKE) {
      intake = new Intake();
    } else {
      intake = null;
    }
    if (Constants.HAVE_FEEDER) {
      feeder = new Feeder();
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
    if (Constants.HAVE_COLOR_SENSOR) {
      sensor = new ColorSensor();
    } else {
      sensor = null;
    }

    // ========================= Autonomous =========================
    // ---------- Create Named Commands for use by Path Planner ----------
    NamedCommands.registerCommand("Spin 180", DriveCommands.spinCommand(driveBase, Rotation2d.fromDegrees(180), 1));
    NamedCommands.registerCommand("StartIntake", LightsCommands.blinkCommand(lightsSubsystem, Color.kPurple));

    Command aimCommand = new ConditionalCommand(
        // Turn to Blue Speaker.
        DriveCommands.turnToTargetCommand(driveBase,
            new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), 4.5),
        // Turn to Red Speaker.
        DriveCommands.turnToTargetCommand(driveBase,
            new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42)), 4.5),
        () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
    Command autoShootCommand;
    if (Constants.HAVE_SHOOTER) {
      autoShootCommand = ShooterCommands.shootCommand(flywheel, feeder, lightsSubsystem, sensor);
    } else {
      autoShootCommand = LightsCommands.blinkCommand(lightsSubsystem, Color.kOrange);
    }
    NamedCommands.registerCommand("ShootNote", new SequentialCommandGroup(aimCommand, autoShootCommand));

    // ---------- Set-up Autonomous Choices ----------
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // ========================= Tele-Op =========================
    // ---------- Configure Joystick for Tele-Op ----------
    driveBase.setDefaultCommand(DriveCommands.joystickDrive(driveBase,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getLeftTriggerAxis() + controller.getRightTriggerAxis()));

    // ---------- Configure D-PAD for Tele-Op ----------
    controller.povUp().whileTrue(Commands.run(() -> driveBase.runVelocity(new ChassisSpeeds(1, 0, 0)),
        driveBase));
    controller.povDown().whileTrue(Commands.run(() -> driveBase.runVelocity(new ChassisSpeeds(-1, 0, 0)),
        driveBase));
    controller.povRight().whileTrue(Commands.run(() -> driveBase.runVelocity(new ChassisSpeeds(0, -1, 0)),
        driveBase));
    controller.povLeft().whileTrue(Commands.run(() -> driveBase.runVelocity(new ChassisSpeeds(0, 1, 0)),
        driveBase));

    // ---------- Configure Buttons for SubSystem Actions ----------
    Command teleOpShootCommand;
    if (Constants.HAVE_SHOOTER) {
      teleOpShootCommand = ShooterCommands.shootCommand(flywheel, feeder, lightsSubsystem, sensor);
    } else {
      teleOpShootCommand = LightsCommands.blinkCommand(lightsSubsystem, Color.kOrange);
    }
    controller.a().onTrue(teleOpShootCommand);

    // ---------- Configure Light Buttons ----------
    controller.start().and(controller.a()).onTrue(lightsSubsystem.setStaticColorCommand(Color.kDarkGreen));
    controller.start().and(controller.b()).onTrue(lightsSubsystem.setStaticPatternCommand(
        new Color[] { Color.kDarkRed, Color.kDarkRed, Color.kBlack, Color.kBlack }));
    controller.start().and(controller.x())
        .onTrue(lightsSubsystem.setDynamicPatternCommand(
            new Color[] { Color.kBlue, Color.kBlue, Color.kBlue, Color.kBlue, Color.kBlue,
                Color.kNavy, Color.kNavy, Color.kNavy, Color.kNavy, Color.kNavy },
            true));
    controller.start().and(controller.y()).onTrue(lightsSubsystem.setDynamicPatternCommand(
        new Color[] {
            Color.kYellow, Color.kYellow, Color.kYellow, Color.kBlack, Color.kBlack, Color.kBlack,
            Color.kOrange, Color.kOrange, Color.kOrange, Color.kBlack, Color.kBlack, Color.kBlack },
        false));
    controller.leftBumper().onTrue(lightsSubsystem.changeBrightnessCommand(true));
    controller.rightBumper().onTrue(lightsSubsystem.changeBrightnessCommand(false));
    controller.leftBumper().and(controller.rightBumper())
        .onTrue(lightsSubsystem.setStaticColorCommand(Color.kBlack));
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
