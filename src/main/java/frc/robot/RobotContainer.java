package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;

import frc.robot.subsystems.base.DriveBase;
import frc.robot.subsystems.base.DriveBase.WheelModuleIndex;
import frc.robot.subsystems.gyro.GyroIoPigeon2;
import frc.robot.subsystems.gyro.GyroIoSimAndReplay;
import frc.robot.subsystems.swerve.SwerveModuleIoReplay;
import frc.robot.subsystems.swerve.SwerveModuleIoSim;
import frc.robot.subsystems.swerve.SwerveModuleIoTalonFx;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  private final CommandXboxController controller = new CommandXboxController(0);
  private final DriveBase driveBase;
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
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
        break;

      case SIMULATION:
        // Sim robot, instantiate physics sim IO implementations
        driveBase = new DriveBase(
          new GyroIoSimAndReplay() {},
          new SwerveModuleIoSim(),
          new SwerveModuleIoSim(),
          new SwerveModuleIoSim(),
          new SwerveModuleIoSim());
        break;

      case LOG_REPLAY:
        // Replayed robot, disable IO implementations
        driveBase = new DriveBase(
          new GyroIoSimAndReplay() {},
          new SwerveModuleIoReplay() {},
          new SwerveModuleIoReplay() {},
          new SwerveModuleIoReplay() {},
          new SwerveModuleIoReplay() {});
        break;

      default:
        throw new RuntimeException("Unknown Run Mode: " + Constants.CURRENT_OPERATING_MODE);
    }

    // ---------- Configure Joystick for Tele-Op ----------
    driveBase.setDefaultCommand(DriveCommands.joystickDrive(driveBase,
      () -> -controller.getLeftY(),
      () -> -controller.getLeftX(),
      () -> -controller.getRightX()
    ));
  
    // TODO: Drive Forward.
    //controller.povUp().onTrue(Commands.run());
    // TODO: Drive Backwards.
    //controller.povDown().onTrue(Commands.run());
    // TODO: Drive Right.
    //controller.povRight().onTrue(Commands.run());
    // TODO: Drive Left.
    //controller.povLeft().onTrue(Commands.run());

    // ---------- Set-up Autonomous Choices ----------
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
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