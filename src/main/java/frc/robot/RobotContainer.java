package frc.robot;

import static frc.robot.constants.AbstractConstants.CONSTANTS;
import static frc.robot.util.SupplierUtil.not;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.LedCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.ShooterCommands.IntakeCommand;
import frc.robot.subsystems.base.DriveBase;
import frc.robot.subsystems.base.DriveBase.WheelModuleIndex;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Traverser;
import frc.robot.subsystems.gyro.GyroIoPigeon2;
import frc.robot.subsystems.gyro.GyroIoSimAndReplay;
import frc.robot.subsystems.led.Leds;
import frc.robot.subsystems.shooter.Aimer;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Intake;
import frc.robot.subsystems.shooter.NoteSensor;
import frc.robot.subsystems.single_motor.SingleMotorIoNeo550Brushless;
import frc.robot.subsystems.single_motor.SingleMotorIoReplay;
import frc.robot.subsystems.swerve_module.SwerveModuleIoReplay;
import frc.robot.subsystems.swerve_module.SwerveModuleIoSim;
import frc.robot.subsystems.swerve_module.SwerveModuleIoTalonFx;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIoLimelight;
import frc.robot.subsystems.vision.VisionIoSimAndReplay;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /**
     * Pilot's Controller
     * Port 0
     */
    private final CommandXboxController pilot = new CommandXboxController(0);
    /**
     * Co-Pilot's Controller
     * Port 1
     */
    private final CommandXboxController coPilot = new CommandXboxController(1);

    private final LoggedDashboardChooser<Command> autoChooser;

    private final DriveBase driveBase;

    private final Aimer aimer;
    private final Climber climber;
    private final NoteSensor noteSensor;
    private final Feeder feeder;
    private final Flywheel flywheel;
    private final Intake intake;
    private final Leds leds;
    private final Vision vision;
    private final Traverser traverser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // #region: ==================== Initialize Subsystems =================

        // #region: Initialize Subsystems with Simulation and/or Log Replay Mode
        switch (CONSTANTS.getCurrentOperatingMode()) {

            case REAL_WORLD:
                // Real robot, instantiate hardware IO implementations
                driveBase = new DriveBase(
                        new GyroIoPigeon2(CONSTANTS.getGyroId(), CONSTANTS.getCanivoreId()),
                        new SwerveModuleIoTalonFx(WheelModuleIndex.FRONT_LEFT),
                        new SwerveModuleIoTalonFx(WheelModuleIndex.FRONT_RIGHT),
                        new SwerveModuleIoTalonFx(WheelModuleIndex.BACK_LEFT),
                        new SwerveModuleIoTalonFx(WheelModuleIndex.BACK_RIGHT));
                feeder = CONSTANTS.hasFeederSubsystem()
                        ? new Feeder(new SingleMotorIoNeo550Brushless(CONSTANTS.getFeederMotorId(),
                                CONSTANTS.isFeederMortorInverted()))
                        : null;
                intake = CONSTANTS.hasIntakeSubsystem()
                        ? new Intake(new SingleMotorIoNeo550Brushless(CONSTANTS.getIntakeMotorId(),
                                CONSTANTS.isIntakeMortorInverted()))
                        : null;
                vision = CONSTANTS.hasVisionSubsystem()
                        ? new Vision(driveBase.poseEstimator, new VisionIoLimelight(CONSTANTS.getCameraName()),
                                new VisionIoLimelight("limelight-back"))
                        : null;
                traverser = CONSTANTS.hasTraverserSubsystem()
                        ? new Traverser(new SingleMotorIoNeo550Brushless(CONSTANTS.getTraverserMotorId(),
                                CONSTANTS.isTraverserInverted()))
                        : null;
                climber = CONSTANTS.hasClimberSubsystem() ? new Climber() : null;
                break;

            case SIMULATION:
                // Sim robot, instantiate physics sim IO implementations
                driveBase = new DriveBase(
                        new GyroIoSimAndReplay(),
                        new SwerveModuleIoSim(),
                        new SwerveModuleIoSim(),
                        new SwerveModuleIoSim(),
                        new SwerveModuleIoSim());
                feeder = CONSTANTS.hasFeederSubsystem()
                        ? new Feeder(new SingleMotorIoNeo550Brushless(CONSTANTS.getFeederMotorId(),
                                CONSTANTS.isFeederMortorInverted()))
                        : null;
                intake = CONSTANTS.hasIntakeSubsystem()
                        ? new Intake(new SingleMotorIoNeo550Brushless(CONSTANTS.getIntakeMotorId(),
                                CONSTANTS.isIntakeMortorInverted()))
                        : null;
                vision = CONSTANTS.hasVisionSubsystem()
                        ? new Vision(driveBase.poseEstimator, new VisionIoSimAndReplay())
                        : null;
                traverser = CONSTANTS.hasTraverserSubsystem()
                        ? new Traverser(new SingleMotorIoNeo550Brushless(CONSTANTS.getTraverserMotorId(),
                                CONSTANTS.isTraverserInverted()))
                        : null;
                climber = CONSTANTS.hasClimberSubsystem() ? new Climber() : null;
                break;

            case LOG_REPLAY:
                // Replayed robot, disable IO implementations
                driveBase = new DriveBase(
                        new GyroIoSimAndReplay(),
                        new SwerveModuleIoReplay(),
                        new SwerveModuleIoReplay(),
                        new SwerveModuleIoReplay(),
                        new SwerveModuleIoReplay());
                feeder = CONSTANTS.hasFeederSubsystem() ? new Feeder(new SingleMotorIoReplay()) : null;
                intake = CONSTANTS.hasIntakeSubsystem() ? new Intake(new SingleMotorIoReplay()) : null;
                vision = CONSTANTS.hasVisionSubsystem()
                        ? new Vision(driveBase.poseEstimator, new VisionIoSimAndReplay())
                        : null;
                traverser = CONSTANTS.hasTraverserSubsystem()
                        ? new Traverser(new SingleMotorIoReplay())
                        : null;
                climber = CONSTANTS.hasClimberSubsystem() ? new Climber() : null;
                break;

            default:
                throw new RuntimeException("Unknown Run Mode: " + CONSTANTS.getCurrentOperatingMode());
        }

        // #endregion

        // #region: Initialize Subsystems without Simulation and/or Log Replay Mode
        aimer = CONSTANTS.hasAimerSubsystem() ? new Aimer() : null;
        noteSensor = CONSTANTS.hasNoteSensorSubsystem() ? new NoteSensor() : null;
        flywheel = CONSTANTS.hasFlywheelSubsystem() ? new Flywheel() : null;
        /*
         * We can safely set LEDs even if there are no LEDs.
         * (The LED control hardware is built into the RoboRio and therfore always
         * "exists".)
         */
        leds = new Leds();

        // #endregion

        // #region: ==================== Default Commands & Triggers ===========
        // #region: ---------- Configure Default Commands ----------
        driveBase.setDefaultCommand(DriveCommands.manualDriveDefaultCommand(driveBase, pilot::getLeftY, pilot::getLeftX,
                () -> -pilot.getRightX()));
        leds.setDefaultCommand(LedCommands.defaultLedCommand(leds));
        if (CONSTANTS.hasFlywheelSubsystem()) {
            flywheel.setDefaultCommand(ShooterCommands.defaultFlywheelCommand(flywheel));
        }

        // #endregion

        // #region: ---------- Configure Command Triggers ----------
        if (CONSTANTS.hasNoteSensorSubsystem()) {
            new Trigger((noteSensor::isObjectDetectedSwitch)).whileTrue(leds.setColorCommand(Color.kBrown));
        }
        // #endregion
        // #region: ---------- Motor Overheat Triggers ----------
        // TODO: Add LEDs Flashing Yellow to all Motor Temperature cutoff commands.
        new Trigger(driveBase::isTemperatureTooHigh).whileTrue(driveBase.stopCommand());
        if (CONSTANTS.hasIntakeSubsystem()) {
            new Trigger(flywheel::isTemperatureTooHigh).whileTrue(flywheel.stopCommand());
        }

        // TODO: Add LEDs Flashing Yellow to all Motor Temperature cutoff commands.
        if (CONSTANTS.hasIntakeSubsystem()) {
            new Trigger(intake::isTemperatureTooHigh).whileTrue(intake.stopCommand());
        }
        if (CONSTANTS.hasFeederSubsystem()) {
            new Trigger(feeder::isTemperatureTooHigh).whileTrue(feeder.stopCommand());
        }
        if (CONSTANTS.hasTraverserSubsystem()) {
            new Trigger(traverser::isTemperatureTooHigh).whileTrue(traverser.stopCommand());
        }

        // #endregion

        // #region: ==================== Autonomous ============================
        // ---------- Create Named Commands for use by Path Planner ----------
        NamedCommands.registerCommand("Spin 180", DriveCommands.spinCommand(driveBase, Rotation2d.fromDegrees(180), 1));
        NamedCommands.registerCommand("StartIntake", new ShooterCommands.IntakeCommand(intake, feeder));
        if (CONSTANTS.hasFlywheelSubsystem()) {
            NamedCommands.registerCommand("Spin Up Flywheel", ShooterCommands.spinUpFlywheelCommand(flywheel));
        }

        Command aimAtSpeakerCommand = Commands.parallel(
                DriveCommands.turnToTargetCommand(driveBase, CONSTANTS::getSpeakerLocation, 4.5), new InstantCommand(
                        () -> aimer.aimAtTarget(CONSTANTS.getSpeakerLocation(), driveBase.getPose().getTranslation())));
        Command autoShootCommand;
        if (CONSTANTS.hasFeederSubsystem() && CONSTANTS.hasNoteSensorSubsystem()) {
            autoShootCommand = ShooterCommands.shootAutonomousCommand(feeder, leds, noteSensor);
        } else {
            autoShootCommand = LedCommands.blinkCommand(leds, Color.kOrange);
        }
        NamedCommands.registerCommand("Auto Shoot", new SequentialCommandGroup(aimAtSpeakerCommand, autoShootCommand));

        // ---------- Set-up Autonomous Choices ----------
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // #endregion

        // #region: ==================== Tele-Op ===============================
        // #region: ---------- Configure Controller 0 for Pilot ----------
        pilot.leftTrigger().whileTrue(DriveCommands.autoAimAndManuallyDriveCommand(driveBase, flywheel, aimer,
                pilot::getLeftY,
                pilot::getLeftX,
                CONSTANTS::getSpeakerLocation));
        pilot.rightTrigger().whileTrue(DriveCommands.autoAimAndManuallyDriveCommand(driveBase, flywheel, aimer,
                pilot::getLeftY,
                pilot::getLeftX,
                CONSTANTS::getAmpLocation));

        pilot.y().onTrue(driveBase.resetFieldOrientationCommand());

        // #endregion

        // #region: ---------- Configure Controller 1 for Co-Pilot ----------
        if (CONSTANTS.hasIntakeSubsystem() && CONSTANTS.hasFeederSubsystem()) {

            if (CONSTANTS.hasNoteSensorSubsystem()) {
                // TODO: Stop Flywheel as well.
                coPilot.leftTrigger().and(not(noteSensor::isObjectDetectedSwitch))
                        .whileTrue(new IntakeCommand(intake, feeder));
            }

            // TODO - Make one Command in ShooterCommands class.
            coPilot.x().whileTrue(new StartEndCommand(intake::reverse, intake::stop, intake));
            coPilot.x().whileTrue(new StartEndCommand(feeder::reverse, feeder::stop, feeder));
            coPilot.x().whileTrue(new StartEndCommand(flywheel::reverse, flywheel::stop, flywheel));
        }

        if (CONSTANTS.hasFeederSubsystem() && CONSTANTS.hasFlywheelSubsystem()) {

            if (CONSTANTS.hasNoteSensorSubsystem()) {
                coPilot.rightTrigger()
                        .onTrue(ShooterCommands.shootTeleopCommand(feeder, flywheel, intake, noteSensor, leds));
            }
            coPilot.a().whileTrue(ShooterCommands.reverseShooterCommand(flywheel, feeder, leds));
        }

        if (CONSTANTS.hasClimberSubsystem()) {
            coPilot.povUp().whileTrue(climber.incrementTargetHeightCommand(.1));
            coPilot.povDown().whileTrue(climber.incrementTargetHeightCommand(-.1));
        }

        if (CONSTANTS.hasTraverserSubsystem()) {
            // TODO.
        }

        if (CONSTANTS.hasAimerSubsystem()) {
            // TODO: Switch to right joystick.
            coPilot.rightBumper()
                    .whileTrue(new RunCommand(() -> aimer.modifyTargetAngle(Rotation2d.fromDegrees(.5))));
            coPilot.leftBumper()
                    .whileTrue(new RunCommand(() -> aimer.modifyTargetAngle(Rotation2d.fromDegrees(-.5))));
        }

        // #endregion
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