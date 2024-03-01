package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.constants.AbstractConstants.CONSTANTS;
import static frc.robot.util.SupplierUtil.not;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.LedCommands;
import frc.robot.commands.ShooterCommands;
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
    final Climber climber;
    private final Feeder feeder;
    private final Flywheel flywheel;
    private final Intake intake;
    private final Leds leds;
    private final NoteSensor noteSensor;
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
                                CONSTANTS.isFeederMotorInverted(), CONSTANTS.getFeederPidValues()))
                        : null;
                intake = CONSTANTS.hasIntakeSubsystem()
                        ? new Intake(new SingleMotorIoNeo550Brushless(CONSTANTS.getIntakeMotorId(),
                                CONSTANTS.isIntakeMotorInverted(), CONSTANTS.getIntakePidValues()))
                        : null;
                traverser = CONSTANTS.hasTraverserSubsystem()
                        ? new Traverser(new SingleMotorIoNeo550Brushless(CONSTANTS.getTraverserMotorId(),
                                CONSTANTS.isTraverserInverted(), CONSTANTS.getTraverserPidValues()))
                        : null;
                vision = CONSTANTS.hasVisionSubsystem()
                        ? new Vision(driveBase.poseEstimator, new VisionIoLimelight(CONSTANTS.getCameraNameFront()),
                                new VisionIoLimelight(CONSTANTS.getCameraNameBack()))
                        : null;
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
                                CONSTANTS.isFeederMotorInverted(), CONSTANTS.getIntakePidValues()))
                        : null;
                intake = CONSTANTS.hasIntakeSubsystem()
                        ? new Intake(new SingleMotorIoNeo550Brushless(CONSTANTS.getIntakeMotorId(),
                                CONSTANTS.isIntakeMotorInverted(), CONSTANTS.getIntakePidValues()))
                        : null;
                traverser = CONSTANTS.hasTraverserSubsystem()
                        ? new Traverser(new SingleMotorIoNeo550Brushless(CONSTANTS.getTraverserMotorId(),
                                CONSTANTS.isTraverserInverted(), CONSTANTS.getTraverserPidValues()))
                        : null;
                vision = CONSTANTS.hasVisionSubsystem()
                        ? new Vision(driveBase.poseEstimator, new VisionIoSimAndReplay())
                        : null;
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
                traverser = CONSTANTS.hasTraverserSubsystem() ? new Traverser(new SingleMotorIoReplay()) : null;
                vision = CONSTANTS.hasVisionSubsystem()
                        ? new Vision(driveBase.poseEstimator, new VisionIoSimAndReplay())
                        : null;
                break;

            default:
                throw new RuntimeException("Unknown Run Mode: " + CONSTANTS.getCurrentOperatingMode());
        }

        // #endregion

        // #region: Initialize Subsystems without Simulation and/or Log Replay Mode
        aimer = CONSTANTS.hasAimerSubsystem() ? new Aimer() : null;
        climber = CONSTANTS.hasClimberSubsystem() ? new Climber() : null;
        flywheel = CONSTANTS.hasFlywheelSubsystem() ? new Flywheel() : null;
        noteSensor = CONSTANTS.hasNoteSensorSubsystem() ? new NoteSensor(CONSTANTS.getLimitSwitchChannel()) : null;
        /*
         * We can safely set LEDs even if there are no LEDs.
         * (The LED control hardware is built into the RoboRio and therfore always
         * "exists".)
         */
        leds = new Leds();

        // #endregion

        // #region: ==================== Default Commands & Triggers ===========
        // #region: ---------- Configure Default Commands ----------
        driveBase.setDefaultCommand(
                DriveCommands.manualDriveDefaultCommand(driveBase, pilot::getLeftY, pilot::getLeftX, pilot::getRightX));
        if (CONSTANTS.hasFlywheelSubsystem()) {
            flywheel.setDefaultCommand(ShooterCommands.defaultFlywheelCommand(flywheel));
        }
        leds.setDefaultCommand(LedCommands.defaultLedCommand(leds));

        // #endregion

        // #region: ---------- Configure Command Triggers ----------
        if (CONSTANTS.hasNoteSensorSubsystem()) {
            new Trigger((noteSensor::isObjectDetected)).whileTrue(leds.setColorCommand(Color.kGreen));
        }
        // TODO: Add LED Trigger for Ready to Shoot.
        // #endregion

        // #region: ---------- Motor Overheat Triggers ----------
        new Trigger(driveBase::isTemperatureTooHigh)
                .whileTrue(driveBase.stopCommand()
                        .alongWith(leds.setDynamicPatternCommand(CONSTANTS.OVERHEAT_EMERGENCY_PATTERN, false)));
        if (CONSTANTS.hasFlywheelSubsystem()) {
            new Trigger(flywheel::isTemperatureTooHigh).whileTrue(flywheel.stopCommand()
                    .alongWith(leds.setDynamicPatternCommand(CONSTANTS.OVERHEAT_EMERGENCY_PATTERN, false)));
        }
        // TODO: Make a generic SingleMotorSubsystem overheat command.
        if (CONSTANTS.hasIntakeSubsystem()) {
            new Trigger(intake::isTemperatureTooHigh).whileTrue(intake.stopCommand()
                    .alongWith(leds.setDynamicPatternCommand(CONSTANTS.OVERHEAT_EMERGENCY_PATTERN, false)));
        }
        if (CONSTANTS.hasFeederSubsystem()) {
            new Trigger(feeder::isTemperatureTooHigh).whileTrue(feeder.stopCommand()
                    .alongWith(leds.setDynamicPatternCommand(CONSTANTS.OVERHEAT_EMERGENCY_PATTERN, false)));
        }
        if (CONSTANTS.hasTraverserSubsystem()) {
            new Trigger(traverser::isTemperatureTooHigh).whileTrue(traverser.stopCommand()
                    .alongWith(leds.setDynamicPatternCommand(CONSTANTS.OVERHEAT_EMERGENCY_PATTERN, false)));
        }

        // #endregion

        // #region: ==================== Autonomous ============================
        // ---------- Create Named Commands for use by Path Planner ----------
        NamedCommands.registerCommand("Spin 180", DriveCommands.spinCommand(driveBase, Rotation2d.fromDegrees(180), 1));
        if (CONSTANTS.hasIntakeSubsystem() && CONSTANTS.hasFeederSubsystem()) {
            NamedCommands.registerCommand("StartIntake", ShooterCommands.intakeStartStopCommand(intake, feeder));
        }
        if (CONSTANTS.hasFlywheelSubsystem()) {
            NamedCommands.registerCommand("Spin Up Flywheel", ShooterCommands.spinUpFlywheelCommand(flywheel));
        }
        if (CONSTANTS.hasFeederSubsystem() && CONSTANTS.hasNoteSensorSubsystem() && CONSTANTS.hasAimerSubsystem()) {
            NamedCommands.registerCommand("Auto Shoot", new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            DriveCommands.turnToTargetCommand(driveBase, CONSTANTS::getSpeakerLocation, 4.5),
                            aimer.aimAtTargetCommand(CONSTANTS.getSpeakerLocation(), driveBase.getTranslation())),
                    ShooterCommands.shootAutonomousCommand(feeder, leds, noteSensor)));
        }

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
        if (CONSTANTS.hasIntakeSubsystem() && CONSTANTS.hasFeederSubsystem() && CONSTANTS.hasFlywheelSubsystem()) {

            if (CONSTANTS.hasNoteSensorSubsystem()) {
                coPilot.leftTrigger().and(not(noteSensor::isObjectDetected)).whileTrue(new ParallelCommandGroup(
                        ShooterCommands.intakeStartStopCommand(intake, feeder), flywheel.stopCommand()));
            }
            coPilot.x().whileTrue(ShooterCommands.reverseShooterAndIntakeCommand(intake, feeder, flywheel));
        }

        if (CONSTANTS.hasFeederSubsystem() && CONSTANTS.hasFlywheelSubsystem()) {

            if (CONSTANTS.hasIntakeSubsystem() && CONSTANTS.hasNoteSensorSubsystem()) {
                coPilot.rightTrigger()
                        .onTrue(ShooterCommands.shootTeleopCommand(feeder, flywheel, intake, noteSensor, leds));
            }
            coPilot.a().whileTrue(ShooterCommands.reverseShooterCommand(flywheel, feeder, leds));
        }

        if (CONSTANTS.hasClimberSubsystem()) {
            coPilot.povUp().whileTrue(climber.modifyHeightCommand(Inches.of(0.1)));
            coPilot.povDown().whileTrue(climber.modifyHeightCommand(Inches.of(-0.1)));
        }

        if (CONSTANTS.hasTraverserSubsystem()) {
            coPilot.povRight().whileTrue(traverser.traverserRightStartStopCommand());
            coPilot.povLeft().whileTrue(traverser.traverserLeftStartStopCommand());
        }

        if (CONSTANTS.hasAimerSubsystem()) {
            coPilot.rightBumper().whileTrue(aimer.modifyAngleCommand(Rotation2d.fromDegrees(0.5)));
            coPilot.leftBumper().whileTrue(aimer.modifyAngleCommand(Rotation2d.fromDegrees(-0.5)));
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