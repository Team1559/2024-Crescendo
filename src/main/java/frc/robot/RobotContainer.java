package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.util.SupplierUtil.not;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.LedCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.io.gyro.GyroIoPigeon2;
import frc.robot.io.gyro.GyroIoSimAndReplay;
import frc.robot.io.motor.MotorIoNeo550Brushless;
import frc.robot.io.motor.MotorIoReplay;
import frc.robot.io.swerve_module.SwerveModuleIoReplay;
import frc.robot.io.swerve_module.SwerveModuleIoSim;
import frc.robot.io.swerve_module.SwerveModuleIoTalonFx;
import frc.robot.io.vision.VisionIoLimelight;
import frc.robot.io.vision.VisionIoSimAndReplay;
import frc.robot.subsystems.base.SwerveBase;
import frc.robot.subsystems.base.SwerveModule.WheelModuleIndex;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Traverser;
import frc.robot.subsystems.led.Leds;
import frc.robot.subsystems.shooter.Aimer;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Intake;
import frc.robot.subsystems.shooter.NoteSensor;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer { // TODO: Merge into the Robot class.
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

    private final SwerveBase driveBase;

    final Aimer aimer;
    final Climber climber;
    private final Feeder feeder;
    private final Flywheel flywheel;
    private final Intake intake;
    private final Leds leds;
    private final NoteSensor noteSensor;
    private final Traverser traverser;

    /** Is used indirectly when its periodic method gets called. */
    @SuppressWarnings("unused")
    private final Vision vision;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // #region: ==================== Initialize Subsystems =================

        // #region: Initialize DriveBase Subsystem.
        switch (Constants.getCurrentOperatingMode()) {
            case REAL_WORLD:
                driveBase = new SwerveBase(
                        new GyroIoPigeon2(Constants.getGyroId(), Constants.getCanivoreId()),
                        new SwerveModuleIoTalonFx(WheelModuleIndex.FRONT_LEFT),
                        new SwerveModuleIoTalonFx(WheelModuleIndex.FRONT_RIGHT),
                        new SwerveModuleIoTalonFx(WheelModuleIndex.BACK_LEFT),
                        new SwerveModuleIoTalonFx(WheelModuleIndex.BACK_RIGHT));
                break;
            case SIMULATION:
                driveBase = new SwerveBase(
                        new GyroIoSimAndReplay(),
                        new SwerveModuleIoSim(DCMotor.getKrakenX60(1), DCMotor.getFalcon500(1)),
                        new SwerveModuleIoSim(DCMotor.getKrakenX60(1), DCMotor.getFalcon500(1)),
                        new SwerveModuleIoSim(DCMotor.getKrakenX60(1), DCMotor.getFalcon500(1)),
                        new SwerveModuleIoSim(DCMotor.getKrakenX60(1), DCMotor.getFalcon500(1)));
                break;
            case LOG_REPLAY:
                driveBase = new SwerveBase(
                        new GyroIoSimAndReplay(),
                        new SwerveModuleIoReplay(),
                        new SwerveModuleIoReplay(),
                        new SwerveModuleIoReplay(),
                        new SwerveModuleIoReplay());
                break;
            default:
                throw new RuntimeException("Unknown Run Mode: " + Constants.getCurrentOperatingMode());
        }

        // #endregion

        // #region: Initialize SingleMotorSubsystems.
        switch (Constants.getCurrentOperatingMode()) {
            case REAL_WORLD:
            case SIMULATION:
                feeder = Constants.hasFeederSubsystem()
                        ? new Feeder(new MotorIoNeo550Brushless(Constants.getFeederMotorId(),
                                Constants.isFeederMotorInverted(), IdleMode.kBrake, Rotation2d.fromRotations(0), // TODO
                                Constants.getFeederPidValues()))
                        : null;
                intake = Constants.hasIntakeSubsystem()
                        ? new Intake(new MotorIoNeo550Brushless(Constants.getIntakeMotorId(),
                                Constants.isIntakeMotorInverted(), IdleMode.kBrake, Rotation2d.fromRotations(0), // TODO
                                Constants.getIntakePidValues()))
                        : null;
                traverser = Constants.hasTraverserSubsystem()
                        ? new Traverser(new MotorIoNeo550Brushless(Constants.getTraverserMotorId(),
                                Constants.isTraverserInverted(), IdleMode.kBrake, Rotation2d.fromRotations(0), // TODO
                                Constants.getTraverserPidValues()))
                        : null;
                break;
            case LOG_REPLAY:
                feeder = Constants.hasFeederSubsystem() ? new Feeder(new MotorIoReplay()) : null;
                intake = Constants.hasIntakeSubsystem() ? new Intake(new MotorIoReplay()) : null;
                traverser = Constants.hasTraverserSubsystem() ? new Traverser(new MotorIoReplay()) : null;
                break;
            default:
                throw new RuntimeException("Unknown Run Mode: " + Constants.getCurrentOperatingMode());
        }

        // #endregion

        // #region: Initialize Vision Subsystem.
        switch (Constants.getCurrentOperatingMode()) {
            case REAL_WORLD:
                vision = Constants.hasVisionSubsystem()
                        ? new Vision(driveBase.poseEstimator, new VisionIoLimelight(Constants.getCameraNameFront()),
                                new VisionIoLimelight(Constants.getCameraNameBack()))
                        : null;
                break;
            case SIMULATION:
            case LOG_REPLAY:
                vision = Constants.hasVisionSubsystem()
                        ? new Vision(driveBase.poseEstimator, new VisionIoSimAndReplay())
                        : null;
                break;
            default:
                throw new RuntimeException("Unknown Run Mode: " + Constants.getCurrentOperatingMode());
        }

        // #endregion

        // #region: Initialize Dual Motor Subsystems.
        aimer = Constants.hasAimerSubsystem() ? new Aimer() : null;
        climber = Constants.hasClimberSubsystem() ? new Climber() : null;
        flywheel = Constants.hasFlywheelSubsystem() ? new Flywheel() : null;

        // #endregion

        // #region: Initialize Lights & Sensors.
        /*
         * We can safely set LEDs even if there are no LEDs.
         * (LED hardware is built into the RoboRio and therefore always "exists".)
         */
        leds = new Leds();
        noteSensor = Constants.hasNoteSensorSubsystem() ? new NoteSensor(Constants.getNoteSensorChannel()) : null;

        // #endregion

        // #region: ==================== Default Commands & Triggers ===========
        // #region: ---------- Configure Default Commands ----------
        driveBase.setDefaultCommand(
                DriveCommands.manualDriveDefaultCommand(driveBase, pilot::getLeftY, pilot::getLeftX, pilot::getRightX));
        if (Constants.hasFlywheelSubsystem()) {
            flywheel.setDefaultCommand(ShooterCommands.defaultFlywheelCommand(flywheel));
        }
        leds.setDefaultCommand(LedCommands.defaultLedCommand(leds));

        // #endregion

        // #region: ---------- Configure Command Triggers ----------
        if (Constants.hasNoteSensorSubsystem()) {
            new Trigger((noteSensor::isObjectDetected)).whileTrue(leds.setColorCommand(Color.kGreen));
        }
        // TODO: Add LED Trigger for Ready to Shoot.
        // #endregion

        // #region: ---------- Motor Overheat Triggers ----------
        new Trigger(driveBase::isTemperatureTooHigh)
                .whileTrue(driveBase.stopCommand()
                        .alongWith(leds.setDynamicPatternCommand(Constants.getMotorOverheatEmergencyPattern(), false)));
        if (Constants.hasFlywheelSubsystem()) {
            new Trigger(flywheel::isTemperatureTooHigh).whileTrue(flywheel.stopCommand()
                    .alongWith(leds.setDynamicPatternCommand(Constants.getMotorOverheatEmergencyPattern(), false)));
        }
        // TODO: Make a generic SingleMotorSubsystem overheat command.
        if (Constants.hasIntakeSubsystem()) {
            new Trigger(intake::isTemperatureTooHigh).whileTrue(intake.stopCommand()
                    .alongWith(leds.setDynamicPatternCommand(Constants.getMotorOverheatEmergencyPattern(), false)));
        }
        if (Constants.hasFeederSubsystem()) {
            new Trigger(feeder::isTemperatureTooHigh).whileTrue(feeder.stopCommand()
                    .alongWith(leds.setDynamicPatternCommand(Constants.getMotorOverheatEmergencyPattern(), false)));
        }
        if (Constants.hasTraverserSubsystem()) {
            new Trigger(traverser::isTemperatureTooHigh).whileTrue(traverser.stopCommand()
                    .alongWith(leds.setDynamicPatternCommand(Constants.getMotorOverheatEmergencyPattern(), false)));
        }

        // #endregion

        // #region: ==================== Autonomous ============================
        // ---------- Create Named Commands for use by Path Planner ----------
        NamedCommands.registerCommand("Spin 180", DriveCommands.spinCommand(driveBase, Rotation2d.fromDegrees(180), 1));
        if (Constants.hasIntakeSubsystem() && Constants.hasFeederSubsystem()) {
            NamedCommands.registerCommand("StartIntake", ShooterCommands.intakeStartStopCommand(intake, feeder));
        }
        if (Constants.hasFlywheelSubsystem()) {
            NamedCommands.registerCommand("Spin Up Flywheel", ShooterCommands.spinUpFlywheelCommand(flywheel));
        }
        if (Constants.hasFeederSubsystem() && Constants.hasNoteSensorSubsystem() && Constants.hasAimerSubsystem()) {
            NamedCommands.registerCommand("Auto Shoot", new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            DriveCommands.turnToTargetCommand(driveBase, Constants::getSpeakerLocation, 4.5),
                            aimer.aimAtTargetCommand(Constants::getSpeakerLocation, driveBase::getTranslation)
                                    .andThen(aimer.waitUntilAtTargetCommand())),
                    ShooterCommands.shootAutonomousCommand(feeder, leds, noteSensor)));
        }

        // ---------- Set-up Autonomous Choices ----------
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // #endregion

        // #region: ==================== Tele-Op ===============================
        // #region: ---------- Configure Controller 0 for Pilot ----------
        pilot.leftTrigger().whileTrue(DriveCommands.autoAimAndManuallyDriveCommand(driveBase, flywheel, aimer,
                pilot::getLeftY, pilot::getLeftX,
                Constants::getSpeakerLocation));
        pilot.rightTrigger().whileTrue(DriveCommands.autoAimAndManuallyDriveCommand(driveBase, flywheel, aimer,
                pilot::getLeftY, pilot::getLeftX,
                Constants::getAmpLocation));
        pilot.y().onTrue(driveBase.resetFieldOrientationCommand());
        // #endregion

        // #region: ---------- Configure Controller 1 for Co-Pilot ----------
        if (Constants.hasIntakeSubsystem() && Constants.hasFeederSubsystem() && Constants.hasFlywheelSubsystem()) {

            if (Constants.hasNoteSensorSubsystem()) {
                coPilot.leftTrigger().and(not(noteSensor::isObjectDetected)).whileTrue(new ParallelCommandGroup(
                        ShooterCommands.intakeStartStopCommand(intake, feeder), flywheel.stopCommand()));
            }
            coPilot.x().whileTrue(ShooterCommands.reverseShooterAndIntakeCommand(intake, feeder, flywheel));
        }

        if (Constants.hasFeederSubsystem() && Constants.hasFlywheelSubsystem()) {

            if (Constants.hasIntakeSubsystem() && Constants.hasNoteSensorSubsystem()) {
                coPilot.rightTrigger()
                        .onTrue(ShooterCommands.shootTeleopCommand(feeder, flywheel, intake, noteSensor, leds));
            }
            coPilot.a().whileTrue(ShooterCommands.reverseShooterCommand(flywheel, feeder, leds));
        }

        if (Constants.hasClimberSubsystem()) {
            coPilot.povUp().whileTrue(climber.modifyHeightCommand(Inches.of(0.05)));
            coPilot.povDown().whileTrue(climber.modifyHeightCommand(Inches.of(-0.1)));
        }

        if (Constants.hasTraverserSubsystem()) {
            coPilot.povRight().whileTrue(traverser.traverserRightStopCommand());
            coPilot.povLeft().whileTrue(traverser.traverserLeftStopCommand());
        }

        if (Constants.hasAimerSubsystem()) {
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