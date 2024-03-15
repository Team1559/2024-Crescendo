package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.PublicTemperature;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.LedCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.io.gyro.GyroIoPigeon2;
import frc.robot.io.gyro.GyroIoSimAndReplay;
import frc.robot.io.motor.MotorIoReplay;
import frc.robot.io.motor.can_spark_max.MotorIoNeo550Brushless;
import frc.robot.io.swerve_module.SwerveModuleIoReplay;
import frc.robot.io.swerve_module.SwerveModuleIoSim;
import frc.robot.io.swerve_module.SwerveModuleIoTalonFx;
import frc.robot.io.vision.VisionIoLimelight;
import frc.robot.io.vision.VisionIoSimAndReplay;
import frc.robot.subsystems.abstract_interface.MotorSubsystem;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Traverser;
import frc.robot.subsystems.drive.SwerveBase;
import frc.robot.subsystems.drive.SwerveModule.WheelModuleIndex;
import frc.robot.subsystems.led.Leds;
import frc.robot.subsystems.shooter.Aimer;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Intake;
import frc.robot.subsystems.shooter.NoteSensor;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.ReflectionUtils;

public class Robot extends LoggedRobot {

    private SwerveBase swerveBase;

    private Aimer aimer;
    private Climber climber;
    private Feeder feeder;
    private Flywheel flywheel;
    private Intake intake;
    private Leds leds;
    private NoteSensor noteSensor;
    private Traverser traverser;

    /** Is used indirectly when its periodic method gets called. */
    @SuppressWarnings("unused")
    private Vision vision;

    private Command autonomousCommand;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {

        // #region: =========== Set Base Units =================================
        // Change Temperature Base Unit from Kelvin to Celsius.
        ReflectionUtils.modifyStaticFinalField(BaseUnits.class, "Temperature",
                new PublicTemperature(x -> x, x -> x, "Celsius", "°C"));
        ReflectionUtils.modifyStaticFinalField(Units.class, "Celsius",
                BaseUnits.Temperature);
        ReflectionUtils.modifyStaticFinalField(Units.class, "Kelvin",
                Units.derive(Units.Celsius).offset(-273.15).named("Kelvin").symbol("K").make());

        // #endregion

        // #region: =========== Set-Up / Start AdvantageKit Logger =============
        // Set up data receivers & replay source.
        switch (Constants.getCurrentOperatingMode()) {

            case REAL_WORLD:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIMULATION:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case LOG_REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;

            default:
                throw new RuntimeException("Unknown Run Mode: " + Constants.getCurrentOperatingMode());
        }

        // Start AdvantageKit logger.
        Logger.start();

        // Log All Commands.
        CommandScheduler.getInstance().onCommandInitialize((command) -> {
            Logger.recordOutput("Commands", command.getName() + ":initialize");
        });
        CommandScheduler.getInstance().onCommandFinish((command) -> {
            Logger.recordOutput("Commands", command.getName() + ":finish");
        });

        // #endregion

        // ==================== Initialize Subsystems ==========================

        // #region: Initialize DriveBase Subsystem.
        switch (Constants.getCurrentOperatingMode()) {
            case REAL_WORLD:
                swerveBase = new SwerveBase(
                        new GyroIoPigeon2(Constants.getGyroId(), Constants.getCanivoreId()),
                        new SwerveModuleIoTalonFx(WheelModuleIndex.FRONT_LEFT),
                        new SwerveModuleIoTalonFx(WheelModuleIndex.FRONT_RIGHT),
                        new SwerveModuleIoTalonFx(WheelModuleIndex.BACK_LEFT),
                        new SwerveModuleIoTalonFx(WheelModuleIndex.BACK_RIGHT));
                break;
            case SIMULATION:
                swerveBase = SwerveBase.createSimOrReplaySwerveBase(new GyroIoSimAndReplay(),
                        new SwerveModuleIoSim(DCMotor.getKrakenX60(1), DCMotor.getFalcon500(1)));
                break;
            case LOG_REPLAY:
                swerveBase = SwerveBase.createSimOrReplaySwerveBase(new GyroIoSimAndReplay(),
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
                        ? new Vision(swerveBase.poseEstimator, new VisionIoLimelight(Constants.getCameraNameFront()),
                                new VisionIoLimelight(Constants.getCameraNameBack()))
                        : null;
                break;
            case SIMULATION:
            case LOG_REPLAY:
                vision = Constants.hasVisionSubsystem()
                        ? new Vision(swerveBase.poseEstimator, new VisionIoSimAndReplay())
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

        // #region: =========== Default Commands & Triggers ====================

        // #region: ---------- Configure Default Commands ----------

        leds.setDefaultCommand(LedCommands.defaultLedCommand(leds));

        // #endregion

        // #region: ---------- Motor Overheat Triggers ----------
        new Trigger(swerveBase::isTemperatureTooHigh)
                .whileTrue(DriveCommands.overheatedMotorShutdownCommand(swerveBase, leds));

        for (MotorSubsystem motorSubsystem : MotorSubsystem.instantiatedSubsystems) {
            new Trigger(motorSubsystem::isTemperatureTooHigh)
                    .whileTrue(ShooterCommands.overheatedMotorShutdownCommand(motorSubsystem, leds));
        }

        // #endregion

    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        // Unused.
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        // Unused.
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {

        // #region: ---------- Create Named Commands for use by Path Planner ----------

        NamedCommands.registerCommand("Spin 180",
                DriveCommands.spinCommand(swerveBase, Rotation2d.fromDegrees(180), 1));

        if (Constants.hasFeederSubsystem() && Constants.hasIntakeSubsystem()) {
            NamedCommands.registerCommand("StartIntake", ShooterCommands.intakeStartStopCommand(feeder, intake));
        }

        if (Constants.hasFlywheelSubsystem()) {
            NamedCommands.registerCommand("Spin Up Flywheel", ShooterCommands.spinUpFlywheelCommand(flywheel));
        }

        if (Constants.hasAimerSubsystem() && Constants.hasFeederSubsystem() && Constants.hasIntakeSubsystem()
                && Constants.hasNoteSensorSubsystem()) {

            NamedCommands.registerCommand("Auto Shoot",
                    DriveCommands.autoShootCommand(swerveBase, aimer, feeder, intake, noteSensor));

            NamedCommands.registerCommand("Initial Shoot",
                    ShooterCommands.autoJustShootCommand(aimer, feeder, intake, noteSensor));

            NamedCommands.registerCommand("Delayed Manual Shot",
                    ShooterCommands.autoDelayedManualShotCommand(aimer, feeder, intake, noteSensor));
        }

        // #endregion

        // #region: ---------- Set-up Autonomous Choices ----------

        LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices",
                AutoBuilder.buildAutoChooser());

        // #endregion

        // #region: ---------- Perform Initialize Actions ----------

        if (Constants.hasAimerSubsystem()) {
            CommandScheduler.getInstance()
                    .schedule(aimer.setAngleCommand(Rotation2d.fromDegrees(2)));
        }

        // #endregion

        // #region: ---------- Schedule Autonomous Command ----------

        autonomousCommand = autoChooser.get();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }

        // #endregion
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        // Unused.
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {

        // This makes sure that the autonomous stops running when teleop starts running.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        CommandXboxController pilot = new CommandXboxController(0);
        CommandXboxController coPilot = new CommandXboxController(1);

        // #region: ---------- Configure Default Commands ----------

        swerveBase.setDefaultCommand(DriveCommands.manualDriveDefaultCommand(swerveBase,
                pilot::getLeftY, pilot::getLeftX, pilot::getRightX));

        if (Constants.hasFlywheelSubsystem()) {
            flywheel.setDefaultCommand(ShooterCommands.defaultFlywheelCommand(flywheel));
        }

        // #endregion

        // #region: ---------- Configure Command Triggers ----------

        if (Constants.hasNoteSensorSubsystem()) {
            new Trigger(noteSensor::isObjectDetectedOnSwitch).whileTrue(leds.setColorCommand(Color.kGreen));
        }

        // TODO: Add LED Trigger for Ready to Shoot.

        // #endregion

        // #region: ---------- Configure Controller 0 for Pilot ----------

        pilot.leftTrigger().whileTrue(DriveCommands.autoAimAndManuallyDriveCommand(swerveBase, aimer, flywheel,
                pilot::getLeftY, pilot::getLeftX,
                Constants::getSpeakerLocation));
        pilot.rightTrigger().whileTrue(DriveCommands.autoAimAndManuallyDriveCommand(swerveBase, aimer, flywheel,
                pilot::getLeftY, pilot::getLeftX,
                Constants::getAmpLocation));
        pilot.leftTrigger().onFalse(aimer.setAngleCommand(Rotation2d.fromDegrees(2)));
        pilot.y().onTrue(swerveBase.resetFieldOrientationCommand());

        // #endregion

        // #region: ---------- Configure Controller 1 for Co-Pilot ----------

        if (Constants.hasFeederSubsystem() && Constants.hasFlywheelSubsystem() && Constants.hasIntakeSubsystem()) {

            if (Constants.hasNoteSensorSubsystem()) {
                coPilot.leftTrigger().and(noteSensor::isObjectNotDetectedSwitch)
                        .whileTrue(ShooterCommands.runIntakeCommand(feeder, flywheel, intake));

            }
            coPilot.x().whileTrue(ShooterCommands.reverseShooterAndIntakeCommand(feeder, flywheel, intake));
        }

        if (Constants.hasFeederSubsystem() && Constants.hasFlywheelSubsystem()) {

            if (Constants.hasIntakeSubsystem() && Constants.hasNoteSensorSubsystem()) {
                coPilot.rightTrigger().onTrue(ShooterCommands.shootTeleopCommand(feeder, flywheel, intake, noteSensor));
            }
            coPilot.a().whileTrue(ShooterCommands.reverseShooterCommand(feeder, flywheel, leds));
        }

        if (Constants.hasClimberSubsystem()) {

            Trigger noModifier = new Trigger(coPilot.y().negate().and(coPilot.b().negate()));

            coPilot.povUp().and(noModifier).whileTrue(climber.modifyHeightCommand(Inches.of(0.1)).repeatedly());
            coPilot.povUp().and(coPilot.y()).whileTrue(climber.modifyHeightLeftCommand(Inches.of(0.1)).repeatedly());
            coPilot.povUp().and(coPilot.b()).whileTrue(climber.modifyHeightRightCommand(Inches.of(0.1)).repeatedly());

            coPilot.povDown().and(noModifier).whileTrue(climber.modifyHeightCommand(Inches.of(-0.1)).repeatedly());
            coPilot.povDown().and(coPilot.y()).whileTrue(climber.modifyHeightLeftCommand(Inches.of(-0.1)).repeatedly());
            coPilot.povDown().and(coPilot.b()).whileTrue(climber.modifyHeightRightCommand(Inches.of(-.1)).repeatedly());
        }

        if (Constants.hasTraverserSubsystem()) {
            coPilot.povRight().whileTrue(traverser.traverserRightThenStopCommand());
            coPilot.povLeft().whileTrue(traverser.traverserLeftThenStopCommand());
        }

        if (Constants.hasAimerSubsystem()) {
            coPilot.rightBumper().whileTrue(aimer.modifyAngleCommand(Rotation2d.fromDegrees(0.5)));
            coPilot.leftBumper().whileTrue(aimer.modifyAngleCommand(Rotation2d.fromDegrees(-0.5)));
        }

        // #endregion
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // Unused.
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        // Unused.
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
        // Unused.
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        // Unused.
    }
}