package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.signals.NeutralModeValue;
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
import frc.robot.io.encoder.EncoderIoGenericPmw;
import frc.robot.io.encoder.EncoderIoReplay;
import frc.robot.io.encoder.EncoderIoSimulation;
import frc.robot.io.gyro.GyroIoPigeon2;
import frc.robot.io.gyro.GyroIoSimAndReplay;
import frc.robot.io.motor.MotorIoReplay;
import frc.robot.io.motor.MotorIoSimulation;
import frc.robot.io.motor.can_spark_max.MotorIoNeo550Brushless;
import frc.robot.io.motor.talon_fx.MotorIoFalcon500;
import frc.robot.io.vision.VisionIoLimelight;
import frc.robot.io.vision.VisionIoSimAndReplay;
import frc.robot.subsystems.abstract_interface.MotorSubsystem;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Traverser;
import frc.robot.subsystems.drive.SwerveBase;
import frc.robot.subsystems.drive.SwerveModule;
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

    private final SwerveBase swerveBase;

    private final Aimer aimer;
    private final Climber climber;
    private final Feeder feeder;
    private final Flywheel flywheel;
    private final Intake intake;
    private final Leds leds;
    private final NoteSensor noteSensor;
    private final Traverser traverser;

    /** Is used indirectly when its periodic method gets called. */
    @SuppressWarnings("unused")
    private final Vision vision;

    private Command autonomousCommand;

    public Robot() {

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

        // #region: Subsystems with Simulation & Log Replay.
        switch (Constants.getCurrentOperatingMode()) {
            case REAL_WORLD:
                swerveBase = new SwerveBase(
                        new GyroIoPigeon2(Constants.getGyroId(), Constants.getCanivoreId()),
                        SwerveModule.createRealSwerveModule(WheelModuleIndex.FRONT_LEFT),
                        SwerveModule.createRealSwerveModule(WheelModuleIndex.FRONT_RIGHT),
                        SwerveModule.createRealSwerveModule(WheelModuleIndex.BACK_LEFT),
                        SwerveModule.createRealSwerveModule(WheelModuleIndex.BACK_RIGHT));
                aimer = Constants.hasAimerSubsystem() ? new Aimer(
                        new MotorIoNeo550Brushless(Constants.getAimerMotorIdLeft(), false, IdleMode.kBrake,
                                Rotation2d.fromRotations(0), null),
                        new MotorIoNeo550Brushless(Constants.getAimerMotorIdRight(), true, IdleMode.kBrake,
                                Rotation2d.fromRotations(0), null),
                        new EncoderIoGenericPmw(Constants.getAimerEncoderPort(), true, Rotation2d.fromRadians(0)))
                        : null;
                climber = Constants.hasClimberSubsystem() ? new Climber(
                        new MotorIoNeo550Brushless(Constants.getClimberMotorIdLeft(), true, IdleMode.kBrake,
                                Rotation2d.fromRotations(0), Constants.getClimberPid()),
                        new MotorIoNeo550Brushless(Constants.getClimberMotorIdRight(), false, IdleMode.kBrake,
                                Rotation2d.fromRotations(0), Constants.getClimberPid()))
                        : null;
                feeder = Constants.hasFeederSubsystem()
                        ? new Feeder(new MotorIoNeo550Brushless(Constants.getFeederMotorId(),
                                Constants.isFeederMotorInverted(), IdleMode.kBrake, Rotation2d.fromRotations(0), // TODO
                                Constants.getFeederPidValues()))
                        : null;
                flywheel = Constants.hasFlywheelSubsystem() ? new Flywheel(
                        new MotorIoFalcon500(Constants.getFlywheelMotorIdLeft(), null, false, NeutralModeValue.Coast,
                                Rotation2d.fromRotations(0), null),
                        new MotorIoFalcon500(Constants.getFlywheelMotorIdRight(), null, true, NeutralModeValue.Coast,
                                Rotation2d.fromRotations(0), null))
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
                vision = Constants.hasVisionSubsystem()
                        ? new Vision(swerveBase.poseEstimator, new VisionIoLimelight(Constants.getCameraNameFront()),
                                new VisionIoLimelight(Constants.getCameraNameBack()))
                        : null;
                break;
            case SIMULATION:
                MotorIoSimulation simulatedMotor;
                swerveBase = new SwerveBase(new GyroIoSimAndReplay(),
                        SwerveModule.createSimulationSwerveModule(WheelModuleIndex.FRONT_LEFT),
                        SwerveModule.createSimulationSwerveModule(WheelModuleIndex.FRONT_RIGHT),
                        SwerveModule.createSimulationSwerveModule(WheelModuleIndex.BACK_LEFT),
                        SwerveModule.createSimulationSwerveModule(WheelModuleIndex.BACK_RIGHT));
                aimer = Constants.hasAimerSubsystem() ? new Aimer(
                        new MotorIoSimulation(DCMotor.getNeo550(1), 1 /* TODO */, MetersPerSecond.zero() /* TODO */),
                        (simulatedMotor = new MotorIoSimulation(DCMotor.getNeo550(1), 1 /* TODO */,
                                MetersPerSecond.zero() /* TODO */)),
                        new EncoderIoSimulation(simulatedMotor))
                        : null;
                climber = Constants.hasClimberSubsystem() ? new Climber(
                        new MotorIoSimulation(DCMotor.getNeo550(1), 1 /* TODO */, MetersPerSecond.zero() /* TODO */),
                        new MotorIoSimulation(DCMotor.getNeo550(1), 1 /* TODO */, MetersPerSecond.zero() /* TODO */))
                        : null;
                feeder = Constants.hasFeederSubsystem()
                        ? new Feeder(new MotorIoSimulation(DCMotor.getNeo550(1), 1 /* TODO */,
                                MetersPerSecond.zero() /* TODO */))
                        : null;
                flywheel = Constants.hasFlywheelSubsystem() ? new Flywheel(
                        new MotorIoSimulation(DCMotor.getFalcon500(1), 1, MetersPerSecond.zero() /* TODO */),
                        new MotorIoSimulation(DCMotor.getFalcon500(1), 1, MetersPerSecond.zero() /* TODO */))
                        : null;
                intake = Constants.hasIntakeSubsystem()
                        ? new Intake(new MotorIoSimulation(DCMotor.getNeo550(1), 1 /* TODO */,
                                MetersPerSecond.zero() /* TODO */))
                        : null;
                traverser = Constants.hasTraverserSubsystem()
                        ? new Traverser(new MotorIoSimulation(DCMotor.getNeo550(1), 1 /* TODO */,
                                MetersPerSecond.zero() /* TODO */))
                        : null;
                vision = Constants.hasVisionSubsystem()
                        ? new Vision(swerveBase.poseEstimator, new VisionIoSimAndReplay())
                        : null;
                break;
            case LOG_REPLAY:
                swerveBase = new SwerveBase(new GyroIoSimAndReplay(),
                        SwerveModule.createReplaySwerveModule(WheelModuleIndex.FRONT_LEFT),
                        SwerveModule.createReplaySwerveModule(WheelModuleIndex.FRONT_RIGHT),
                        SwerveModule.createReplaySwerveModule(WheelModuleIndex.BACK_LEFT),
                        SwerveModule.createReplaySwerveModule(WheelModuleIndex.BACK_RIGHT));
                aimer = Constants.hasAimerSubsystem()
                        ? new Aimer(new MotorIoReplay(), new MotorIoReplay(), new EncoderIoReplay())
                        : null;
                climber = Constants.hasClimberSubsystem() ? new Climber(new MotorIoReplay(), new MotorIoReplay())
                        : null;
                feeder = Constants.hasFeederSubsystem() ? new Feeder(new MotorIoReplay()) : null;
                flywheel = Constants.hasFlywheelSubsystem() ? new Flywheel(new MotorIoReplay(), new MotorIoReplay())
                        : null;
                intake = Constants.hasIntakeSubsystem() ? new Intake(new MotorIoReplay()) : null;
                traverser = Constants.hasTraverserSubsystem() ? new Traverser(new MotorIoReplay()) : null;
                vision = Constants.hasVisionSubsystem()
                        ? new Vision(swerveBase.poseEstimator, new VisionIoSimAndReplay())
                        : null;
                break;
            default:
                throw new RuntimeException("Unknown Run Mode: " + Constants.getCurrentOperatingMode());
        }

        // #endregion

        // #region: Subsystems without Simulation & Log Replay.
        /*
         * We can safely set LEDs even if there are no LEDs.
         * (LED hardware is built into the RoboRio and therefore always "exists".)
         */
        leds = new Leds();

        // TODO: Find a way to Simulate.
        noteSensor = Constants.hasNoteSensorSubsystem() ? new NoteSensor(Constants.getNoteSensorChannel()) : null;

        // #endregion

    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {

        // #region: =========== Default Commands & Triggers ====================

        // #region: ---------- Configure Default Commands ----------

        leds.setDefaultCommand(leds.setAllianceColorCommand());

        // #endregion

        // #region: ---------- Motor Overheat Triggers ----------
        new Trigger(swerveBase::isTemperatureTooHigh)
                .whileTrue(swerveBase.overheatedMotorShutdownCommand(leds));

        for (MotorSubsystem motorSubsystem : MotorSubsystem.instantiatedSubsystems) {
            new Trigger(motorSubsystem::isTemperatureTooHigh)
                    .whileTrue(CompoundCommands.overheatedMotorShutdownCommand(motorSubsystem, leds));
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
                swerveBase.spinCommand(Rotation2d.fromDegrees(180), 1));

        if (Constants.hasFeederSubsystem() && Constants.hasIntakeSubsystem()) {
            NamedCommands.registerCommand("StartIntake", CompoundCommands.intakeStartStopCommand(feeder, intake));
        }

        if (Constants.hasFlywheelSubsystem()) {
            NamedCommands.registerCommand("Spin Up Flywheel", flywheel.spinUpFlywheelCommand());
        }

        if (Constants.hasAimerSubsystem() && Constants.hasFeederSubsystem() && Constants.hasIntakeSubsystem()
                && Constants.hasNoteSensorSubsystem()) {

            NamedCommands.registerCommand("Auto Shoot",
                    CompoundCommands.autoShootCommand(swerveBase, aimer, feeder, intake, noteSensor));

            NamedCommands.registerCommand("Initial Shoot",
                    CompoundCommands.autoJustShootCommand(aimer, feeder, intake, noteSensor));

            NamedCommands.registerCommand("Delayed Manual Shot",
                    CompoundCommands.autoDelayedManualShotCommand(aimer, feeder, intake, noteSensor));
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

        swerveBase.setDefaultCommand(
                swerveBase.manualDriveDefaultCommand(pilot::getLeftY, pilot::getLeftX, pilot::getRightX));

        if (Constants.hasFlywheelSubsystem()) {
            flywheel.setDefaultCommand(flywheel.defaultFlywheelCommand());
        }

        // #endregion

        // #region: ---------- Configure Command Triggers ----------

        if (Constants.hasNoteSensorSubsystem()) {
            new Trigger(noteSensor::isObjectDetectedOnSwitch).whileTrue(leds.setColorCommand(Color.kGreen));
        }

        // TODO: Add LED Trigger for Ready to Shoot.

        // #endregion

        // #region: ---------- Configure Controller 0 for Pilot ----------

        pilot.leftTrigger().whileTrue(CompoundCommands.autoAimAndManuallyDriveCommand(swerveBase, aimer, flywheel,
                pilot::getLeftY, pilot::getLeftX,
                Constants::getSpeakerLocation));
        pilot.rightTrigger().whileTrue(CompoundCommands.autoAimAndManuallyDriveCommand(swerveBase, aimer, flywheel,
                pilot::getLeftY, pilot::getLeftX,
                Constants::getAmpLocation));
        pilot.leftTrigger().onFalse(aimer.setAngleCommand(Rotation2d.fromDegrees(2)));
        pilot.y().onTrue(swerveBase.resetFieldOrientationCommand());

        // #endregion

        // #region: ---------- Configure Controller 1 for Co-Pilot ----------

        if (Constants.hasFeederSubsystem() && Constants.hasFlywheelSubsystem() && Constants.hasIntakeSubsystem()) {

            if (Constants.hasNoteSensorSubsystem()) {
                coPilot.leftTrigger().and(noteSensor::isObjectNotDetectedSwitch)
                        .whileTrue(CompoundCommands.runIntakeCommand(feeder, flywheel, intake));

            }
            coPilot.x().whileTrue(CompoundCommands.reverseShooterAndIntakeCommand(feeder, flywheel, intake));
        }

        if (Constants.hasFeederSubsystem() && Constants.hasFlywheelSubsystem()) {

            if (Constants.hasIntakeSubsystem() && Constants.hasNoteSensorSubsystem()) {
                coPilot.rightTrigger()
                        .onTrue(CompoundCommands.shootTeleopCommand(feeder, flywheel, intake, noteSensor));
            }
            coPilot.a().whileTrue(CompoundCommands.reverseShooterCommand(feeder, flywheel, leds));
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