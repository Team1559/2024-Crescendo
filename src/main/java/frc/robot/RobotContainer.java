package frc.robot;

import static frc.robot.util.SupplierUtil.not;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.LedCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.constants.NewConstants;
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
import frc.robot.subsystems.sinigle_motor.SingleMotorIoReplay;
import frc.robot.subsystems.sinigle_motor.SingleMotorIoSparkMax;
import frc.robot.subsystems.swerve_module.SwerveModuleIoReplay;
import frc.robot.subsystems.swerve_module.SwerveModuleIoSim;
import frc.robot.subsystems.swerve_module.SwerveModuleIoTalonFx;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIoLimelight;
import frc.robot.subsystems.vision.VisionIoSimAndReplay;
import frc.robot.util.KColor;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final CommandXboxController controller1 = new CommandXboxController(0);
    private final CommandXboxController controller2 = new CommandXboxController(1);
    private final LoggedDashboardChooser<Command> autoChooser;

    private final DriveBase driveBase;

    private final Aimer aimer;
    private final ColorSensor colorSensor;
    private final Feeder feeder;
    private final Flywheel flywheel;
    private final Intake intake;
    private final Leds leds;
    private final Vision vision;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // ----- Initialize Subsystems with Simulation and/or Log Replay Modes -----
        switch (NewConstants.get().getCurrentOperatingMode()) {

            case REAL_WORLD:
                // Real robot, instantiate hardware IO implementations
                driveBase = new DriveBase(
                        new GyroIoPigeon2(),
                        new SwerveModuleIoTalonFx(WheelModuleIndex.FRONT_LEFT),
                        new SwerveModuleIoTalonFx(WheelModuleIndex.FRONT_RIGHT),
                        new SwerveModuleIoTalonFx(WheelModuleIndex.BACK_LEFT),
                        new SwerveModuleIoTalonFx(WheelModuleIndex.BACK_RIGHT));
                feeder = Constants.HAVE_FEEDER
                        ? new Feeder(new SingleMotorIoSparkMax(Constants.FEEDER_MOTOR_ID, Constants.IS_FEEDER_INVERTED))
                        : null;
                intake = Constants.HAVE_INTAKE
                        ? new Intake(new SingleMotorIoSparkMax(Constants.INTAKE_MOTOR_ID, Constants.IS_INTAKE_INVERTED))
                        : null;
                vision = Constants.HAVE_VISION
                        ? new Vision(driveBase.poseEstimator, new VisionIoLimelight(Constants.SHOOTER_CAMERA_NAME))
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
                feeder = Constants.HAVE_FEEDER
                        ? new Feeder(new SingleMotorIoSparkMax(Constants.FEEDER_MOTOR_ID, Constants.IS_FEEDER_INVERTED))
                        : null;
                intake = Constants.HAVE_INTAKE
                        ? new Intake(new SingleMotorIoSparkMax(Constants.INTAKE_MOTOR_ID, Constants.IS_INTAKE_INVERTED))
                        : null;
                vision = Constants.HAVE_VISION ? new Vision(driveBase.poseEstimator, new VisionIoSimAndReplay())
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
                feeder = Constants.HAVE_FEEDER ? new Feeder(new SingleMotorIoReplay()) : null;
                intake = Constants.HAVE_INTAKE ? new Intake(new SingleMotorIoReplay()) : null;
                vision = Constants.HAVE_VISION ? new Vision(driveBase.poseEstimator, new VisionIoSimAndReplay())
                        : null;
                break;

            default:
                throw new RuntimeException(
                        "Unknown Run Mode: " + NewConstants.get().getCurrentOperatingMode());
        }

        // ----- Initialize Subsystems without Simulation and/or Log Replay Modes -----
        aimer = Constants.HAVE_AIMER ? new Aimer() : null;
        colorSensor = Constants.HAVE_COLOR_SENSOR ? new ColorSensor() : null;
        flywheel = Constants.HAVE_FLYWHEEL ? new Flywheel() : null;
        // We can safely emit LED instructions even if there are no LEDs.
        // (The LED control hardware is built into the RoboRio so always "exists".)
        leds = new Leds();

        // ========================= Auto & Tele-Op ============================
        // ---------- Configure Default Commands ----------
        driveBase.setDefaultCommand(DriveCommands.manualDriveDefaultCommand(driveBase,
                () -> -controller1.getLeftY(),
                () -> -controller1.getLeftX(),
                () -> controller1.getLeftTriggerAxis() > controller1.getRightTriggerAxis()
                        ? controller1.getLeftTriggerAxis()
                        : -controller1.getRightTriggerAxis()));
        if (Constants.HAVE_INTAKE && Constants.HAVE_COLOR_SENSOR) {
            intake.setDefaultCommand(ShooterCommands.defaultIntakeCommand(intake, colorSensor));
        }
        if (Constants.HAVE_FEEDER && Constants.HAVE_COLOR_SENSOR) {
            intake.setDefaultCommand(ShooterCommands.defaultFeederCommand(feeder, colorSensor));
        }
        leds.setDefaultCommand(LedCommands.defaultLedCommand(leds));
        if (Constants.HAVE_FLYWHEEL) {
            flywheel.setDefaultCommand(ShooterCommands.defaultFlywheelCommand(flywheel)); // @formatter:off I finished this code before Xander and Brenan finished talking about if this is the right way to do it. @formatter:on
        }
        // ---------- Configure Command Triggers ----------
        if (Constants.HAVE_COLOR_SENSOR) {
            new Trigger((colorSensor::isObjectDetected)).whileTrue(leds.setColorCommand(Color.kDarkOrange));
        }

        // ========================= Autonomous ================================
        // ---------- Create Named Commands for use by Path Planner ----------
        NamedCommands.registerCommand("Spin 180", DriveCommands.spinCommand(driveBase, Rotation2d.fromDegrees(180), 1));
        NamedCommands.registerCommand("StartIntake", LedCommands.blinkCommand(leds, Color.kPurple));
        if (Constants.HAVE_FLYWHEEL) {
            NamedCommands.registerCommand("Spin Up Flywheel", ShooterCommands.spinUpFlywheelCommand(flywheel));
        }

        Command aimCommand = new ConditionalCommand(
                // Turn to Blue Speaker.
                DriveCommands.turnToTargetCommand(driveBase, Constants.BLUE_SPEAKER_LOCATION, 4.5),
                // Turn to Red Speaker.
                DriveCommands.turnToTargetCommand(driveBase, Constants.RED_SPEAKER_LOCATION, 4.5),
                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
        Command autoShootCommand;
        if (Constants.HAVE_SHOOTER) {
            autoShootCommand = ShooterCommands.shootAutonomousCommand(feeder, leds, colorSensor);
        } else {
            autoShootCommand = LedCommands.blinkCommand(leds, Color.kOrange);
        }
        NamedCommands.registerCommand("Auto Shoot", new SequentialCommandGroup(aimCommand, autoShootCommand));

        // ---------- Set-up Autonomous Choices ----------
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // ========================= Tele-Op ===================================
        // ---------- Configure Buttons for SubSystem Actions (Controller 1) ----------
        Command teleOpShootCommand;
        Command reverseShooterCommand;
        Command stopIntakeFeederCommand;
        if (Constants.HAVE_SHOOTER) {
            teleOpShootCommand = ShooterCommands.shootTeleopCommand(flywheel, feeder, leds, colorSensor);
            reverseShooterCommand = ShooterCommands.reverseShooterCommand(flywheel, feeder, intake, leds);
            stopIntakeFeederCommand = ShooterCommands.stopIntakeFeederCommand(intake, feeder, leds);
        } else {
            teleOpShootCommand = LedCommands.blinkCommand(leds, Color.kOrange);
            reverseShooterCommand = LedCommands.blinkCommand(leds, Color.kTomato);
            stopIntakeFeederCommand = LedCommands.blinkCommand(leds, Color.kDarkKhaki);
        }
        controller1.y().and(controller1.b()).onTrue(teleOpShootCommand);

        controller1.b().and(not(controller1.rightBumper()))
                .whileTrue(DriveCommands.autoAimAndManuallyDriveCommand(driveBase, flywheel,
                        () -> -controller1.getLeftY(), () -> -controller1.getLeftX(),
                        Constants.SPEAKER_LOCATION_SUPPLIER));
        controller1.b().and(controller1.rightBumper())
                .whileTrue(DriveCommands.autoAimAndManuallyDriveCommand(driveBase, flywheel,
                        () -> -controller1.getLeftY(), () -> -controller1.getLeftX(),
                        Constants.AMP_LOCATION_SUPPLIER));

        // ---------- Configure Light Buttons (Controller 1) ----------
        controller1.start().and(controller1.a()).onTrue(leds.setColorCommand(Color.kDarkGreen));
        controller1.start().and(controller1.b()).onTrue(leds.setStaticPatternCommand(
                new Color[] { KColor.ALLIANCE_RED, KColor.ALLIANCE_RED, Color.kBlack, Color.kBlack }));
        controller1.start().and(controller1.x()).onTrue(leds.setDynamicPatternCommand(new Color[] {
                KColor.ALLIANCE_BLUE, KColor.ALLIANCE_BLUE, KColor.ALLIANCE_BLUE,
                Color.kDarkViolet, Color.kDarkViolet, Color.kDarkViolet }, true));
        controller1.start().and(controller1.y()).onTrue(leds.setDynamicPatternCommand(new Color[] {
                Color.kYellow, Color.kYellow, Color.kYellow, Color.kBlack, Color.kBlack, Color.kBlack,
                Color.kOrange, Color.kOrange, Color.kOrange, Color.kBlack, Color.kBlack, Color.kBlack },
                false));
        controller1.leftBumper().onTrue(leds.changeBrightnessCommand(true));
        controller1.rightBumper().onTrue(leds.changeBrightnessCommand(false));
        controller1.leftBumper().and(controller1.rightBumper()).onTrue(leds.setColorCommand(Color.kBlack));

        // ---------- Configure D-PAD for Tele-Op (Controller 1) ----------
        controller2.povUp().whileTrue(driveBase.runVelocityCommand(new ChassisSpeeds(1, 0, 0)));
        controller2.povDown().whileTrue(driveBase.runVelocityCommand(new ChassisSpeeds(-1, 0, 0)));
        controller2.povRight().whileTrue(driveBase.runVelocityCommand(new ChassisSpeeds(0, -1, 0)));
        controller2.povLeft().whileTrue(driveBase.runVelocityCommand(new ChassisSpeeds(0, 1, 0)));

        // ---------- Configure Subsystem Debug Buttons (Controller 2) ----------
        if (Constants.HAVE_INTAKE) {
            controller2.a().whileTrue(new StartEndCommand(intake::start, intake::stop, intake));
        }
        if (Constants.HAVE_FLYWHEEL) {
            controller2.b().whileTrue(new StartEndCommand(flywheel::start, flywheel::stop, flywheel));
        }
        if (Constants.HAVE_FEEDER) {
            controller2.y().whileTrue(new StartEndCommand(feeder::start, feeder::stop, feeder));
        }
        if (Constants.HAVE_AIMER) {
            controller2.rightTrigger().onTrue(new InstantCommand(() -> aimer.setTargetAngle(aimer.getAngle() + 5)));
            controller2.leftTrigger().onTrue(new InstantCommand(() -> aimer.setTargetAngle(aimer.getAngle() - 5)));
        }
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