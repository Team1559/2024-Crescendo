package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.abstract_interface.MotorSubsystem;
import frc.robot.subsystems.led.Leds;
import frc.robot.subsystems.shooter.Aimer;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Intake;
import frc.robot.subsystems.shooter.NoteSensor;
import frc.robot.util.CommandUtils;

/**
 * Only used for commands that use multiple subsystems.
 * <p>
 * Single subsystem commands should be in their subsystem class.
 * </p>
 */
public class ShootCommands {

    /** Makes Class non-instantiable */
    private ShootCommands() {
    }

    // ========================= Default Commands =========================

    public static Command defaultFlywheelCommand(Flywheel flywheel) {
        Command command = new SequentialCommandGroup(new WaitCommand(.25), flywheel.stopCommand());

        return CommandUtils.addName(command);
    }

    // ========================= Trigger Commands ==============================

    public static Command overheatedMotorShutdownCommand(MotorSubsystem motorSubsystem, Leds leds) {
        Command command = motorSubsystem.stopCommand()
                .alongWith(leds.setDynamicPatternCommand(Constants.getMotorOverheatEmergencyPattern(), false));

        return CommandUtils.addName(command);
    }

    // ========================= Other Commands =========================

    public static Command autoJustShootCommand(Aimer aimer, Feeder feeder, NoteSensor noteSensor, Leds leds) {
        Command command = aimer.setAngleCommand(Rotation2d.fromDegrees(36.7))
                .andThen(new WaitUntilCommand(() -> aimer.isAtTarget()))
                .andThen(ShootCommands.shootAutonomousCommand(feeder, noteSensor, leds));

        return CommandUtils.addName(command);
    }

    public static Command intakeStartStopCommand(Feeder feeder, Intake intake) {
        Command command = new StartEndCommand(
                () -> {
                    intake.forward();
                    feeder.forward();
                },
                () -> {
                    intake.stop();
                    feeder.stop();
                },
                intake, feeder);

        return CommandUtils.addName(command);
    }

    public static Command reverseShooterAndIntakeCommand(Feeder feeder, Flywheel flywheel, Intake intake) {
        Command command = new ParallelCommandGroup(new StartEndCommand(flywheel::reverse, flywheel::stop, flywheel),
                new StartEndCommand(feeder::reverse, feeder::stop, feeder),
                new StartEndCommand(intake::reverse, intake::stop, intake));

        return CommandUtils.addName(command);
    }

    public static Command reverseShooterCommand(Feeder feeder, Flywheel flywheel, Leds leds) {
        Command reverseShooterCommand = new Command() {
            @Override
            public void execute() {
                flywheel.reverse();
                feeder.reverse();
                leds.setDynamicPattern(new Color[] { Color.kRed, Color.kRed, Color.kBlack, Color.kBlack }, true);
            }

            @Override
            public void end(boolean interrupted) {
                flywheel.stop();
                feeder.stop();
                leds.setAllianceColor();
            }
        };
        reverseShooterCommand.addRequirements(flywheel, feeder, leds);

        return CommandUtils.addName(reverseShooterCommand);
    }

    public static Command runIntakeCommand(Feeder feeder, Flywheel flywheel, Intake intake) {

        Command command = new ParallelCommandGroup(ShootCommands.intakeStartStopCommand(feeder, intake),
                flywheel.stopCommand());

        return CommandUtils.addName(command);
    }

    public static Command shootAutonomousCommand(Feeder feeder, NoteSensor noteSensor, Leds leds) {
        Command command = new SequentialCommandGroup(
                feeder.forwardCommand(),
                LedCommands.blinkCommand(leds, Color.kOrange),
                noteSensor.waitForNoObjectOnSwitchCommand(),
                new WaitCommand(.25),
                feeder.stopCommand());

        return CommandUtils.addName(command);
    }

    public static Command shootTeleopCommand(Feeder feeder, Flywheel flywheel, Intake intake, NoteSensor noteSensor,
            Leds leds) {

        ParallelRaceGroup group = new ParallelRaceGroup(new StartEndCommand(intake::forward, intake::stop, intake),
                feeder.forwardMaxVelocityThenStopCommand(),
                leds.setColorCommand(Color.kPurple).repeatedly(),
                noteSensor.waitForNoObjectOnSwitchCommand(), new WaitCommand(5));

        // TODO: Spin up flywheel if not already spinning.

        return CommandUtils.addName(group);
    }

    public static Command spinUpFlywheelCommand(Flywheel flywheel) {
        Command command = new SequentialCommandGroup(
                flywheel.forwardCommand(),
                new WaitCommand(1) // TODO: Tune.
        );

        return CommandUtils.addName(command);
    }
}
