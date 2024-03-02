package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.io.motor.MotorIoNeo550Brushless;
import frc.robot.subsystems.led.Leds;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Intake;
import frc.robot.subsystems.shooter.NoteSensor;

public class ShooterCommands {

    /** Makes Class non-instantiable */
    private ShooterCommands() {
    }

    // ========================= Default Commands =========================
    public static Command defaultIntakeCommand(Intake intake, NoteSensor sensor) {
        return Commands.run(() -> {
            if (sensor.isObjectDetected()) {
                intake.stop();
            } else {
                intake.start();
            }
        }, intake);
    }

    public static Command defaultFeederCommand(Feeder feeder, NoteSensor sensor) {
        return Commands.run(() -> {
            if (sensor.isObjectDetected()) {
                feeder.stop();
            } else {
                feeder.start();
            }
        }, feeder);
    }

    public static Command defaultFlywheelCommand(Flywheel flywheel) {
        return new SequentialCommandGroup(new WaitCommand(.25), flywheel.stopCommand());
    }

    // ========================= Other Commands =========================

    public static Command intakeStartStopCommand(Intake intake, Feeder feeder) {
        return new StartEndCommand(
                () -> {
                    intake.start();
                    feeder.start();
                },
                () -> {
                    intake.stop();
                    feeder.stop();
                },
                intake, feeder);
    }

    public static Command reverseShooterAndIntakeCommand(Intake intake, Feeder feeder, Flywheel flywheel) {
        return new ParallelCommandGroup(new StartEndCommand(flywheel::reverse, flywheel::stop, flywheel),
                new StartEndCommand(feeder::reverse, feeder::stop, feeder),
                new StartEndCommand(intake::reverse, intake::stop, intake));
    }

    public static Command reverseShooterCommand(Flywheel flywheel, Feeder feeder, Leds leds) {
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
        return reverseShooterCommand;
    }

    public static Command shootAutonomousCommand(Feeder feeder, Leds leds, NoteSensor noteSensor) {
        return new SequentialCommandGroup(
                feeder.startCommand(),
                LedCommands.blinkCommand(leds, Color.kOrange),
                noteSensor.waitForNoObjectCommandSwitch(),
                new WaitCommand(.25),
                feeder.stopCommand());
    }

    public static Command shootTeleopCommand(Feeder feeder, Flywheel flywheel, Intake intake, NoteSensor noteSensor,
            Leds leds) {

        ParallelRaceGroup group = new ParallelRaceGroup(new StartEndCommand(intake::start, intake::stop, intake),
                new StartEndCommand(() -> feeder.setVelocity(MotorIoNeo550Brushless.MAX_VELOCITY), feeder::stop,
                        feeder),
                leds.setColorCommand(Color.kPurple).repeatedly(),
                noteSensor.waitForNoObjectCommandSwitch(), new WaitCommand(5));

        // TODO: Spin up flywheelsm if not already spinning.
        return group;
    }

    public static Command spinUpFlywheelCommand(Flywheel flywheel) {
        return new SequentialCommandGroup(
                flywheel.startCommand(),
                new WaitCommand(1) // TODO: Tune.
        );
    }

    public static Command stopIntakeFeederCommand(Intake intake, Feeder feeder, Leds leds) {
        return new InstantCommand(() -> {
            intake.stop();
            feeder.stop();
            leds.setDynamicPattern(new Color[] { Color.kRed, Color.kRed, Color.kBlack, Color.kBlack }, true);
        }, intake, feeder);
    }
}
