package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.io.motor.MotorIo;
import frc.robot.subsystems.abstract_interface.DualMotorSubsystem;
import frc.robot.util.CommandUtils;

public class Flywheel extends DualMotorSubsystem {

    // ========================= Constructors ==================================

    public Flywheel(MotorIo leftMotorIo, MotorIo rightMotorIo) {
        super(leftMotorIo, rightMotorIo,
                Constants.getFlywheelForwardVoltage(), Constants.getFlywheelReverseVoltage());
    }

    // ========================= Functions =====================================

    // -------------------- Default Actions --------------------

    @Override
    public void forward() {
        setVoltage(Constants.getFlywheelForwardVoltage());
    }

    @Override
    public void forwardMaxVelocity() {
        leftMotor.setVelocity(leftMotor.motorIo.getMaxSafeVelocity().times(Constants.flywheelSpinOffset()));
        rightMotor.forwardMaxVelocity();
    }

    // -------------------- Setters --------------------

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        if (velocity.magnitude() > 0) { // Only offset when shooting.
            leftMotor.setVelocity(velocity.times(Constants.flywheelSpinOffset()));
            rightMotor.setVelocity(velocity);
        } else {
            super.setVelocity(velocity);
        }
    }

    @Override
    public void setVoltage(Measure<Voltage> voltage) {
        if (voltage.magnitude() > 0) { // Only offset when shooting.
            leftMotor.setVoltage(voltage.times(Constants.flywheelSpinOffset()));
            rightMotor.setVoltage(voltage);
        } else {
            super.setVoltage(voltage);
        }
    }

    // ========================= Default Commands =========================

    public Command defaultFlywheelCommand() {
        Command command = new SequentialCommandGroup(new WaitCommand(.5), this.stopCommand());
        return CommandUtils.addName(command);
    }

    // ========================= Other Commands =========================

    public Command spinUpFlywheelCommand() {

        Command command = new SequentialCommandGroup(
                forwardCommand(),
                new WaitCommand(1) // TODO: Tune.
        );

        return CommandUtils.addName(command);
    }

}
