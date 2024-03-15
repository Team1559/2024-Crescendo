package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import frc.robot.Constants;
import frc.robot.io.motor.MotorIo;
import frc.robot.subsystems.abstract_interface.DualMotorSubsystem;

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
}
