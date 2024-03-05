package frc.robot.subsystems.shooter;

import frc.robot.io.motor.MotorIo;
import frc.robot.subsystems.AbstractSingleMotorSubsystem;

import frc.robot.Constants;

public class Intake extends AbstractSingleMotorSubsystem {
    public Intake(MotorIo io) {
        super("Shooter/Intake", io, Constants.getIntakeVelocityForward(), Constants.getIntakeVelocityReverse());
    }
}
