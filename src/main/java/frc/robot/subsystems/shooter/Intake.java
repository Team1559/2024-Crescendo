package frc.robot.subsystems.shooter;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

import frc.robot.io.motor.MotorIo;
import frc.robot.subsystems.AbstractSingleMotorSubsystem;

public class Intake extends AbstractSingleMotorSubsystem {
    public Intake(MotorIo io) {
        super("Shooter/Intake", io, CONSTANTS.getIntakeVelocityForward(), CONSTANTS.getIntakeVelocityReverse());
    }
}
