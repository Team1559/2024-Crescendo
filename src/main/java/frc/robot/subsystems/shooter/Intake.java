package frc.robot.subsystems.shooter;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

import frc.robot.io.motor.MotorIo;
import frc.robot.subsystems.single_motor.SingleMotorSubsystem;

public class Intake extends SingleMotorSubsystem {
    public Intake(MotorIo io) {
        super("Shooter/Intake", io, CONSTANTS.getIntakeVelocityForward(), CONSTANTS.getIntakeVelocityReverse());
    }
}
