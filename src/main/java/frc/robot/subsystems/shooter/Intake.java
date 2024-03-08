package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.io.motor.MotorIo;
import frc.robot.subsystems.AbstractSingleMotorSubsystem;

public class Intake extends AbstractSingleMotorSubsystem {
    public Intake(MotorIo io) {
        super(io, Constants.getIntakeVelocityForward(), Constants.getIntakeVelocityReverse());
    }
}
