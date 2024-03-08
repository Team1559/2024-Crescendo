package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.io.motor.MotorIo;
import frc.robot.subsystems.abstract_interface.SingleMotorSubsystem;

public class Intake extends SingleMotorSubsystem {
    public Intake(MotorIo io) {
        super(io, Constants.getIntakeVelocityForward(), Constants.getIntakeVelocityReverse());
    }
}
