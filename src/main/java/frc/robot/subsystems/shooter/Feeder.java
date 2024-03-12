package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.io.motor.MotorIo;
import frc.robot.subsystems.AbstractSingleMotorSubsystem;

public class Feeder extends AbstractSingleMotorSubsystem {
    public Feeder(MotorIo io) {
        super(io, Constants.getFeederVelocityForward(), Constants.getFeederVelocityReverse());
    }
}
