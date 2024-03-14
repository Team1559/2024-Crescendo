package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.io.motor.MotorIo;
import frc.robot.subsystems.abstract_interface.SingleMotorSubsystem;

public class Feeder extends SingleMotorSubsystem {
    public Feeder(MotorIo io) {
        super(io, Constants.getFeederVelocityForward(), Constants.getFeederVelocityReverse());
    }
}
