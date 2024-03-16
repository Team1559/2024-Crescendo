package org.victorrobotics.frc.subsystems.shooter;

import org.victorrobotics.frc.Constants;
import org.victorrobotics.frc.io.motor.MotorIo;
import org.victorrobotics.frc.subsystems.abstract_interface.SingleMotorSubsystem;

public class Feeder extends SingleMotorSubsystem {
    public Feeder(MotorIo io) {
        super(io, Constants.getFeederVelocityForward(), Constants.getFeederVelocityReverse());
    }
}
