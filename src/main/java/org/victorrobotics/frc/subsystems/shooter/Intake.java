package org.victorrobotics.frc.subsystems.shooter;

import org.victorrobotics.frc.Constants;
import org.victorrobotics.frc.io.motor.MotorIo;
import org.victorrobotics.frc.subsystems.abstract_interface.SingleMotorSubsystem;

public class Intake extends SingleMotorSubsystem {
    public Intake(MotorIo io) {
        super(io, Constants.getIntakeVelocityForward(), Constants.getIntakeVelocityReverse());
    }
}
