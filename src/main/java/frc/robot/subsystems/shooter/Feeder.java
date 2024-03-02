package frc.robot.subsystems.shooter;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

import frc.robot.io.motor.MotorIo;
import frc.robot.subsystems.single_motor.SingleMotorSubsystem;

public class Feeder extends SingleMotorSubsystem {
    public Feeder(MotorIo io) {
        super("Shooter/Feeder", io, CONSTANTS.getFeederVelocityForward(), CONSTANTS.getFeederVelocityReverse());
    }
}
