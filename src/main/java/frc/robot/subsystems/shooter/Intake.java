package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.subsystems.general.SingleMotorIo;
import frc.robot.subsystems.general.SingleMotorSubsystem;

public class Intake extends SingleMotorSubsystem {
    public Intake(SingleMotorIo io) {
        super("Shooter/Intake", io, Constants.INTAKE_FORWARD_VOLTAGE, Constants.INTAKE_REVERSE_VOLTAGE);
    }
}
