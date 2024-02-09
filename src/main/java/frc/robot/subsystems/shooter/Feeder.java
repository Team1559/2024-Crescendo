package frc.robot.subsystems.shooter;

import frc.robot.Constants;

public class Feeder extends SingleCanSparkMaxSubsystem {
    public Feeder() {
        super("Feeder", Constants.FEEDER_MOTOR_ID, Constants.FEEDER_FORWARD_VOLTAGE, Constants.FEEDER_REVERSE_VOLTAGE);
    }
}
