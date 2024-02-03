package frc.robot.subsystems.shooter;

import frc.robot.Constants;

public class Feeder extends DualCanSparkMaxSubsystem {
    public Feeder() {
        super("Feeder", Constants.FEEDER_L_ID, Constants.FEEDER_R_ID,
                Constants.FEEDER_FORWARD_VOLTAGE, Constants.FEEDER_REVERSE_VOLTAGE);
    }
}
