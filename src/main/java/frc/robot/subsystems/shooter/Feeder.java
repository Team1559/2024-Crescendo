package frc.robot.subsystems.shooter;

public class Feeder extends DualCanSparkMaxSubsystem {
    public Feeder() {
        DualCanSparkMaxSubsystem("Feeder", Constants.FEEDER_L_ID, Constants.FEEDER_R_ID,
                Constants.FEEDER_FORWARD_VOLTAGE, Constants.FEEDER_REVERSE_VOLTAGE);
    }
}
