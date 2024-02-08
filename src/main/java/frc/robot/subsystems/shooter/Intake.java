package frc.robot.subsystems.shooter;

import frc.robot.Constants;

public class Intake extends DualCanSparkMaxSubsystem {
    public Intake() {
        super("Intake", Constants.INTAKE_L_ID, Constants.INTAKE_R_ID,
                Constants.INTAKE_FORWARD_VOLTAGE, Constants.INTAKE_REVERSE_VOLTAGE);

    }
}
