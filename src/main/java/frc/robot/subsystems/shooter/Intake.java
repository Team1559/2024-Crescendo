package frc.robot.subsystems.shooter;

import frc.robot.Constants;

public class Intake extends SingleCanSparkMaxSubsystem {
    public Intake() {
        super("Intake", Constants.INTAKE_MOTOR_ID, Constants.INTAKE_FORWARD_VOLTAGE, Constants.INTAKE_REVERSE_VOLTAGE,
                Constants.IS_INTAKE_INVERTED);
    }
}
