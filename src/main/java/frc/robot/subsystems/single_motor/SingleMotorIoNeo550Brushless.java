package frc.robot.subsystems.single_motor;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Units;

public class SingleMotorIoNeo550Brushless extends SingleMotorIoSparkMax {

    /**
     * Create a new subsystem for a single SparkMax-controlled motor in voltage mode
     * 
     * @param motorId  Motor CAN ID
     * @param inverted True if the motor direction should be inverted
     */
    public SingleMotorIoNeo550Brushless(int motorId, boolean inverted) {
        super(motorId, inverted);
    }

    public Measure<Temperature> getMaxSafeTemperature() {
        // https://www.revrobotics.com/neo-550-brushless-motor-locked-rotor-testing
        return Units.Celsius.of(40);
    }
}
