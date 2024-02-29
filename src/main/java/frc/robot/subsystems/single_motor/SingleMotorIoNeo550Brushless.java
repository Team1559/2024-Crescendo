package frc.robot.subsystems.single_motor;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.constants.AbstractConstants.PID;

public class SingleMotorIoNeo550Brushless extends SingleMotorIoSparkMax {

    public static final Measure<Velocity<Angle>> MAX_VELOCITY = RevolutionsPerSecond.of(11000);

    /**
     * Create a new subsystem for a single SparkMax-controlled motor in voltage mode
     * 
     * @param motorId  Motor CAN ID
     * @param inverted True if the motor direction should be inverted
     */
    public SingleMotorIoNeo550Brushless(int motorId, boolean inverted, PID pidValues) {
        super(motorId, inverted, pidValues);
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        super.setVelocity(RevolutionsPerSecond.of(MathUtil.clamp(velocity.in(RevolutionsPerSecond),
                MAX_VELOCITY.negate().in(RevolutionsPerSecond), MAX_VELOCITY.in(RevolutionsPerSecond))));
    }

    public Measure<Temperature> getMaxSafeTemperature() {
        // https://www.revrobotics.com/neo-550-brushless-motor-locked-rotor-testing
        return Units.Celsius.of(40);
    }
}
