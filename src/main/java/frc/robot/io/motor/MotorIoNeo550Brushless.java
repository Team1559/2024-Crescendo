package frc.robot.io.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.Constants;
import frc.robot.Constants.PID;

public class MotorIoNeo550Brushless extends MotorIoSparkMax {

    public static final Measure<Velocity<Angle>> MAX_VELOCITY = RevolutionsPerSecond.of(11000);

    /**
     * Create a new subsystem for a single SparkMax-controlled motor in voltage mode
     * 
     * @param motorId  Motor CAN ID
     * @param inverted True if the motor direction should be inverted
     */
    public MotorIoNeo550Brushless(int motorId, boolean inverted, IdleMode idleMode, Rotation2d absoluteEncoderOffset,
            PID pidValues) {
        super(motorId, inverted, idleMode, absoluteEncoderOffset, pidValues);

        motor.setSmartCurrentLimit((int) Constants.getNeo550BrushlessCurrentLimit().in(Amps));
        motor.setSecondaryCurrentLimit(Constants.getNeo550BrushlessCurrentSecondaryLimit().in(Amps));
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
