package frc.robot.io.motor.can_spark_max;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.PidValues;

public class MotorIoNeo550Brushless extends MotorIoCanSparkMax {

    public MotorIoNeo550Brushless(int motorId, boolean inverted, IdleMode idleMode, Rotation2d absoluteEncoderOffset,
            PidValues pidValues) {
        super(motorId, inverted, idleMode, absoluteEncoderOffset, pidValues);

        motor.setSmartCurrentLimit((int) Constants.getNeo550BrushlessCurrentLimit().in(Amps));
        motor.setSecondaryCurrentLimit(Constants.getNeo550BrushlessCurrentSecondaryLimit().in(Amps));
    }

    @Override
    public Measure<Temperature> getMaxSafeTemperature() {
        // https://www.revrobotics.com/neo-550-brushless-motor-locked-rotor-testing
        return Celsius.of(40);
    }

    @Override
    public Measure<Velocity<Angle>> getMaxSafeVelocity() {
        // https://www.revrobotics.com/rev-21-1651
        return RevolutionsPerSecond.of(11_000);
    }

    @Override
    public Measure<Voltage> getMaxSafeVoltage() {
        // https://www.revrobotics.com/rev-21-1651
        // Could not find any max voltage. But Peak Output Power is 279 watts, and Free
        // Running Current is 1.4 amps.
        return Volts.of(279 / 1.4);
    }
}
