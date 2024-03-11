package frc.robot.io.motor.talon_fx;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.PidValues;

public class MotorIoFalcon500 extends MotorIoTalonFx {

    public MotorIoFalcon500(int motorId, boolean inverted, NeutralModeValue idleMode, Rotation2d absoluteEncoderOffset,
            PidValues pidValues) {
        super(motorId, inverted, idleMode, absoluteEncoderOffset, pidValues);
    }

    @Override
    public Measure<Temperature> getMaxSafeTemperature() {
        // https://www.chiefdelphi.com/uploads/short-url/eVYO5tVOYZecwq6Tl2kURlFZFgq.pdf
        return Constants.getFalcon500MaxTemperature();
    }

    @Override
    public Measure<Velocity<Angle>> getMaxSafeVelocity() {
        // https://www.vexrobotics.com/217-6515.html#attr-vex_other_info
        // Could not find any max speed, but. But Free Speed is 6,380 +/- 10%.
        return RevolutionsPerSecond.of(6_380 * 1.1);
    }

    @Override
    public Measure<Voltage> getMaxSafeVoltage() {
        // https://www.vexrobotics.com/217-6515.html#attr-vex_other_info
        return Volts.of(16);
    }
}
