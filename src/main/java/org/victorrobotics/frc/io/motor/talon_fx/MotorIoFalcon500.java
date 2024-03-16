package org.victorrobotics.frc.io.motor.talon_fx;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.victorrobotics.frc.Constants;
import org.victorrobotics.frc.Constants.PidValues;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public class MotorIoFalcon500 extends MotorIoTalonFx {

    public MotorIoFalcon500(int motorId, String canivoreId, boolean inverted, NeutralModeValue idleMode,
            Rotation2d absoluteEncoderOffset, PidValues pidValues) {
        super(motorId, canivoreId, inverted, idleMode, absoluteEncoderOffset, pidValues, null);
    }

    public MotorIoFalcon500(int motorId, String canivoreId, boolean inverted, NeutralModeValue idleMode,
            Rotation2d absoluteEncoderOffset, PidValues pidValues, Measure<Current> currentLimit) {
        super(motorId, canivoreId, inverted, idleMode, absoluteEncoderOffset, pidValues, currentLimit);
    }

    @Override
    public Measure<Current> getMaxSafeCurrent() {
        // https://store.ctr-electronics.com/announcing-kraken-x60
        // Stall Current is 257.
        return Amps.of(256);
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
