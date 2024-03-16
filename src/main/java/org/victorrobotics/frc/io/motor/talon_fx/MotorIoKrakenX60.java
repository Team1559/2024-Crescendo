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

public class MotorIoKrakenX60 extends MotorIoTalonFx {

    public MotorIoKrakenX60(int motorId, String canivoreId, boolean inverted, NeutralModeValue idleMode,
            Rotation2d absoluteEncoderOffset, PidValues pidValues) {
        this(motorId, canivoreId, inverted, idleMode, absoluteEncoderOffset, pidValues, null);
    }

    public MotorIoKrakenX60(int motorId, String canivoreId, boolean inverted, NeutralModeValue idleMode,
            Rotation2d absoluteEncoderOffset, PidValues pidValues, Measure<Current> currentLimit) {
        super(motorId, canivoreId, inverted, idleMode, absoluteEncoderOffset, pidValues, currentLimit);
    }

    @Override
    public Measure<Current> getMaxSafeCurrent() {
        // https://store.ctr-electronics.com/announcing-kraken-x60
        return Amps.of(40);
    }

    @Override
    public Measure<Temperature> getMaxSafeTemperature() {
        // TODO: No data was found.
        return Constants.getFalcon500MaxTemperature();
    }

    @Override
    public Measure<Velocity<Angle>> getMaxSafeVelocity() {
        // https://store.ctr-electronics.com/announcing-kraken-x60
        // Max Velocity not defined. This is free RPM.
        return RevolutionsPerSecond.of(6_000);
    }

    @Override
    public Measure<Voltage> getMaxSafeVoltage() {
        // https://store.ctr-electronics.com/announcing-kraken-x60
        return Volts.of(24);
    }
}
