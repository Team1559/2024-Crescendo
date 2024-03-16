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

public class MotorIoKrakenX60 extends MotorIoTalonFx {

    public MotorIoKrakenX60(int motorId, String canivoreId, boolean inverted, NeutralModeValue idleMode,
            Rotation2d absoluteEncoderOffset, PidValues pidValues) {
        super(motorId, canivoreId, inverted, idleMode, absoluteEncoderOffset, pidValues);
    }

    @Override
    public Measure<Temperature> getMaxSafeTemperature() {
        // TODO: No data was found.
        return Constants.getFalcon500MaxTemperature();
    }

    @Override
    public Measure<Velocity<Angle>> getMaxSafeVelocity() {
        // https://store.ctr-electronics.com/announcing-kraken-x60
        return RevolutionsPerSecond.of(6_000);
    }

    @Override
    public Measure<Voltage> getMaxSafeVoltage() {
        // https://store.ctr-electronics.com/announcing-kraken-x60
        return Volts.of(24);
    }
}
