package frc.robot.io.motor;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public class MotorIoReplay implements MotorIo {

    Rotation2d position = Rotation2d.fromRotations(0);
    Measure<Velocity<Angle>> velocity;
    Measure<Voltage> voltage;

    @Override
    public void updateInputs(MotorIoInputs inputs) {
        // No functionality.
    }

    @Override
    public Measure<Temperature> getMaxSafeTemperature() {
        return Celsius.of(Double.MAX_VALUE);
    }

    @Override
    public Measure<Velocity<Angle>> getMaxSafeVelocity() {
        return RevolutionsPerSecond.of(Double.POSITIVE_INFINITY);
    }

    @Override
    public Measure<Voltage> getMaxSafeVoltage() {
        return Volts.of(Double.POSITIVE_INFINITY);
    }

    @Override
    public Rotation2d getPositionAbsolute() {
        return position;
    }

    @Override
    public Rotation2d getPositionRelative() {
        return position.plus(Rotation2d.fromDegrees(0));
    }

    @Override
    public Measure<Temperature> getTemperature() {
        return Celsius.zero();
    }

    @Override
    public Measure<Velocity<Angle>> getVelocity() {
        return velocity;
    }

    @Override
    public Measure<Voltage> getVoltage() {
        return voltage;
    }

    @Override
    public void setPosition(Rotation2d position) {
        this.position = position;
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        this.velocity = velocity;
    }

    @Override
    public void setVoltage(Measure<Voltage> voltage) {
        this.voltage = voltage;
    }

    @Override
    public void stop() {
        velocity = null;
        voltage = null;
    }
}
