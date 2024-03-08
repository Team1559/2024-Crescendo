package frc.robot.io.motor;

import static edu.wpi.first.units.Units.Celsius;

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
    public Rotation2d getAbsolutePosition() {
        return position;
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
