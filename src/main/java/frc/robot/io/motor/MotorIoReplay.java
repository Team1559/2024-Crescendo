package frc.robot.io.motor;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public class MotorIoReplay implements MotorIo {

    Measure<Velocity<Angle>> velocity = RotationsPerSecond.zero();
    Measure<Voltage> voltage = Volts.zero();

    @Override
    public void updateInputs(MotorIoInputs inputs) {
        inputs.velocityActual = inputs.velocityTarget = velocity;
        inputs.voltsTarget = inputs.voltsTarget = voltage;
    }

    @Override
    public Measure<Temperature> getMaxSafeTemperature() {
        return Celsius.of(Double.MAX_VALUE);
    }

    @Override
    public Rotation2d getAbsolutePosition() {
        return Rotation2d.fromRotations(0);
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
    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        this.velocity = velocity;
    }

    @Override
    public void setVoltage(Measure<Voltage> voltage) {
        this.voltage = voltage;
    }
}
