package frc.robot.subsystems.single_motor;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

public class SingleMotorIoReplay implements SingleMotorIo {

    Measure<Velocity<Angle>> velocity = RotationsPerSecond.of(0);

    @Override
    public void updateInputs(SingleMotorIoInputs inputs) {
        inputs.velocityActual = inputs.velocityTarget = velocity;
    }

    public Measure<Temperature> getMaxSafeTemperature() {
        return Units.Celsius.of(Double.MAX_VALUE);
    }

    public Measure<Temperature> getTemperature() {
        return Units.Celsius.of(0);
    }

    @Override
    public Measure<Velocity<Angle>> getVelocity() {
        return velocity;
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        this.velocity = velocity;
    }
}
