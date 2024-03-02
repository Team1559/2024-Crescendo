package frc.robot.io.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;

public interface MotorIo {

    @AutoLog
    static class MotorIoInputs {

        public String[] faults = new String[0],
                stickyFaults = new String[0];

        /**
         * Percentage of power from -1 to 1.
         */
        public double appliedOutput;

        public Measure<Current> outputCurrent = Amps.of(0);

        public Measure<Temperature> motorTemperature = Celsius.of(0);

        public Measure<Velocity<Angle>> velocityActual = RotationsPerSecond.of(0);
        public Measure<Velocity<Angle>> velocityTarget = RotationsPerSecond.of(0);
    }

    public void updateInputs(MotorIoInputs inputs);

    public Measure<Temperature> getMaxSafeTemperature();

    public Measure<Temperature> getTemperature();

    public Measure<Velocity<Angle>> getVelocity();

    public void setVelocity(Measure<Velocity<Angle>> velocity);
}
