package frc.robot.io.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public interface MotorIo {

    @AutoLog
    static class MotorIoInputs {

        public Measure<Current> currentActual = Amps.zero();

        public String[] faults = new String[0];

        public Rotation2d positionAbsolute = new Rotation2d();
        public Rotation2d positionTarget = null;

        /** From -1 to 1. */
        public float powerPercentage = 0;

        public Measure<Temperature> temperature = Celsius.zero();

        public Measure<Velocity<Angle>> velocityActual = RotationsPerSecond.zero();
        public Measure<Velocity<Angle>> velocityTarget = null;

        public Measure<Voltage> voltsActual = Volts.zero();
        public Measure<Voltage> voltsAvailable = Volts.zero();
        public Measure<Voltage> voltsTarget = null;
    }

    public void updateInputs(MotorIoInputs inputs);

    public Measure<Temperature> getMaxSafeTemperature();

    /**
     * @return The position from the absolute encoder.
     */
    public Rotation2d getAbsolutePosition();

    public Measure<Temperature> getTemperature();

    public Measure<Velocity<Angle>> getVelocity();

    public Measure<Voltage> getVoltage();

    public void setPosition(Rotation2d position);

    public void setVelocity(Measure<Velocity<Angle>> velocity);

    public void setVoltage(Measure<Voltage> voltage);

    public void stop();
}
