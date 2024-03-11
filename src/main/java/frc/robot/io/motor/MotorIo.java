package frc.robot.io.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import frc.robot.Constants.PidValues;

public interface MotorIo {

    /**
     * <ul>
     * <li><b>P:</b> {@code 1}</li>
     * <li><b>I:</b> {@code 0}</li>
     * <li><b>D:</b> {@code 0}</li>
     * <li><b>FF:</b> {@code 0}</li>
     * </ul>
     */
    public static PidValues DEFAULT_PID_VALUES = new PidValues(1, 0, 0, 0);

    @AutoLog
    static class MotorIoInputs {

        public Measure<Current> currentActual = Amps.zero();
        public Measure<Current> currentAvailable = null;

        public String[] faults = new String[0];

        public Rotation2d positionAbsolute = null;
        public Rotation2d positionRelative = null;
        public Rotation2d positionTarget = null;

        /** From -1 to 1. */
        public float powerPercentage = 0;

        public Measure<Temperature> temperature = null;

        public Measure<Velocity<Angle>> velocityActual = null;
        public Measure<Velocity<Angle>> velocityTarget = null;
        public Measure<Velocity<Angle>> velocityTargetClamped = null;

        public Measure<Voltage> voltsActual = Volts.zero();
        public Measure<Voltage> voltsAvailable = null;
        public Measure<Voltage> voltsTarget = null;
        public Measure<Voltage> voltsTargetClamped = null;
    }

    public void updateInputs(MotorIoInputs inputs);

    public Measure<Temperature> getMaxSafeTemperature();

    public Measure<Velocity<Angle>> getMaxSafeVelocity();

    public Measure<Voltage> getMaxSafeVoltage();

    /**
     * @return The position from the absolute encoder.
     */
    public Rotation2d getPositionAbsolute();

    /**
     * @return The position from the relative encoder.
     */
    public Rotation2d getPositionRelative();

    public Measure<Temperature> getTemperature();

    public Measure<Velocity<Angle>> getVelocity();

    public Measure<Voltage> getVoltage();

    public void setPosition(Rotation2d position);

    public void setVelocity(Measure<Velocity<Angle>> velocity);

    public void setVoltage(Measure<Voltage> voltage);

    public void stop();
}
