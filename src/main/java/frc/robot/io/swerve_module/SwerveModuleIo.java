package frc.robot.io.swerve_module;

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

public interface SwerveModuleIo {

    @AutoLog
    static class SwerveModuleIoInputs {

        // ---------- CAN Coder ----------
        public Rotation2d cancoderOffsetPosition = new Rotation2d();
        public Rotation2d cancoderAbsolutePosition = new Rotation2d();

        // ---------- Drive Motor ----------
        public Measure<Current> driveMotorCurrentActual = Amps.zero();
        public Measure<Current> driveMotorCurrentAvailable = Amps.zero();
        public String[] driveMotorFaults = new String[0];
        public Rotation2d driveMotorPositionAbsolute = new Rotation2d();
        /** From -1 to 1. */
        public float driveMotorPowerPercentage = 0;
        public Measure<Temperature> driveMotorTemp = Celsius.zero();
        public Measure<Voltage> driveMotorVoltsActual = Volts.zero();
        public Measure<Voltage> driveMotorVoltsAvailable = Volts.zero();
        public Measure<Voltage> driveMotorVoltsTarget = Volts.zero();
        public Measure<Velocity<Angle>> driveMotorVelocityActual = RotationsPerSecond.zero();
        public Measure<Velocity<Angle>> driveMotorVelocityTarget = RotationsPerSecond.zero();

        // ---------- Steer Motor ----------
        public Measure<Current> steerMotorCurrentActual = Amps.zero();
        public Measure<Current> steerMotorCurrentAvailable = Amps.zero();
        public String[] steerMotorFaults = new String[0];
        public Rotation2d steerMotorPositionAbsolute = new Rotation2d();
        /** From -1 to 1. */
        public float steerMotorPowerPercentage = 0;
        public Measure<Temperature> steerMotorTemp = Celsius.zero();
        public Measure<Voltage> steerMotorVoltsActual = Volts.zero();
        public Measure<Voltage> steerMotorVoltsAvailable = Volts.zero();
        public Measure<Voltage> steerMotorVoltsTarget = Volts.zero();
        public Measure<Velocity<Angle>> steerMotorVelocityActual = RotationsPerSecond.zero();
        public Measure<Velocity<Angle>> steerMotorVelocityTarget = RotationsPerSecond.zero();
    }

    public SwerveModuleIo clone();

    /**
     * @return The maximum safe operating temperature of these motors.
     */
    public Measure<Temperature> getMaxSafeMotorTemperature();

    /** Run the drive motor at the specified voltage. */
    public void setDriveVoltage(Measure<Voltage> volts);

    /** Run the turn motor at the specified voltage. */
    public void setTurnVoltage(Measure<Voltage> volts);

    /** Updates the set of loggable inputs. */
    public void updateInputs(SwerveModuleIoInputs inputs); // TODO: Update to use MotorIoInputs instead.
}