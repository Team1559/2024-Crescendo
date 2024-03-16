package frc.robot.io.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class MotorIoSimulation implements MotorIo {

    private final DCMotorSim motor;

    Rotation2d targetPosition = Rotation2d.fromRotations(0);
    Measure<Velocity<Angle>> targetVelocity;
    Measure<Voltage> voltage;

    public MotorIoSimulation(DCMotor motorToSimulate, double gearRatio, Measure<Velocity<Distance>> momentOfInertia) {
        motor = new DCMotorSim(motorToSimulate, gearRatio, momentOfInertia.in(MetersPerSecond));
    }

    @Override
    public void updateInputs(MotorIoInputs inputs) {

        motor.update(0.02);

        inputs.currentActual = Amps.of(motor.getCurrentDrawAmps());
        // currentAvailable = // N/A.

        // inputs.faults = // N/A.

        inputs.positionAbsolute = getPositionAbsolute();
        inputs.positionRelative = getPositionRelative();
        inputs.positionTarget = targetPosition;

        // inputs.powerPercentage = // N/A.

        // inputs.temperature = // N/A.

        inputs.velocityActual = getVelocity();
        inputs.velocityTarget = targetVelocity;
        inputs.velocityTargetClamped = targetVelocity;

        inputs.voltsActual = getVoltage();
        // inputs.voltsAvailable = // N/A.
        inputs.voltsTarget = voltage;
        inputs.voltsTargetClamped = voltage;
    }

    // ========================= Functions =========================

    @Override
    public Measure<Current> getMaxSafeCurrent() {
        return Amps.of(Double.MAX_VALUE);
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
        return Rotation2d.fromRotations(motor.getAngularPositionRotations());
    }

    @Override
    public Rotation2d getPositionRelative() {
        return getPositionAbsolute().plus(Rotation2d.fromDegrees(0));
    }

    @Override
    public Measure<Temperature> getTemperature() {
        return Celsius.zero();
    }

    @Override
    public Measure<Velocity<Angle>> getVelocity() {
        return RadiansPerSecond.of(motor.getAngularVelocityRadPerSec());
    }

    @Override
    public Measure<Voltage> getVoltage() {
        return voltage;
    }

    @Override
    public void setPosition(Rotation2d position) {
        this.targetPosition = position;
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        this.targetVelocity = velocity;
    }

    @Override
    public void setVoltage(Measure<Voltage> voltage) {
        this.voltage = voltage;
    }

    @Override
    public void stop() {
        targetVelocity = null;
        voltage = null;
    }
}
