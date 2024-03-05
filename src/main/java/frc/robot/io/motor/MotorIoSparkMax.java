package frc.robot.io.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.LinkedList;
import java.util.List;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import frc.robot.Constants.PID;

public abstract class MotorIoSparkMax implements MotorIo {

    protected final boolean isInverted;
    protected final CANSparkMax motor;
    protected final Rotation2d absoluteEncoderOffset;

    protected Rotation2d targetPosition;
    protected Measure<Velocity<Angle>> targetVelocity;
    protected Measure<Voltage> targetVoltage;

    /**
     * Create a new subsystem for a single SparkMax-controlled motor in voltage mode
     * 
     * @param motorId  Motor CAN ID
     * @param inverted True if the motor direction should be inverted
     */
    public MotorIoSparkMax(int motorId, boolean inverted, IdleMode idleMode, Rotation2d absoluteEncoderOffset,
            PID pidValues) {

        // Create & Configure Motor.
        motor = new CANSparkMax(motorId, MotorType.kBrushless);
        // Randomly flips back. TODO: Figure out why?
        // motor.setInverted(false);
        this.isInverted = inverted;
        motor.setIdleMode(idleMode);

        // Configure Encoder.
        this.absoluteEncoderOffset = absoluteEncoderOffset;

        // Configure piD Controller.
        motor.getPIDController().setP(pidValues.P);
        motor.getPIDController().setI(pidValues.I);
        motor.getPIDController().setD(pidValues.D);
        motor.getPIDController().setFF(pidValues.FF);
    }

    @Override
    public void updateInputs(MotorIoInputs inputs) {

        inputs.powerPercentage = (float) motor.getAppliedOutput();

        inputs.currentActual = Amps.of(motor.getOutputCurrent());

        List<String> faults = new LinkedList<>();
        for (FaultID faultID : FaultID.values()) {
            if (motor.getFault(faultID)) {
                faults.add(faultID.name());
            }
        }
        inputs.faults = faults.toArray(new String[0]);

        inputs.temperature = getTemperature();

        inputs.positionAbsolute = getAbsolutePosition();
        inputs.positionTarget = targetPosition;

        inputs.voltsActual = getVoltage();
        inputs.voltsAvailable = Volts.of(motor.getBusVoltage());
        inputs.voltsTarget = targetVoltage;

        inputs.velocityActual = getVelocity();
        inputs.velocityTarget = targetVelocity;
    }

    // ========================= Functions =========================

    @Override
    public Rotation2d getAbsolutePosition() {
        return Rotation2d.fromRotations(motor.getAbsoluteEncoder().getPosition()).plus(absoluteEncoderOffset);
    }

    @Override
    public Measure<Temperature> getTemperature() {
        return Units.Celsius.of(motor.getMotorTemperature());
    }

    @Override
    public Measure<Velocity<Angle>> getVelocity() {
        return RevolutionsPerSecond.of(motor.getEncoder().getVelocity());
    }

    @Override
    public Measure<Voltage> getVoltage() {
        return Volts.of(motor.getBusVoltage() * motor.getAppliedOutput());
    }

    @Override
    public void setPosition(Rotation2d position) {
        motor.getPIDController().setReference(isInverted ? -position.getRotations() : position.getRotations(),
                CANSparkMax.ControlType.kPosition);
        targetPosition = position;
        targetVelocity = null;
        targetVoltage = null;
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        motor.getPIDController().setReference(
                isInverted ? velocity.negate().in(RevolutionsPerSecond) : velocity.in(RevolutionsPerSecond),
                CANSparkMax.ControlType.kVelocity);
        targetVelocity = velocity;
        targetPosition = null;
        targetVoltage = null;
    }

    @Override
    public void setVoltage(Measure<Voltage> voltage) {
        motor.getPIDController().setReference(isInverted ? voltage.negate().in(Volts) : voltage.in(Volts),
                CANSparkMax.ControlType.kVoltage);
        targetVoltage = voltage;
        targetPosition = null;
        targetVelocity = null;
    }
}
