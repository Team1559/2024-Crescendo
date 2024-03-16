package org.victorrobotics.frc.io.motor.can_spark_max;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.LinkedList;
import java.util.List;

import org.victorrobotics.frc.Constants.PidValues;
import org.victorrobotics.frc.io.motor.MotorIo;
import org.victorrobotics.frc.util.MathUtils;

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

public abstract class MotorIoCanSparkMax implements MotorIo {

    protected final boolean isInverted;
    protected final CANSparkMax motor;
    protected final Rotation2d absoluteEncoderOffset;

    protected Rotation2d targetPosition;
    protected Measure<Velocity<Angle>> targetVelocity, targetVelocityClamped;
    protected Measure<Voltage> targetVoltage, targetVoltageClamped;

    /**
     * Create a new subsystem for a single SparkMax-controlled motor in voltage mode
     * 
     * @param motorId               Motor CAN ID
     * @param inverted              True if the motor direction should be inverted
     * @param idleMode              What the motor should do when no commands are
     *                              being sent to
     *                              it.
     * @param absoluteEncoderOffset The offset for the internal/external encoder.
     * @param pidValues             The values to configure the motors built in PID
     *                              Controller.
     *                              <p>
     *                              (If {@code null}, the the
     *                              {@link MotorIo#DEFAULT_PID_VALUES}
     *                              will be used.)
     *                              </p>
     */
    public MotorIoCanSparkMax(int motorId, boolean inverted, IdleMode idleMode, Rotation2d absoluteEncoderOffset,
            PidValues pidValues) {

        // Create & Configure Motor.
        motor = new CANSparkMax(motorId, MotorType.kBrushless);
        // Randomly flips back. TODO: Figure out why?
        motor.setInverted(false);
        this.isInverted = inverted;
        motor.setIdleMode(idleMode);

        // Configure Encoder.
        this.absoluteEncoderOffset = absoluteEncoderOffset;

        // Configure piD Controller.
        pidValues = pidValues == null ? DEFAULT_PID_VALUES : pidValues;
        motor.getPIDController().setP(pidValues.P);
        motor.getPIDController().setI(pidValues.I);
        motor.getPIDController().setD(pidValues.D);
        motor.getPIDController().setFF(pidValues.FF_S);
    }

    @Override
    public void updateInputs(MotorIoInputs inputs) {

        inputs.currentActual = Amps.of(motor.getOutputCurrent());
        // currentAvailable= // Not Supported.

        List<String> faults = new LinkedList<>();
        for (FaultID faultID : FaultID.values()) {
            if (motor.getFault(faultID)) {
                faults.add(faultID.name());
            }
        }
        inputs.faults = faults.toArray(new String[0]);

        inputs.positionAbsolute = getPositionAbsolute();
        inputs.positionRelative = getPositionRelative();
        inputs.positionTarget = targetPosition;

        inputs.powerPercentage = (float) motor.getAppliedOutput();

        inputs.temperature = getTemperature();

        inputs.velocityActual = getVelocity();
        inputs.velocityTarget = targetVelocity;
        inputs.velocityTargetClamped = targetVelocityClamped;

        inputs.voltsActual = getVoltage();
        inputs.voltsAvailable = Volts.of(motor.getBusVoltage());
        inputs.voltsTarget = targetVoltage;
        inputs.voltsTargetClamped = targetVoltageClamped;
    }

    // ========================= Functions =========================

    @Override
    public Rotation2d getPositionAbsolute() {
        return Rotation2d.fromRotations(motor.getAbsoluteEncoder().getPosition()).plus(absoluteEncoderOffset);
    }

    @Override
    public Rotation2d getPositionRelative() {

        Rotation2d positionRelative = Rotation2d.fromRotations(motor.getEncoder().getPosition());

        // Forces the Rotation2d object to detect if the continuous input threshold has
        // been exceeded and account for it.
        positionRelative = positionRelative.plus(Rotation2d.fromDegrees(0));

        return positionRelative;
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

        targetPosition = position;
        targetVelocity = null;
        targetVelocityClamped = null;
        targetVoltage = null;
        targetVoltageClamped = null;

        motor.getPIDController().setReference(isInverted ? -position.getRotations() : position.getRotations(),
                CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity) {

        targetPosition = null;
        targetVelocity = velocity;
        targetVelocityClamped = MathUtils.clamp(velocity, getMaxSafeVelocity());
        targetVoltage = null;
        targetVoltageClamped = null;

        motor.getPIDController().setReference(isInverted ? targetVelocityClamped.negate().in(RevolutionsPerSecond)
                : targetVelocityClamped.in(RevolutionsPerSecond), CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setVoltage(Measure<Voltage> voltage) {

        targetPosition = null;
        targetVelocity = null;
        targetVelocityClamped = null;
        targetVoltage = voltage;
        targetVoltageClamped = MathUtils.clamp(voltage, getMaxSafeVoltage());

        motor.getPIDController().setReference(isInverted ? targetVoltageClamped.negate().in(Volts)
                : targetVoltageClamped.in(Volts), CANSparkMax.ControlType.kVoltage);
    }

    @Override
    public void stop() {
        motor.stopMotor();
        targetPosition = null;
        targetVelocity = null;
        targetVoltage = null;
    }
}
