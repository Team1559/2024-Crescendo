package frc.robot.io.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import java.util.LinkedList;
import java.util.List;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.constants.AbstractConstants.PID;

public abstract class MotorIoSparkMax implements MotorIo {

    protected final CANSparkMax motor;
    protected Measure<Velocity<Angle>> targetVelocity;
    protected boolean inverted;

    /**
     * Create a new subsystem for a single SparkMax-controlled motor in voltage mode
     * 
     * @param motorId  Motor CAN ID
     * @param inverted True if the motor direction should be inverted
     */
    public MotorIoSparkMax(int motorId, boolean inverted, PID pidValues) {

        // Create & Configure Motor.
        motor = new CANSparkMax(motorId, MotorType.kBrushless);
        // Randomly flips back. TODO: Figure out why?
        // motor.setInverted(false);
        this.inverted = inverted;
        motor.setIdleMode(IdleMode.kBrake);

        // Configure piD Controller.
        motor.getPIDController().setP(pidValues.P);
        motor.getPIDController().setI(pidValues.I);
        motor.getPIDController().setD(pidValues.D);
        motor.getPIDController().setFF(pidValues.FF);
    }

    @Override
    public void updateInputs(MotorIoInputs inputs) {

        List<String> faults = new LinkedList<>();
        for (FaultID faultID : FaultID.values()) {
            if (motor.getFault(faultID)) {
                faults.add(faultID.name());
            }
        }
        inputs.faults = faults.toArray(new String[0]);

        inputs.appliedOutput = (float) motor.getAppliedOutput();

        inputs.outputCurrent = Amps.of(motor.getOutputCurrent());
        inputs.motorTemp = Celsius.of(motor.getMotorTemperature());
        inputs.velocityActual = getVelocity();
        inputs.velocityTarget = targetVelocity;
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
    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        motor.getPIDController().setReference(
                inverted ? velocity.negate().in(RevolutionsPerSecond) : velocity.in(RevolutionsPerSecond),
                CANSparkMax.ControlType.kVelocity);

        targetVelocity = velocity;
    }
}
