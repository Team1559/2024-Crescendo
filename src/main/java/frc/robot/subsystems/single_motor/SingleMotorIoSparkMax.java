package frc.robot.subsystems.single_motor;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class SingleMotorIoSparkMax implements SingleMotorIo {
    private final CANSparkMax motor;

    /**
     * Create a new subsystem for a single SparkMax-controlled motor in voltage mode
     * 
     * @param motorId  Motor CAN ID
     * @param inverted True if the motor direction should be inverted
     */
    public SingleMotorIoSparkMax(int motorId, boolean inverted) {
        motor = new CANSparkMax(motorId, MotorType.kBrushless);
        motor.setInverted(inverted);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(CONSTANTS.getNeoBrushlessCurrentLimit());
        motor.setSecondaryCurrentLimit(CONSTANTS.getNeoBrushlessCurrentSecondaryLimit());
    }

    public void updateInputs(SingleMotorIoInputs inputs) {
        inputs.appliedOutput = motor.getAppliedOutput();
        inputs.outputCurrent = motor.getOutputCurrent();
        inputs.motorTemp = motor.getMotorTemperature();
        inputs.faults = motor.getFaults();
        inputs.velocity = motor.getEncoder().getVelocity();
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }
}
