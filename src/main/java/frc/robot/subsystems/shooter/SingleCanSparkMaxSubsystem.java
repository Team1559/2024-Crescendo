package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SingleCanSparkMaxSubsystem extends SubsystemBase {

    @AutoLog
    static class SingleCanSparkMaxSubsystemInputs {
        public double appliedOutput;
        public double outputCurrent;
        public double motorTemp;
        public int faults;
        public double velocity;
    }

    public final double fowardsVoltage;
    public final double reverseVoltage;

    private final CANSparkMax motor;
    private final SingleCanSparkMaxSubsystemInputsAutoLogged inputs = new SingleCanSparkMaxSubsystemInputsAutoLogged();

    /**
     * Create a new subsystem for two motors controlled by CANspark Controller
     * 
     * @param name    Name of subsystem
     * @param motorId Left motor ID
     * @param voltage Voltage for both fowards and reverse voltage
     */
    protected SingleCanSparkMaxSubsystem(String name, int motorId, double voltage) {
        this(name, motorId, voltage, voltage);
    }

    /**
     * Create a new subsystem for two motors controlled by CANspark Controller
     * 
     * @param name           Name of subsystem
     * @param motorId        Left motor ID
     * @param fowardsVoltage Voltage for fowards movement
     * @param reverseVoltage Voltage for reverse movement
     */
    protected SingleCanSparkMaxSubsystem(String name, int motorId, double fowardsVoltage, double reverseVoltage) {
        super(name);
        motor = new CANSparkMax(motorId, MotorType.kBrushless);
        motor.setInverted(false); // TODO: This needs to be passed into the constructor.
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(Constants.NEO_SPARK_BRUSHLESS_CURRENT_LIMIT);
        motor.setSecondaryCurrentLimit(Constants.NEO_SPARK_BRUSHLESS_CURRENT_SECONDARY_LIMIT);
        this.fowardsVoltage = fowardsVoltage;
        this.reverseVoltage = reverseVoltage;
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Shooter/" + getName(), inputs);
    }

    private void updateInputs() {
        inputs.appliedOutput = motor.getAppliedOutput();
        inputs.outputCurrent = motor.getOutputCurrent();
        inputs.motorTemp = motor.getMotorTemperature();
        inputs.faults = motor.getFaults();
        inputs.velocity = motor.getEncoder().getVelocity();
    }

    // ========================= Functions =========================
    public void start() {
        setVoltage(fowardsVoltage);
    }

    public void stop() {
        setVoltage(0.0);
    }

    public void reverse() {
        setVoltage(reverseVoltage);
    }

    private void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    // ========================= Commands =========================

    public Command startCommand() {
        return new InstantCommand(this::start, this);
    }

    public Command stopCommand() {
        return new InstantCommand(this::stop, this);
    }

    public Command reverseCommand() {
        return new InstantCommand(this::reverse, this);
    }

}
