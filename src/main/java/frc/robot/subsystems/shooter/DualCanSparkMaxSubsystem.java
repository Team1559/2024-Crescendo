package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DualCanSparkMaxSubsystem extends SubsystemBase {

    @AutoLog
    static class DualMotorInputs {
        public double lAppliedOutput;
        public double lOutputCurrent;
        public double lMotorTemp;
        public int lFaults;
        public double lVelocity;

        public double rAppliedOutput;
        public double rOutputCurrent;
        public double rMotorTemp;
        public int rFaults;
        public double rVelocity;
    }

    private final CANSparkMax motorL;
    private final CANSparkMax motorR;
    private final DualMotorInputsAutoLogged inputs = new DualMotorInputsAutoLogged();
    public final double fowardsVoltage;
    public final double reverseVoltage;

    /**
     * Create a new subsystem for two motors controlled by CANspark Controller
     * 
     * @param name     Name of subsystem
     * @param motorLId Left motor ID
     * @param motorRId Right motor ID
     * @param voltage  Voltage for both fowards and reverse voltage
     */
    public DualCanSparkMaxSubsystem(String name, int motorLId, int motorRId, double voltage) {
        this(name, motorLId, motorRId, voltage, voltage);
    }

    /**
     * Create a new subsystem for two motors controlled by CANspark Controller
     * 
     * @param name           Name of subsystem
     * @param motorLId       Left motor ID
     * @param motorRId       Right motor ID
     * @param fowardsVoltage Voltage for fowards movement
     * @param reverseVoltage Voltage for reverse movement
     */
    public DualCanSparkMaxSubsystem(String name, int motorLId, int motorRId, double fowardsVoltage,
            double reverseVoltage) {
        super(name);
        motorL = new CANSparkMax(motorLId, MotorType.kBrushless);
        motorR = new CANSparkMax(motorRId, MotorType.kBrushless);
        motorL.setInverted(false);
        motorR.setInverted(true);
        motorL.setIdleMode(IdleMode.kBrake);
        motorR.setIdleMode(IdleMode.kBrake);
        motorL.setSmartCurrentLimit(Constants.NEO_SPARK_BRUSHLESS_CURRENT_LIMIT);
        motorR.setSmartCurrentLimit(Constants.NEO_SPARK_BRUSHLESS_CURRENT_LIMIT);
        motorL.setSecondaryCurrentLimit(Constants.NEO_SPARK_BRUSHLESS_CURRENT_SECONDARY_LIMIT);
        motorL.setSecondaryCurrentLimit(Constants.NEO_SPARK_BRUSHLESS_CURRENT_SECONDARY_LIMIT);
        this.fowardsVoltage = fowardsVoltage;
        this.reverseVoltage = reverseVoltage;

    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Shooter/" + getName(), inputs);
    }

    private void updateInputs() {
        inputs.lAppliedOutput = motorL.getAppliedOutput();
        inputs.lOutputCurrent = motorL.getOutputCurrent();
        inputs.lMotorTemp = motorL.getMotorTemperature();
        inputs.lFaults = motorL.getFaults();
        inputs.lVelocity = motorL.getEncoder().getVelocity();

        inputs.rAppliedOutput = motorR.getAppliedOutput();
        inputs.rOutputCurrent = motorR.getOutputCurrent();
        inputs.rMotorTemp = motorR.getMotorTemperature();
        inputs.rFaults = motorR.getFaults();
        inputs.rVelocity = motorR.getEncoder().getVelocity();
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
        motorR.setVoltage(voltage);
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
