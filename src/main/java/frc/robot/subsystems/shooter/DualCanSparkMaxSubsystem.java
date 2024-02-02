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

// TODO: Convert this into a generic Dual Spark Max Subsystem.
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

    private final CANSparkMax motorL = new CANSparkMax(Constants.LEFT_FEED_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax motorR = new CANSparkMax(Constants.RIGHT_FEED_MOTOR_ID, MotorType.kBrushless);
    private final DualMotorInputsAutoLogged inputs = new DualMotorInputsAutoLogged();

    public DualCanSparkMaxSubsystem() {
        motorL.setInverted(false);
        motorR.setInverted(true);
        motorL.setIdleMode(IdleMode.kBrake);
        motorR.setIdleMode(IdleMode.kBrake);
        motorL.setSmartCurrentLimit(Constants.NEO_SPARK_BRUSHLESS_CURRENT_LIMIT);
        motorR.setSmartCurrentLimit(Constants.NEO_SPARK_BRUSHLESS_CURRENT_LIMIT);
        motorL.setSecondaryCurrentLimit(Constants.NEO_SPARK_BRUSHLESS_CURRENT_SECONDARY_LIMIT);
        motorL.setSecondaryCurrentLimit(Constants.NEO_SPARK_BRUSHLESS_CURRENT_SECONDARY_LIMIT);
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Shooter/DualCanSparkMaxSubsystem", inputs);
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
    public void startFeeder() {
        setVoltage(Constants.FEEDER_FORWARD_VOLTAGE);
    }

    public void stopFeeder() {
        setVoltage(0.0);
    }

    public void reverseFeeder() {
        setVoltage(Constants.FEEDER_REVERSE_VOLTAGE);
    }

    private void setVoltage(double voltage) {
        motorR.setVoltage(voltage);
    }

    // ========================= Commands =========================

    public Command startFeederCommand() {
        return new InstantCommand(this::startFeeder, this);
    }

    public Command stopFeederCommand() {
        return new InstantCommand(this::stopFeeder, this);
    }

    public Command reverseFeederCommand() {
        return new InstantCommand(this::reverseFeeder, this);
    }

}
