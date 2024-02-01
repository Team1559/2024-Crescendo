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

public class Feeder extends SubsystemBase {
    @AutoLog
    static class FeederInputs {
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

    private final CANSparkMax feedMotorL = new CANSparkMax(Constants.LEFT_FEED_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax feedMotorR = new CANSparkMax(Constants.RIGHT_FEED_MOTOR_ID, MotorType.kBrushless);
    private final FeederInputsAutoLogged inputs = new FeederInputsAutoLogged();

    public Feeder() {
        feedMotorL.setInverted(false);
        feedMotorR.setInverted(true);
        feedMotorL.setIdleMode(IdleMode.kBrake);
        feedMotorR.setIdleMode(IdleMode.kBrake);
    }

    private void updateInputs() {
        inputs.lAppliedOutput = feedMotorL.getAppliedOutput();
        inputs.lOutputCurrent = feedMotorL.getOutputCurrent();
        inputs.lMotorTemp = feedMotorL.getMotorTemperature();
        inputs.lFaults = feedMotorL.getFaults();
        inputs.lVelocity = feedMotorL.getEncoder().getVelocity();

        inputs.rAppliedOutput = feedMotorR.getAppliedOutput();
        inputs.rOutputCurrent = feedMotorR.getOutputCurrent();
        inputs.rMotorTemp = feedMotorR.getMotorTemperature();
        inputs.rFaults = feedMotorR.getFaults();
        inputs.rVelocity = feedMotorR.getEncoder().getVelocity();
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Shooter/Feeder", inputs);
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
        feedMotorR.setVoltage(voltage);
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
