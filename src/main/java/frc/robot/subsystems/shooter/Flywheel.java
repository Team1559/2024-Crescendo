package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
    @AutoLog
    static class FlywheelInputs {
        public double lMotorVoltage;
        public double rMotorVoltage;
        public double lStatorCurrent;
        public double rStatorCurrent;
        public double lSupplyVoltage;
        public double rSupplyVoltage;
        public double lVelocity;
        public double rVelocity;
    }

    private final StatusSignal<Double> flywheelLMotorVoltage, flywheelRMotorVoltage;
    private final StatusSignal<Double> flywheelLStatorCurrent, flywheelRStatorCurrent;
    private final StatusSignal<Double> flywheelLSupplyVoltage, flywheelRSupplyVoltage;
    private final StatusSignal<Double> flywheelLVelocity, flywheelRVelocity;

    private final TalonFX flywheelMotorL = new TalonFX(Constants.FLYWHEEL_L_ID);
    private final TalonFX flywheelMotorR = new TalonFX(Constants.FLYWHEEL_R_ID);
    private final FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();

    public Flywheel() {

        // ---------- Configure Motors ----------
        flywheelMotorL.setInverted(true);
        flywheelMotorR.setInverted(false);
        flywheelMotorL.setNeutralMode(NeutralModeValue.Coast);
        flywheelMotorR.setNeutralMode(NeutralModeValue.Coast);

        // ---------- Define Loggable Fields ----------
        flywheelLMotorVoltage = flywheelMotorL.getMotorVoltage();
        flywheelRMotorVoltage = flywheelMotorR.getMotorVoltage();
        flywheelLStatorCurrent = flywheelMotorL.getStatorCurrent();
        flywheelRStatorCurrent = flywheelMotorR.getStatorCurrent();
        flywheelLSupplyVoltage = flywheelMotorL.getSupplyVoltage();
        flywheelRSupplyVoltage = flywheelMotorR.getSupplyVoltage();
        flywheelLVelocity = flywheelMotorL.getVelocity();
        flywheelRVelocity = flywheelMotorR.getVelocity();

        // ---------- Optimize Bus Utilization ----------
        BaseStatusSignal.setUpdateFrequencyForAll(Constants.ADVANTAGE_DEFAULT_LOG_FREQUENCY,
                flywheelLMotorVoltage, flywheelRMotorVoltage,
                flywheelLStatorCurrent, flywheelRStatorCurrent,
                flywheelLSupplyVoltage, flywheelRSupplyVoltage,
                flywheelLVelocity, flywheelRVelocity);
        flywheelMotorL.optimizeBusUtilization();
        flywheelMotorR.optimizeBusUtilization();
    }

    private void updateInputs() {
        inputs.lMotorVoltage = flywheelLMotorVoltage.getValueAsDouble();
        inputs.rMotorVoltage = flywheelRMotorVoltage.getValueAsDouble();
        inputs.lStatorCurrent = flywheelLStatorCurrent.getValueAsDouble();
        inputs.rStatorCurrent = flywheelRStatorCurrent.getValueAsDouble();
        inputs.lSupplyVoltage = flywheelLSupplyVoltage.getValueAsDouble();
        inputs.rSupplyVoltage = flywheelRSupplyVoltage.getValueAsDouble();
        inputs.lVelocity = flywheelLVelocity.getValueAsDouble();
        inputs.rVelocity = flywheelRVelocity.getValueAsDouble();
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Shooter/Flywheel", inputs);
    }

    // ========================= Functions =========================

    public void startFlywheel(double voltage) {
        flywheelMotorL.setControl(new VoltageOut(voltage));
        flywheelMotorR.setControl(new VoltageOut(voltage));
    }

    public void stopFlywheel() {
        flywheelMotorL.stopMotor();
        flywheelMotorR.stopMotor();
    }

    public void reverseFlywheel() {
        startFlywheel(Constants.FLYWHEEL_REVERSE_VOLTAGE);
    }

    // ========================= Commands =========================

    public Command startFlywheelCommand(double voltage) {
        return new InstantCommand(() -> startFlywheel(voltage), this);
    }

    public Command stopFlywheelCommand() {
        return new InstantCommand(this::stopFlywheel, this);
    }

    public Command reverseFlywheelCommand() {
        return new InstantCommand(this::reverseFlywheel, this);
    }
}
