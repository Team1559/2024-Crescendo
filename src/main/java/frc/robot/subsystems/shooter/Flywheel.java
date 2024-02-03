package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

        public double lSupplyCurrent;
        public double rSupplyCurrent;

        public double lSupplyVoltage;
        public double rSupplyVoltage;

        public double lVelocity;
        public double rVelocity;

        public double rMotorTemp;
        public double lMotorTemp;

        public int rFaults;
        public int lFaults;
    }

    private final StatusSignal<Double> flywheelLMotorVoltage, flywheelRMotorVoltage;
    private final StatusSignal<Double> flywheelLSupplyCurrent, flywheelRSupplyCurrent;
    private final StatusSignal<Double> flywheelLSupplyVoltage, flywheelRSupplyVoltage;
    private final StatusSignal<Double> flywheelLVelocity, flywheelRVelocity;
    private final StatusSignal<Double> flywheelLMotorTemp, flywheelRMotorTemp;
    private final StatusSignal<Integer> flywheelLFaults, flywheelRFaults;

    private final TalonFX flywheelMotorL = new TalonFX(Constants.FLYWHEEL_L_ID);
    private final TalonFX flywheelMotorR = new TalonFX(Constants.FLYWHEEL_R_ID);

    private final FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();

    public Flywheel() {

        // ---------- Configure Motors ----------
        flywheelMotorL.setInverted(true);
        flywheelMotorR.setInverted(false);
        flywheelMotorL.setNeutralMode(NeutralModeValue.Coast);
        flywheelMotorR.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();
        driveTalonFXConfiguration.CurrentLimits = Constants.getDefaultCurrentLimitsConfig();
        flywheelMotorL.getConfigurator().apply(driveTalonFXConfiguration);
        flywheelMotorR.getConfigurator().apply(driveTalonFXConfiguration);

        // ---------- Define Loggable Fields ----------
        flywheelLMotorVoltage = flywheelMotorL.getMotorVoltage();
        flywheelRMotorVoltage = flywheelMotorR.getMotorVoltage();
        flywheelLSupplyCurrent = flywheelMotorL.getSupplyCurrent();
        flywheelRSupplyCurrent = flywheelMotorR.getSupplyCurrent();
        flywheelLSupplyVoltage = flywheelMotorL.getSupplyVoltage();
        flywheelRSupplyVoltage = flywheelMotorR.getSupplyVoltage();
        flywheelLVelocity = flywheelMotorL.getVelocity();
        flywheelRVelocity = flywheelMotorR.getVelocity();
        flywheelLMotorTemp = flywheelMotorL.getDeviceTemp();
        flywheelRMotorTemp = flywheelMotorR.getDeviceTemp();
        flywheelLFaults = flywheelMotorL.getFaultField();
        flywheelRFaults = flywheelMotorR.getFaultField();

        // ---------- Optimize Bus Utilization ----------
        flywheelMotorL.optimizeBusUtilization();
        flywheelMotorR.optimizeBusUtilization();
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Shooter/Flywheel", inputs);
    }

    private void updateInputs() {
        BaseStatusSignal.refreshAll(
                flywheelLMotorVoltage, flywheelRMotorVoltage,
                flywheelLSupplyCurrent, flywheelRSupplyCurrent,
                flywheelLSupplyVoltage, flywheelRSupplyVoltage,
                flywheelLVelocity, flywheelRVelocity,
                flywheelLMotorTemp, flywheelRMotorTemp,
                flywheelLFaults, flywheelRFaults);

        inputs.lMotorVoltage = flywheelLMotorVoltage.getValueAsDouble();
        inputs.rMotorVoltage = flywheelRMotorVoltage.getValueAsDouble();

        inputs.lSupplyCurrent = flywheelLSupplyCurrent.getValueAsDouble();
        inputs.rSupplyCurrent = flywheelRSupplyCurrent.getValueAsDouble();

        inputs.lSupplyVoltage = flywheelLSupplyVoltage.getValueAsDouble();
        inputs.rSupplyVoltage = flywheelRSupplyVoltage.getValueAsDouble();

        inputs.lVelocity = flywheelLVelocity.getValueAsDouble();
        inputs.rVelocity = flywheelRVelocity.getValueAsDouble();

        inputs.lMotorTemp = flywheelLMotorTemp.getValueAsDouble();
        inputs.rMotorTemp = flywheelRMotorTemp.getValueAsDouble();

        inputs.lFaults = flywheelLFaults.getValue();
        inputs.rFaults = flywheelRFaults.getValue();
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
