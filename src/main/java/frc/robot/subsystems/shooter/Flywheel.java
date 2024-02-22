package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Celsius;
import static frc.robot.constants.AbstractConstants.CONSTANTS;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve_module.SwerveModuleIoTalonFx;

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

        public Measure<Temperature> rMotorTemp;
        public Measure<Temperature> lMotorTemp;

        public int rFaults;
        public int lFaults;
    }

    private final StatusSignal<Double> flywheelLMotorVoltage, flywheelRMotorVoltage;
    private final StatusSignal<Double> flywheelLSupplyCurrent, flywheelRSupplyCurrent;
    private final StatusSignal<Double> flywheelLSupplyVoltage, flywheelRSupplyVoltage;
    private final StatusSignal<Double> flywheelLVelocity, flywheelRVelocity;
    private final StatusSignal<Double> flywheelLMotorTemp, flywheelRMotorTemp;
    private final StatusSignal<Integer> flywheelLFaults, flywheelRFaults;

    private final TalonFX flywheelMotorL = new TalonFX(CONSTANTS.getFlywheelMotorIdLeft());
    private final TalonFX flywheelMotorR = new TalonFX(CONSTANTS.getFlywheelMotorIdRight());

    private final FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();

    private double currentVoltage;
    /**
     * Null when both wheels run, true when right wheel runs, false when left wheel
     * runs.
     */
    private Boolean runOneWheelFlag;

    public Flywheel() {

        // ---------- Configure Motors ----------
        // Only set the Motor Configuration once, to avoid accidentally overriding
        // configs with defaults.
        flywheelMotorL.getConfigurator().apply(SwerveModuleIoTalonFx.getDefaultTalonFXConfiguration(
                InvertedValue.CounterClockwise_Positive /* default */, NeutralModeValue.Coast));
        flywheelMotorR.getConfigurator().apply(SwerveModuleIoTalonFx.getDefaultTalonFXConfiguration(
                InvertedValue.Clockwise_Positive /* inverted */, NeutralModeValue.Coast));

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
        BaseStatusSignal.setUpdateFrequencyForAll(
                CONSTANTS.getPathPlannerLogUpdateFrequencyDefault(),
                flywheelLMotorVoltage, flywheelRMotorVoltage,
                flywheelLSupplyCurrent, flywheelRSupplyCurrent,
                flywheelLSupplyVoltage, flywheelRSupplyVoltage,
                flywheelLVelocity, flywheelRVelocity,
                flywheelLMotorTemp, flywheelRMotorTemp,
                flywheelLFaults, flywheelRFaults);
        flywheelMotorL.optimizeBusUtilization();
        flywheelMotorR.optimizeBusUtilization();
    }

    @Override
    public void periodic() {

        // Set Voltages.
        if (runOneWheelFlag == null || runOneWheelFlag) {
            flywheelMotorR.setControl(new VoltageOut(currentVoltage));
        }
        if (runOneWheelFlag == null || !runOneWheelFlag) {
            flywheelMotorL.setControl(
                    new VoltageOut(currentVoltage * CONSTANTS.getFlywheelMotorPowerDifferentialPercentage()));
        }

        // Log Inputs.
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

        inputs.lMotorTemp = Celsius.of(flywheelLMotorTemp.getValueAsDouble());
        inputs.rMotorTemp = Celsius.of(flywheelRMotorTemp.getValueAsDouble());

        inputs.lFaults = flywheelLFaults.getValue();
        inputs.rFaults = flywheelRFaults.getValue();
    }

    // ========================= Functions =========================
    /**
     * @return The Temperature of the hottest motor.
     */
    public Measure<Temperature> getMaxTemperature() {
        return inputs.rMotorTemp.gt(inputs.lMotorTemp) ? inputs.rMotorTemp : inputs.lMotorTemp;
    }

    public boolean isTemperatureTooHigh() {
        // 90% Buffer.
        return getMaxTemperature()
                .gt(CONSTANTS.getFalcon500MaxTemperature().times(CONSTANTS.SAFE_MOTOR_TEMPERATURE_BUFFER));
    }

    /**
     * Start the Flywheels with a specific voltage
     * 
     * @param voltage Set Flywheels to this voltage
     */
    public void start(double voltage) {
        currentVoltage = voltage;
        runOneWheelFlag = null;
    }

    /**
     * Start the Flywheels with default volage
     */
    public void start() {
        start(CONSTANTS.getFlywheelForwardVoltage());
    }

    public void startOneMotor(boolean runRightWheel) {
        start();
        runOneWheelFlag = runRightWheel;
    }

    public double getCurrentVoltage() {
        return currentVoltage;
    }

    /**
     * Stop the Flywheels
     */
    public void stop() {
        currentVoltage = 0;
        flywheelMotorL.stopMotor();
        flywheelMotorR.stopMotor();
    }

    /**
     * Reverse the Flywheels
     */
    public void reverse() {
        start(CONSTANTS.getFlywheelReverseVoltage());
    }

    // ========================= Commands =========================
    public Command startCommand() {
        return new InstantCommand(this::start, this);
    }

    public Command startCommand(double voltage) {
        return new InstantCommand(() -> start(voltage), this);
    }

    public Command startOneMotorCommand(boolean runRightWheel) {
        return new InstantCommand(() -> startOneMotor(runRightWheel), this);
    }

    public Command stopCommand() {
        return new InstantCommand(this::stop, this);
    }

    public Command reverseCommand() {
        return new InstantCommand(this::reverse, this);
    }
}
