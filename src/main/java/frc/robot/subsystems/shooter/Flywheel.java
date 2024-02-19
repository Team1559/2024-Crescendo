package frc.robot.subsystems.shooter;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

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
        // TODO: Determine why these aren't being respected.
        // (Velecity is being inverted in #periodic() as a workaround.)
        // flywheelMotorL.setInverted(true);
        // flywheelMotorR.setInverted(false);
        flywheelMotorL.setNeutralMode(NeutralModeValue.Coast);
        flywheelMotorR.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();
        driveTalonFXConfiguration.CurrentLimits = CONSTANTS.getFalcon500CurrentLimitsConfigs();
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
        // Set Voltages
        if (runOneWheelFlag == null || runOneWheelFlag) {
            // TODO: Removbe this workaround to the inveted config not taking.
            flywheelMotorR.setControl(new VoltageOut(-currentVoltage));
        }
        if (runOneWheelFlag == null || !runOneWheelFlag) {
            flywheelMotorL.setControl(new VoltageOut(currentVoltage * .75));
        }

        // Log inputs
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
