package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CONSTANTS;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {

    @AutoLog
    static class FlywheelInputs {

        public Measure<Current> currentActual = Amps.zero();
        public Measure<Current> currentAvailable = Amps.zero();

        public String[] faults = new String[0];

        public Rotation2d positionAbsolute = new Rotation2d();

        /** From -1 to 1. */
        public float powerPercentage = 0;

        public Measure<Temperature> temperature = Celsius.zero();

        public Measure<Voltage> voltsActual = Volts.zero();
        public Measure<Voltage> voltsAvailable = Volts.zero();
        public Measure<Voltage> voltsTarget = Volts.zero();

        public Measure<Velocity<Angle>> velocityActual = RotationsPerSecond.zero();
        public Measure<Velocity<Angle>> velocityTarget = RotationsPerSecond.zero();
    }

    private final StatusSignal<Double> flywheelLDeviceTemp, flywheelRDeviceTemp;
    private final StatusSignal<Double> flywheelLDutyCycle, flywheelRDutyCycle;
    private final StatusSignal<Double> flywheelLMotorVoltage, flywheelRMotorVoltage;
    private final StatusSignal<Double> flywheelLPosition, flywheelRPosition;
    private final StatusSignal<Double> flywheelLSupplyCurrent, flywheelRSupplyCurrent;
    private final StatusSignal<Double> flywheelLSupplyVoltage, flywheelRSupplyVoltage;
    private final StatusSignal<Double> flywheelLTorqueCurrent, flywheelRTorqueCurrent;
    private final StatusSignal<Double> flywheelLVelocity, flywheelRVelocity;
    private final Map<String, StatusSignal<Boolean>> flywheelLFaults, flywheelRFaults;
    private final List<StatusSignal<?>> statusSignals = new LinkedList<>();
    private final StatusSignal<?>[] statusSignalArray;

    // TODO: Create TalonFXIo class that extends the MotorIo class, and use that.
    private final TalonFX flywheelMotorL, flywheelMotorR;

    private final FlywheelInputsAutoLogged lInputs = new FlywheelInputsAutoLogged();
    private final FlywheelInputsAutoLogged rInputs = new FlywheelInputsAutoLogged();

    private Measure<Voltage> targetVoltage;

    /**
     * Null when both wheels run, true when right wheel runs, false when left wheel
     * runs.
     */
    private Boolean runOneWheelFlag;

    public Flywheel() {

        // ---------- Create & Configure Motors ----------
        flywheelMotorL = new TalonFX(CONSTANTS.getFlywheelMotorIdLeft());
        flywheelMotorR = new TalonFX(CONSTANTS.getFlywheelMotorIdRight());

        // Only set the Motor Configuration once, to avoid accidentally overriding
        // configs with defaults.
        flywheelMotorL.getConfigurator().apply(CONSTANTS.getDefaultTalonFXConfiguration(
                InvertedValue.CounterClockwise_Positive /* default */, NeutralModeValue.Coast));
        flywheelMotorR.getConfigurator().apply(CONSTANTS.getDefaultTalonFXConfiguration(
                InvertedValue.Clockwise_Positive /* inverted */, NeutralModeValue.Coast));

        // ---------- Define Loggable Fields ----------
        flywheelLFaults = CONSTANTS.getAllGetFaultStatusSignalMethods(flywheelMotorL);
        flywheelRFaults = CONSTANTS.getAllGetFaultStatusSignalMethods(flywheelMotorR);

        statusSignals.addAll(flywheelLFaults.values());
        statusSignals.addAll(flywheelRFaults.values());

        statusSignals.add(flywheelLDeviceTemp = flywheelMotorL.getDeviceTemp());
        statusSignals.add(flywheelRDeviceTemp = flywheelMotorR.getDeviceTemp());

        statusSignals.add(flywheelLDutyCycle = flywheelMotorL.getDutyCycle());
        statusSignals.add(flywheelRDutyCycle = flywheelMotorR.getDutyCycle());

        statusSignals.add(flywheelLMotorVoltage = flywheelMotorL.getMotorVoltage());
        statusSignals.add(flywheelRMotorVoltage = flywheelMotorR.getMotorVoltage());

        statusSignals.add(flywheelLPosition = flywheelMotorL.getPosition());
        statusSignals.add(flywheelRPosition = flywheelMotorR.getPosition());

        statusSignals.add(flywheelLSupplyCurrent = flywheelMotorL.getSupplyCurrent());
        statusSignals.add(flywheelRSupplyCurrent = flywheelMotorR.getSupplyCurrent());

        statusSignals.add(flywheelLSupplyVoltage = flywheelMotorL.getSupplyVoltage());
        statusSignals.add(flywheelRSupplyVoltage = flywheelMotorR.getSupplyVoltage());

        statusSignals.add(flywheelLTorqueCurrent = flywheelMotorL.getTorqueCurrent());
        statusSignals.add(flywheelRTorqueCurrent = flywheelMotorR.getTorqueCurrent());

        statusSignals.add(flywheelLVelocity = flywheelMotorL.getVelocity());
        statusSignals.add(flywheelRVelocity = flywheelMotorR.getVelocity());

        // ---------- Optimize Bus Utilization ----------
        statusSignalArray = statusSignals.toArray(new StatusSignal[0]);
        BaseStatusSignal.setUpdateFrequencyForAll(CONSTANTS.getPathPlannerLogUpdateFrequencyDefault(),
                statusSignalArray);
        flywheelMotorL.optimizeBusUtilization();
        flywheelMotorR.optimizeBusUtilization();
    }

    @Override
    public void periodic() {

        // Set/Log Voltages.
        if (runOneWheelFlag == null || runOneWheelFlag) {
            lInputs.voltsTarget = getTargetVoltage();
            flywheelMotorR.setControl(new VoltageOut(lInputs.voltsTarget.in(Volts)));
        }
        if (runOneWheelFlag == null || !runOneWheelFlag) {
            rInputs.voltsTarget = getTargetVoltage().times(CONSTANTS.flywheelSpinOffset());
            flywheelMotorL.setControl(new VoltageOut(rInputs.voltsTarget.in(Volts)));
        }

        // Log Inputs.
        BaseStatusSignal.refreshAll(statusSignalArray);

        lInputs.currentActual = Amps.of(flywheelLTorqueCurrent.getValue());
        rInputs.currentActual = Amps.of(flywheelRTorqueCurrent.getValue());

        lInputs.currentAvailable = Amps.of(flywheelLSupplyCurrent.getValue());
        rInputs.currentAvailable = Amps.of(flywheelRSupplyCurrent.getValue());

        lInputs.faults = CONSTANTS.getFaults(flywheelLFaults);
        rInputs.faults = CONSTANTS.getFaults(flywheelRFaults);

        lInputs.temperature = Celsius.of(flywheelLDeviceTemp.getValue());
        rInputs.temperature = Celsius.of(flywheelRDeviceTemp.getValue());

        lInputs.voltsActual = Volts.of(flywheelLMotorVoltage.getValue());
        rInputs.voltsActual = Volts.of(flywheelRMotorVoltage.getValue());

        lInputs.positionAbsolute = Rotation2d.fromRotations(flywheelLPosition.getValue());
        rInputs.positionAbsolute = Rotation2d.fromRotations(flywheelRPosition.getValue());

        lInputs.powerPercentage = (float) (flywheelLDutyCycle.getValue() / 2);
        rInputs.powerPercentage = (float) (flywheelRDutyCycle.getValue() / 2);

        lInputs.voltsAvailable = Volts.of(flywheelLSupplyVoltage.getValue());
        rInputs.voltsAvailable = Volts.of(flywheelRSupplyVoltage.getValue());

        lInputs.velocityActual = RotationsPerSecond.of(flywheelLVelocity.getValue());
        rInputs.velocityActual = RotationsPerSecond.of(flywheelRVelocity.getValue());

        // Not Currently Supported. (TODO)
        // lInputs.velocityTarget =
        // rInputs.velocityTarget =

        Logger.processInputs("Shooter/Flywheel/LeftMotor", lInputs);
        Logger.processInputs("Shooter/Flywheel/RightMotor", rInputs);
    }

    // ========================= Functions =========================
    /**
     * @return The Temperature of the hottest motor.
     */
    public Measure<Temperature> getMaxTemperature() {
        return rInputs.temperature.gt(lInputs.temperature) ? rInputs.temperature : lInputs.temperature;
    }

    public boolean isTemperatureTooHigh() {
        // 90% Buffer.
        return getMaxTemperature()
                .gt(CONSTANTS.getFalcon500MaxTemperature().times(CONSTANTS.getMotorSafeTemperatureBuffer()));
    }

    /**
     * Start the Flywheels with a specific voltage
     * 
     * @param voltage Set Flywheels to this voltage
     */
    public void start(Measure<Voltage> voltage) {
        targetVoltage = voltage;
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

    public Measure<Voltage> getTargetVoltage() {
        return targetVoltage;
    }

    /**
     * Stop the Flywheels
     */
    public void stop() {
        targetVoltage = Volts.zero();
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

    public Command startCommand(Measure<Voltage> voltage) {
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
