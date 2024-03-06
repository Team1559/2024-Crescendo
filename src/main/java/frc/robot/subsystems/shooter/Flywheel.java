package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

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
        public Measure<Voltage> voltsTarget;

        public Measure<Velocity<Angle>> velocityActual = RotationsPerSecond.zero();
        public Measure<Velocity<Angle>> velocityTarget;
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
        flywheelMotorL = new TalonFX(Constants.getFlywheelMotorIdLeft());
        flywheelMotorR = new TalonFX(Constants.getFlywheelMotorIdRight());

        // Only set the Motor Configuration once, to avoid accidentally overriding
        // configs with defaults.
        flywheelMotorL.getConfigurator().apply(Constants.getDefaultTalonFXConfiguration(
                InvertedValue.CounterClockwise_Positive /* default */, NeutralModeValue.Coast));
        flywheelMotorR.getConfigurator().apply(Constants.getDefaultTalonFXConfiguration(
                InvertedValue.Clockwise_Positive /* inverted */, NeutralModeValue.Coast));

        // ---------- Define Loggable Fields ----------
        flywheelLFaults = Constants.getAllGetFaultStatusSignalMethods(flywheelMotorL);
        flywheelRFaults = Constants.getAllGetFaultStatusSignalMethods(flywheelMotorR);

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
        BaseStatusSignal.setUpdateFrequencyForAll(Constants.getPathPlannerLogUpdateFrequencyDefault(),
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
            rInputs.voltsTarget = getTargetVoltage().times(Constants.flywheelSpinOffset());
            flywheelMotorL.setControl(new VoltageOut(rInputs.voltsTarget.in(Volts)));
        }

        // Log Inputs.
        BaseStatusSignal.refreshAll(statusSignalArray);

        lInputs.currentActual = Amps.of(flywheelLTorqueCurrent.getValue());
        rInputs.currentActual = Amps.of(flywheelRTorqueCurrent.getValue());

        lInputs.currentAvailable = Amps.of(flywheelLSupplyCurrent.getValue());
        rInputs.currentAvailable = Amps.of(flywheelRSupplyCurrent.getValue());

        lInputs.faults = Constants.getFaults(flywheelLFaults);
        rInputs.faults = Constants.getFaults(flywheelRFaults);

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

        // Not Currently Supported.
        // lInputs.velocityTarget =
        // rInputs.velocityTarget =

        Logger.processInputs("Shooter/Flywheel/LeftMotor", lInputs);
        Logger.processInputs("Shooter/Flywheel/RightMotor", rInputs);
    }

    // ========================= Functions =====================================
    /** @return The Temperature of the hottest motor. */
    public Measure<Temperature> getMaxTemperature() {
        return rInputs.temperature.gt(lInputs.temperature) ? rInputs.temperature : lInputs.temperature;
    }

    public boolean isTemperatureTooHigh() {
        return getMaxTemperature()
                .gt(Constants.getFalcon500MaxTemperature().times(Constants.getMotorSafeTemperatureBuffer()));
    }

    public void start() {
        start(Constants.getFlywheelForwardVoltage());
    }

    public void start(Measure<Voltage> voltage) {
        start(voltage, null);
    }

    private void start(Measure<Voltage> voltage, Boolean runOneWheelFlag) {
        targetVoltage = voltage;
        this.runOneWheelFlag = runOneWheelFlag;
    }

    public void startOneMotor(boolean runRightWheel) {
        start(Constants.getFlywheelForwardVoltage(), runRightWheel);
    }

    public Measure<Voltage> getTargetVoltage() {
        return targetVoltage;
    }

    public void stop() {
        targetVoltage = Volts.zero();
        flywheelMotorL.stopMotor();
        flywheelMotorR.stopMotor();
    }

    public void reverse() {
        start(Constants.getFlywheelReverseVoltage());
    }

    // ========================= Function Commands =============================

    public Command startCommand() {
        return new InstantCommand(this::start, this);
    }

    public Command startStopCommand() {
        return new StartEndCommand(this::start, this::stop, this);
    }

    public Command startCommand(Measure<Voltage> voltage) {
        return new InstantCommand(() -> start(voltage), this);
    }

    public Command startStopCommand(Measure<Voltage> voltage) {
        return new StartEndCommand(() -> start(voltage), this::stop, this);
    }

    public Command startOneMotorCommand(boolean runRightWheel) {
        return new InstantCommand(() -> startOneMotor(runRightWheel), this);
    }

    public Command startOneMotorStopCommand(boolean runRightWheel) {
        return new StartEndCommand(() -> startOneMotor(runRightWheel), this::stop, this);
    }

    public Command stopCommand() {
        return new InstantCommand(this::stop, this);
    }

    public Command reverseCommand() {
        return new InstantCommand(this::reverse, this);
    }

    public Command reverseStopCommand() {
        return new StartEndCommand(this::reverse, this::stop, this);
    }

    // ========================= Game Commands =================================

    public Command defaultFlywheelCommand() {
        return new SequentialCommandGroup(new WaitCommand(.25), stopCommand());
    }

    public Command spinUpFlywheelCommand() {
        return new SequentialCommandGroup(
                startCommand(),
                new WaitCommand(1) // TODO: Tune.
        );
    }

}
