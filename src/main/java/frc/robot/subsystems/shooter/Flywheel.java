package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.AbstractConstants.CONSTANTS;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

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
import frc.robot.subsystems.swerve_module.SwerveModuleIoTalonFx;

public class Flywheel extends SubsystemBase {

    @AutoLog
    static class FlywheelInputs {

        public Measure<Current> supplyCurrent = Amps.zero();

        public Measure<Temperature> motorTemp = Celsius.zero();

        public Measure<Voltage> voltsActual = Volts.zero();
        public Measure<Voltage> voltsAvailable = Volts.zero();
        public Measure<Voltage> voltsTarget = Volts.zero();

        public Measure<Velocity<Angle>> velocityActual = RotationsPerSecond.zero();
        public Measure<Velocity<Angle>> velocityTarget = RotationsPerSecond.zero();

        public Rotation2d positionAbsolute = new Rotation2d();

        public String[] faults = new String[0];
    }

    private final StatusSignal<Double> flywheelLMotorTemp, flywheelRMotorTemp;
    private final StatusSignal<Double> flywheelLMotorVoltage, flywheelRMotorVoltage;
    private final StatusSignal<Double> flywheelLPosition, flywheelRPosition;
    private final StatusSignal<Double> flywheelLSupplyCurrent, flywheelRSupplyCurrent;
    private final StatusSignal<Double> flywheelLSupplyVoltage, flywheelRSupplyVoltage;
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
        flywheelMotorL.getConfigurator().apply(SwerveModuleIoTalonFx.getDefaultTalonFXConfiguration(
                InvertedValue.CounterClockwise_Positive /* default */, NeutralModeValue.Coast));
        flywheelMotorR.getConfigurator().apply(SwerveModuleIoTalonFx.getDefaultTalonFXConfiguration(
                InvertedValue.Clockwise_Positive /* inverted */, NeutralModeValue.Coast));

        // ---------- Define Loggable Fields ----------
        statusSignals.add(flywheelLMotorVoltage = flywheelMotorL.getMotorVoltage());
        statusSignals.add(flywheelRMotorVoltage = flywheelMotorR.getMotorVoltage());

        statusSignals.add(flywheelLSupplyCurrent = flywheelMotorL.getSupplyCurrent());
        statusSignals.add(flywheelRSupplyCurrent = flywheelMotorR.getSupplyCurrent());

        statusSignals.add(flywheelLSupplyVoltage = flywheelMotorL.getSupplyVoltage());
        statusSignals.add(flywheelRSupplyVoltage = flywheelMotorR.getSupplyVoltage());

        statusSignals.add(flywheelLVelocity = flywheelMotorL.getVelocity());
        statusSignals.add(flywheelRVelocity = flywheelMotorR.getVelocity());

        statusSignals.add(flywheelLMotorTemp = flywheelMotorL.getDeviceTemp());
        statusSignals.add(flywheelRMotorTemp = flywheelMotorR.getDeviceTemp());

        statusSignals.add(flywheelLPosition = flywheelMotorL.getPosition());
        statusSignals.add(flywheelRPosition = flywheelMotorR.getPosition());

        flywheelLFaults = new HashMap<>();
        flywheelRFaults = new HashMap<>();

        addAllGetFaultStatusSignalMethods(flywheelLFaults, flywheelMotorL);
        addAllGetFaultStatusSignalMethods(flywheelRFaults, flywheelMotorR);

        statusSignals.addAll(flywheelLFaults.values());
        statusSignals.addAll(flywheelRFaults.values());

        // ---------- Optimize Bus Utilization ----------
        statusSignalArray = statusSignals.toArray(new StatusSignal[0]);
        BaseStatusSignal.setUpdateFrequencyForAll(CONSTANTS.getPathPlannerLogUpdateFrequencyDefault(),
                statusSignalArray);
        flywheelMotorL.optimizeBusUtilization();
        flywheelMotorR.optimizeBusUtilization();
    }

    @SuppressWarnings("unchecked")
    private void addAllGetFaultStatusSignalMethods(Map<String, StatusSignal<Boolean>> faults, TalonFX motor) {
        Class<?> c = motor.getClass();
        Method[] publicMethods = c.getMethods();
        for (int i = 0; i < publicMethods.length; i++) {
            Method method = publicMethods[i];
            String[] parts = method.toString().split("_");
            if (parts.length == 2 && parts[0].equals("public StatusSignal<Boolean> getFault")) {
                try {
                    faults.put(parts[1].split("(")[0], (StatusSignal<Boolean>) method.invoke(motor));
                } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
                    // Ignore.
                    e.printStackTrace();
                }
            }
        }
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

        updateFaultInputs(lInputs, flywheelMotorL, flywheelLFaults);
        updateFaultInputs(rInputs, flywheelMotorR, flywheelRFaults);

        lInputs.motorTemp = Celsius.of(flywheelLMotorTemp.getValue());
        rInputs.motorTemp = Celsius.of(flywheelRMotorTemp.getValue());

        lInputs.voltsActual = Volts.of(flywheelLMotorVoltage.getValue());
        rInputs.voltsActual = Volts.of(flywheelRMotorVoltage.getValue());

        lInputs.positionAbsolute = Rotation2d.fromRotations(flywheelLPosition.getValue());
        rInputs.positionAbsolute = Rotation2d.fromRotations(flywheelRPosition.getValue());

        lInputs.supplyCurrent = Amps.of(flywheelLSupplyCurrent.getValue());
        rInputs.supplyCurrent = Amps.of(flywheelRSupplyCurrent.getValue());

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

    private void updateFaultInputs(FlywheelInputsAutoLogged inputs, TalonFX motor,
            Map<String, StatusSignal<Boolean>> faultsMap) {

        List<String> faults = new LinkedList<>();
        for (Entry<String, StatusSignal<Boolean>> entry : faultsMap.entrySet()) {
            if (entry.getValue().getValue()) {
                faults.add(entry.getKey());
            }
        }
        inputs.faults = faults.toArray(new String[0]);
    }

    // ========================= Functions =========================
    /**
     * @return The Temperature of the hottest motor.
     */
    public Measure<Temperature> getMaxTemperature() {
        return rInputs.motorTemp.gt(lInputs.motorTemp) ? rInputs.motorTemp : lInputs.motorTemp;
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
