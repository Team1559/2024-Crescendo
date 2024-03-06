package frc.robot.io.swerve_module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Voltage;
/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn
 * motor controller, and CANcoder.
 * <p>
 * NOTE: This implementation should be used as a starting point and adapted to
 * different hardware configurations (e.g. If using an analog encoder, copy from
 * "ModuleIOSparkMax").
 * </p>
 * <p>
 * To calibrate the absolute encoder offsets, point the modules straight (such
 * that forward motion on the drive motor will propel the robot forward) and
 * copy the reported values from the absolute encoders using AdvantageScope.
 * These values are logged under "/Drive/ModuleX/TurnAbsolutePositionRad".
 * </p>
 */
import frc.robot.Constants;
import frc.robot.subsystems.base.SwerveModule.WheelModuleIndex;

public class SwerveModuleIoTalonFx implements SwerveModuleIo {

    private final CANcoder cancoder;

    // Update to use MotorIo instead.
    private final TalonFX driveMotor, steerMotor;

    private final StatusSignal<Double> cancoderAbsolutePosition;
    private final StatusSignal<Double> driveMotorAppliedVolts, steerMotorAppliedVolts;
    private final StatusSignal<Double> driveMotorDutyCycle, steerMotorDutyCycle;
    private final Map<String, StatusSignal<Boolean>> driveMotorFaults, steerMotorFaults;
    private final StatusSignal<Double> driveMotorPosition, steerMotorPosition;
    private final StatusSignal<Double> driveMotorSupplyCurrent, steerMotorSupplyCurrent;
    private final StatusSignal<Double> driveMotorSupplyVoltage, steerMotorSupplyVoltage;
    private final StatusSignal<Double> driveMotorTemp, steerMotorTemp;
    private final StatusSignal<Double> driveMotorTorqueCurrent, steerMotorTorqueCurrent;
    private final StatusSignal<Double> driveMotorVelocity, steerMotorVelocity;

    private final List<StatusSignal<?>> statusSignals = new LinkedList<>();
    private final StatusSignal<?>[] statusSignalArray;

    private Measure<Voltage> driveVoltsTarget = Volts.zero();
    private Measure<Voltage> steerVoltsTarget = Volts.zero();

    private final Rotation2d absoluteEncoderOffset;

    private final WheelModuleIndex wheelModuleIndex;

    public SwerveModuleIoTalonFx(WheelModuleIndex index) {

        wheelModuleIndex = index;

        absoluteEncoderOffset = Constants.getSwerveModuleEncoderOffsets().get(index);

        // ---------- Instantiate Hardware ----------
        cancoder = new CANcoder(Constants.getSwerveModuleHardwareIds().get(index).CANCODER_ID,
                Constants.getCanivoreId());
        driveMotor = new TalonFX(Constants.getSwerveModuleHardwareIds().get(index).DRIVE_MOTOR_ID,
                Constants.getCanivoreId());
        steerMotor = new TalonFX(Constants.getSwerveModuleHardwareIds().get(index).STEER_MOTOR_ID,
                Constants.getCanivoreId());

        // ---------- Configure Hardware ----------
        // ----- Cancoder -----
        // Set CAN Coder configs to defaults.
        cancoder.getConfigurator().apply(new CANcoderConfiguration());

        // ----- Motors -----
        // Only set the Motor Configuration once, to avoid accidentally overriding
        // configs with defaults.
        driveMotor.getConfigurator()
                // Inverted to match our Swerve Drive Module Gear Box & Motors.
                .apply(Constants.getDefaultTalonFXConfiguration(InvertedValue.Clockwise_Positive,
                        NeutralModeValue.Brake));
        steerMotor.getConfigurator()
                .apply(Constants.getDefaultTalonFXConfiguration(InvertedValue.CounterClockwise_Positive,
                        NeutralModeValue.Brake));

        // ---------- Get StatusSignals ----------
        statusSignals.add(cancoderAbsolutePosition = cancoder.getAbsolutePosition());

        statusSignals.add(driveMotorAppliedVolts = driveMotor.getMotorVoltage());
        statusSignals.add(steerMotorAppliedVolts = steerMotor.getMotorVoltage());

        statusSignals.add(driveMotorDutyCycle = driveMotor.getDutyCycle());
        statusSignals.add(steerMotorDutyCycle = steerMotor.getDutyCycle());

        driveMotorFaults = Constants.getAllGetFaultStatusSignalMethods(driveMotor);
        steerMotorFaults = Constants.getAllGetFaultStatusSignalMethods(steerMotor);

        statusSignals.addAll(driveMotorFaults.values());
        statusSignals.addAll(steerMotorFaults.values());

        driveMotorPosition = driveMotor.getPosition();
        steerMotorPosition = steerMotor.getPosition();

        statusSignals.add(driveMotorTemp = driveMotor.getDeviceTemp());
        statusSignals.add(steerMotorTemp = steerMotor.getDeviceTemp());

        statusSignals.add(driveMotorSupplyCurrent = driveMotor.getSupplyCurrent());
        statusSignals.add(steerMotorSupplyCurrent = steerMotor.getSupplyCurrent());

        statusSignals.add(driveMotorSupplyVoltage = driveMotor.getSupplyVoltage());
        statusSignals.add(steerMotorSupplyVoltage = steerMotor.getSupplyVoltage());

        statusSignals.add(driveMotorTorqueCurrent = driveMotor.getTorqueCurrent());
        statusSignals.add(steerMotorTorqueCurrent = steerMotor.getTorqueCurrent());

        statusSignals.add(driveMotorVelocity = driveMotor.getVelocity());
        statusSignals.add(steerMotorVelocity = steerMotor.getVelocity());

        // ---------- Set Update Frequency ----------
        BaseStatusSignal.setUpdateFrequencyForAll(Constants.getPathPlannerLogUpdateFrequencyDefault(),
                statusSignals.toArray(new StatusSignal[0]));

        // Required for odometry, use faster rate
        BaseStatusSignal.setUpdateFrequencyForAll(Constants.getPathPlannerLogFrequencyForOdometry(),
                driveMotorPosition, steerMotorPosition);
        statusSignals.add(driveMotorPosition);
        statusSignals.add(steerMotorPosition);
        statusSignalArray = statusSignals.toArray(new StatusSignal[0]);

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(SwerveModuleIoInputs inputs) {

        BaseStatusSignal.refreshAll(statusSignalArray);

        // ---------- CANCoder ----------
        inputs.cancoderAbsolutePosition = Rotation2d.fromRotations(cancoderAbsolutePosition.getValueAsDouble());
        inputs.cancoderOffsetPosition = inputs.cancoderAbsolutePosition.minus(absoluteEncoderOffset);

        // ---------- Drive Motor ----------
        inputs.driveMotorCurrentActual = Amps.of(driveMotorTorqueCurrent.getValue());
        inputs.driveMotorCurrentAvailable = Amps.of(driveMotorSupplyCurrent.getValue());
        inputs.driveMotorFaults = Constants.getFaults(driveMotorFaults);
        // Inverted so autonomous sees the robot moving in the correct direction.
        inputs.driveMotorPositionAbsolute = Rotation2d
                .fromRadians(Units.rotationsToRadians(-driveMotorPosition.getValueAsDouble())
                        / Constants.getGearRatioOfDriveWheel());
        /** From -1 to 1. */
        inputs.driveMotorPowerPercentage = (float) (driveMotorDutyCycle.getValue() / 2);
        inputs.driveMotorTemp = Celsius.of(driveMotorTemp.getValue());
        inputs.driveMotorVoltsActual = Volts.of(driveMotorAppliedVolts.getValue());
        inputs.driveMotorVoltsTarget = driveVoltsTarget;
        inputs.driveMotorVelocityActual = RadiansPerSecond.of(
                Units.rotationsToRadians(driveMotorVelocity.getValueAsDouble()) / Constants.getGearRatioOfDriveWheel());
        inputs.driveMotorVoltsAvailable = Volts.of(driveMotorSupplyVoltage.getValue());
        // Not Currently Supported. (TODO)
        // inputs.driveMotorVelocityTarget = RotationsPerSecond.zero();

        // ---------- Steer Motor ----------
        inputs.steerMotorCurrentActual = Amps.of(steerMotorTorqueCurrent.getValue());
        inputs.driveMotorCurrentAvailable = Amps.of(steerMotorSupplyCurrent.getValue());
        inputs.steerMotorFaults = Constants.getFaults(steerMotorFaults);
        inputs.steerMotorPositionAbsolute = Rotation2d
                .fromRotations(steerMotorPosition.getValueAsDouble() / Constants.getGearRatioOfTurnWheel())
                .plus(Rotation2d.fromRadians(0));
        /** From -1 to 1. */
        inputs.steerMotorPowerPercentage = (float) (steerMotorDutyCycle.getValue() / 2);
        inputs.steerMotorTemp = Celsius.of(steerMotorTemp.getValue());
        inputs.steerMotorVoltsActual = Volts.of(steerMotorAppliedVolts.getValue());
        inputs.steerMotorVoltsTarget = steerVoltsTarget;
        inputs.steerMotorVelocityActual = RadiansPerSecond.of(
                Units.rotationsToRadians(steerMotorVelocity.getValueAsDouble()) / Constants.getGearRatioOfTurnWheel());
        inputs.driveMotorVoltsAvailable = Volts.of(steerMotorSupplyVoltage.getValue());
        // Not Currently Supported. (TODO)
        // inputs.steerMotorVelocityTarget = RotationsPerSecond.zero();
    }

    @Override
    public SwerveModuleIoTalonFx clone() {
        return new SwerveModuleIoTalonFx(wheelModuleIndex);
    }

    // ========================= Functions =========================

    @Override
    public Measure<Temperature> getMaxSafeMotorTemperature() {
        return Constants.getFalcon500MaxTemperature();
    }

    @Override
    public void setDriveVoltage(Measure<Voltage> volts) {
        driveMotor.setControl(new VoltageOut(volts.in(Volts)));
        driveVoltsTarget = volts;
    }

    @Override
    public void setTurnVoltage(Measure<Voltage> volts) {
        steerMotor.setControl(new VoltageOut(volts.in(Volts)));
        steerVoltsTarget = volts;
    }
}