package frc.robot.io.motor.talon_fx;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.PidValues;
import frc.robot.io.motor.MotorIo;
import frc.robot.util.MathUtils;

public abstract class MotorIoTalonFx implements MotorIo {

    protected final TalonFX motor;
    protected final Rotation2d absoluteEncoderOffset;
    protected final PidValues pidValues;

    private final StatusSignal<Double> deviceTemp;
    private final StatusSignal<Double> dutyCycle;
    private final StatusSignal<Double> motorVoltage;
    private final StatusSignal<Double> position;
    private final StatusSignal<Double> supplyCurrent;
    private final StatusSignal<Double> supplyVoltage;
    private final StatusSignal<Double> torqueCurrent;
    private final StatusSignal<Double> velocity;
    private final Map<String, StatusSignal<Boolean>> faults;
    private final List<StatusSignal<?>> statusSignals = new LinkedList<>();
    private final StatusSignal<?>[] statusSignalArray;

    protected Rotation2d targetPosition;
    protected Measure<Velocity<Angle>> targetVelocity, targetVelocityClamped;
    protected Measure<Voltage> targetVoltage, targetVoltageClamped;

    /**
     * Create a new subsystem for a single SparkMax-controlled motor in voltage mode
     * 
     * @param motorId   Motor CAN ID
     * @param inverted  True if the motor direction should be inverted
     * @param idleMode  What the motor should do when no commands are being sent to
     *                  it.
     * @param pidValues The values to configure the motors built in PID Controller.
     *                  <p>
     *                  (If {@code null}, the the {@link MotorIo#DEFAULT_PID_VALUES}
     *                  will be used.)
     *                  </p>
     */
    public MotorIoTalonFx(int motorId, boolean inverted, NeutralModeValue idleMode, Rotation2d absoluteEncoderOffset,
            PidValues pidValues) {

        // ---------- Create & Configure Motors ----------
        // Configure piD Controller.
        this.pidValues = pidValues = pidValues == null ? DEFAULT_PID_VALUES : pidValues;
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = pidValues.P;
        slot0Configs.kI = pidValues.I;
        slot0Configs.kD = pidValues.D;
        // slot0Configs. = pidValues.FF; // TODO.

        // Only set the Motor Configuration once, to avoid accidentally overriding
        // configs with defaults.
        TalonFXConfiguration talonFXConfiguration = Constants.getDefaultTalonFXConfiguration(
                inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive, idleMode);
        talonFXConfiguration.withSlot0(slot0Configs);

        // Create & Configure Motor.
        motor = new TalonFX(motorId);
        motor.getConfigurator().apply(talonFXConfiguration);

        // Configure Encoder.
        this.absoluteEncoderOffset = absoluteEncoderOffset;

        // ---------- Define Loggable Fields ----------
        faults = getAllGetFaultStatusSignalMethods(motor);
        statusSignals.addAll(faults.values());

        deviceTemp = motor.getDeviceTemp(); // Updated on Call & Log.
        statusSignals.add(dutyCycle = motor.getDutyCycle());
        motorVoltage = motor.getMotorVoltage(); // Updated on Call & Log.
        position = motor.getPosition(); // Updated on Call & Log.
        statusSignals.add(supplyCurrent = motor.getSupplyCurrent());
        statusSignals.add(supplyVoltage = motor.getSupplyVoltage());
        statusSignals.add(torqueCurrent = motor.getTorqueCurrent());
        velocity = motor.getVelocity(); // Updated on Call & Log.

        // ---------- Optimize Bus Utilization ----------
        statusSignalArray = statusSignals.toArray(new StatusSignal[0]);
        BaseStatusSignal.setUpdateFrequencyForAll(Constants.getPathPlannerLogUpdateFrequencyDefault(),
                statusSignalArray);
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(MotorIoInputs inputs) {

        BaseStatusSignal.refreshAll(statusSignalArray);

        inputs.currentActual = Amps.of(torqueCurrent.getValue());
        inputs.currentAvailable = Amps.of(supplyCurrent.getValue());

        inputs.faults = getFaults(faults);

        inputs.temperature = getTemperature();

        inputs.positionAbsolute = getPositionAbsolute();
        inputs.positionRelative = getPositionRelative();

        inputs.powerPercentage = (float) (dutyCycle.getValue() / 2);

        inputs.velocityActual = getVelocity();
        inputs.velocityTarget = targetVelocity;
        inputs.velocityTargetClamped = targetVelocityClamped;

        inputs.voltsAvailable = Volts.of(supplyVoltage.getValue());
        inputs.voltsActual = getVoltage();
        inputs.voltsTarget = targetVoltage;
        inputs.voltsTargetClamped = targetVoltageClamped;
    }

    // ========================= Functions =========================

    @Override
    public Rotation2d getPositionAbsolute() {
        return Rotation2d.fromRotations(position.refresh().getValue()).plus(absoluteEncoderOffset);
    }

    @Override
    public Rotation2d getPositionRelative() {
        return Rotation2d.fromRotations(position.refresh().getValue()).plus(Rotation2d.fromDegrees(0));
    }

    @Override
    public Measure<Temperature> getTemperature() {
        return Celsius.of(deviceTemp.refresh().getValue());
    }

    @Override
    public Measure<Velocity<Angle>> getVelocity() {
        return RotationsPerSecond.of(velocity.refresh().getValue());
    }

    @Override
    public Measure<Voltage> getVoltage() {
        return Volts.of(motorVoltage.refresh().getValue());
    }

    @Override
    public void setPosition(Rotation2d position) {

        targetPosition = position;
        targetVelocity = null;
        targetVelocityClamped = null;
        targetVoltage = null;
        targetVoltageClamped = null;

        motor.setControl(new PositionDutyCycle(position.getRotations()).withFeedForward(pidValues.FF));
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity) {

        targetPosition = null;
        targetVelocity = velocity;
        targetVelocityClamped = MathUtils.clamp(velocity, getMaxSafeVelocity());
        targetVoltage = null;
        targetVoltageClamped = null;

        motor.setControl(
                new VelocityVoltage(targetVelocityClamped.in(RotationsPerSecond)).withFeedForward(pidValues.FF));
    }

    @Override
    public void setVoltage(Measure<Voltage> voltage) {

        targetPosition = null;
        targetVelocity = null;
        targetVelocityClamped = null;
        targetVoltage = voltage;
        targetVoltageClamped = MathUtils.clamp(voltage, getMaxSafeVoltage());

        motor.setControl(new VoltageOut(targetVoltageClamped.in(Volts)));
    }

    @Override
    public void stop() {
        motor.stopMotor();
        targetPosition = null;
        targetVelocity = null;
        targetVoltage = null;
    }

    // ========================= Static Helper Methods =========================

    @SuppressWarnings("unchecked")
    public static Map<String, StatusSignal<Boolean>> getAllGetFaultStatusSignalMethods(TalonFX motor) {
        Map<String, StatusSignal<Boolean>> faults = new HashMap<>();
        Class<?> c = motor.getClass();
        Method[] publicMethods = c.getMethods();
        for (int i = 0; i < publicMethods.length; i++) {
            Method method = publicMethods[i];
            String[] parts = method.toString().split("_");
            if (parts.length == 2 && parts[0].equals("public static StatusSignal<Boolean> getFault")) {
                try {
                    faults.put(parts[1].split("(")[0], (StatusSignal<Boolean>) method.invoke(motor));
                } catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
                    // Ignore.
                    e.printStackTrace();
                }
            }
        }
        return faults;
    }

    public static String[] getFaults(Map<String, StatusSignal<Boolean>> faultStatusSignals) {

        List<String> faults = new LinkedList<>();
        for (Entry<String, StatusSignal<Boolean>> entry : faultStatusSignals.entrySet()) {
            if (entry.getValue().getValue()) {
                faults.add(entry.getKey());
            }
        }
        return faults.toArray(new String[0]);
    }

}
