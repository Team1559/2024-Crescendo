package frc.robot.subsystems.abstract_interface;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.BetterLogger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.motor.MotorIo;
import frc.robot.io.motor.MotorIoInputsAutoLogged;
import frc.robot.util.MathUtils;

public class SingleMotorSubsystem extends SubsystemBase implements MotorSubsystem {

    @AutoLog
    static class SingleMotorSubsystemInputs {

        public Measure<Velocity<Angle>> targetVelocity, targetVelocityClamped;
        public Measure<Voltage> targetVoltage, targetVoltageClamped;
        public Rotation2d targetPosition, targetPositionClamped;
    }

    public static List<SingleMotorSubsystem> instantiatedSubsystems = Collections
            .synchronizedList(new LinkedList<>());

    public final MotorIo motorIo;
    private final MotorIoInputsAutoLogged ioInputs = new MotorIoInputsAutoLogged();

    private final Measure<Velocity<Angle>> defaultForwardVelocity, defaultReverseVelocity;
    private final Measure<Voltage> defaultForwardVoltage, defaultReverseVoltage;

    protected final SingleMotorSubsystemInputsAutoLogged subsystemInputs = new SingleMotorSubsystemInputsAutoLogged();

    private boolean isRunning = false;

    private Measure<Velocity<Angle>> maximumVelocity;
    private Measure<Voltage> maximumVoltage;
    private Rotation2d maximumPosition, minimumPosition;

    // ========================= Constructors & Config =========================

    /**
     * Constricts a Single Motor Subsystem.
     * 
     * @param motorIo                The Motor that belongs to this
     *                               {@link Subsystem}.
     *                               <p>
     *                               (<i>Note:</i> It should not be in any other
     *                               {@link Subsystem}.)
     *                               </p>
     * @param defaultForwardVelocity The velocity that will be used by the
     *                               {@link #forward()} method.
     * @param defaultReverseVelocity The velocity that will be used by the
     *                               {@link #reverse()} method.
     * @param velocities             <i>Ignore.</i> (Used to allow constructor
     *                               overloading with generics.)
     */
    protected SingleMotorSubsystem(MotorIo motorIo, Measure<Velocity<Angle>> defaultForwardVelocity,
            Measure<Velocity<Angle>> defaultReverseVelocity, Velocity<?>... velocities) {
        this.motorIo = motorIo;
        this.defaultForwardVelocity = defaultForwardVelocity;
        this.defaultReverseVelocity = defaultReverseVelocity;
        this.defaultForwardVoltage = null;
        this.defaultReverseVoltage = null;
        config();
    }

    /**
     * Constricts a Single Motor Subsystem.
     * 
     * @param motorIo               The Motor that belongs to this
     *                              {@link Subsystem}.
     *                              <p>
     *                              (<i>Note:</i> It should not be in any other
     *                              {@link Subsystem}.)
     *                              </p>
     * @param defaultForwardVoltage The voltage that will be used by the
     *                              {@link #forward()} method.
     * @param defaultReverseVoltage The voltage that will be used by the
     *                              {@link #reverse()} method.
     *                              or {@code null} if there is no limit.
     * @param velocities            <i>Ignore.</i> (Used to allow constructor
     *                              overloading with generics.)
     */
    protected SingleMotorSubsystem(MotorIo motorIo, Measure<Voltage> defaultForwardVoltage,
            Measure<Voltage> defaultReverseVoltage, Voltage... voltages) {
        this.motorIo = motorIo;
        this.defaultForwardVelocity = null;
        this.defaultReverseVelocity = null;
        this.defaultForwardVoltage = defaultForwardVoltage;
        this.defaultReverseVoltage = defaultReverseVoltage;
        config();
    }

    /**
     * Constricts a Single Motor Subsystem with no default velocities or voltages.
     * <p>
     * This means that the {@link #forward()} and {@link #reverse()} methods will do
     * nothing.
     * </p>
     * 
     * @param motorIo The Motor that belongs to this
     *                {@link Subsystem}.
     *                <p>
     *                (<i>Note:</i> It should not be in any other
     *                {@link Subsystem}.)
     *                </p>
     */
    protected SingleMotorSubsystem(MotorIo motorIo) {
        this.motorIo = motorIo;
        this.defaultForwardVelocity = null;
        this.defaultReverseVelocity = null;
        this.defaultForwardVoltage = null;
        this.defaultReverseVoltage = null;
        config();
    }

    private void config() {

        // Set Name.
        super.setName(MotorSubsystem.getSubsystemName(this.getClass()));

        // Add to Collection.
        instantiatedSubsystems.add(this);
        MotorSubsystem.instantiatedSubsystems.add(this);
    }

    @Override
    public void setMaxVelocity(Measure<Velocity<Angle>> maxVelocity) {
        this.maximumVelocity = maxVelocity;
    }

    @Override
    public void setMaxVoltage(Measure<Voltage> maxVoltage) {
        this.maximumVoltage = maxVoltage;
    }

    @Override
    public void setMinAndMaxPositions(Rotation2d minPosition, Rotation2d maxPosition) {
        this.minimumPosition = minPosition;
        this.maximumPosition = maxPosition;
    }

    // ========================= Periodic ======================================

    @Override
    public void periodic() {

        // Set Velocity.
        // Have to tell the motor to run repeatedly, or the Safety Protocols will stop
        // it. See:
        // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#motor-safety
        if (subsystemInputs.targetPositionClamped != null) {
            motorIo.setPosition(subsystemInputs.targetPositionClamped);
            isRunning = true;
        } else if (subsystemInputs.targetVelocityClamped != null) {
            motorIo.setVelocity(subsystemInputs.targetVelocityClamped);
            isRunning = true;
        } else if (subsystemInputs.targetVoltageClamped != null) {
            motorIo.setVoltage(subsystemInputs.targetVoltageClamped);
            isRunning = true;
        } else if (isRunning) { // Avoids setting up a callback for stop.
            motorIo.stop();
            isRunning = false;
        }

        // Log Subsystem Inputs.
        BetterLogger.processInputs(getName(), subsystemInputs);

        // Log Motor Inputs.
        motorIo.updateInputs(ioInputs);
        BetterLogger.processInputs(getName(), ioInputs);
    }

    // ========================= Functions =====================================

    // -------------------- Default Actions --------------------

    @Override
    public void forward() {
        if (subsystemInputs.targetVelocity != null) {
            setVelocity(defaultForwardVelocity);
        } else if (subsystemInputs.targetVoltage != null) {
            setVoltage(defaultForwardVoltage);
        } else {
            motorIo.stop();
        }
    }

    @Override
    public void forwardMaxVelocity() {
        setVelocity(motorIo.getMaxSafeVelocity());
    }

    @Override
    public void reverse() {
        if (subsystemInputs.targetVelocity != null) {
            setVelocity(defaultReverseVelocity);
        } else if (subsystemInputs.targetVoltage != null) {
            setVoltage(defaultReverseVoltage);
        } else {
            motorIo.stop();
        }
    }

    @Override
    public void reverseMaxVelocity() {
        setVelocity(motorIo.getMaxSafeVelocity().negate());
    }

    @Override
    public void stop() {
        subsystemInputs.targetPosition = null;
        subsystemInputs.targetVelocity = null;
        subsystemInputs.targetVoltage = null;
        motorIo.stop();
    }

    // -------------------- Getters --------------------

    @Override
    public Rotation2d getCurrentAbsolutePosition() {
        return motorIo.getPositionAbsolute();
    }

    @Override
    public Rotation2d getCurrentRelativePosition() {
        return motorIo.getPositionRelative();
    }

    @Override
    public Measure<Velocity<Angle>> getCurrentVelocity() {
        return motorIo.getVelocity();
    }

    @Override
    public Measure<Voltage> getCurrentVoltage() {
        return motorIo.getVoltage();
    }

    @Override
    public Rotation2d getTargetPosition() {
        return subsystemInputs.targetPosition;
    }

    @Override
    public Measure<Velocity<Angle>> getTargetVelocity() {
        return subsystemInputs.targetVelocity;
    }

    @Override
    public Measure<Voltage> getTargetVoltage() {
        return subsystemInputs.targetVoltage;
    }

    @Override
    public boolean isTemperatureTooHigh() {
        return motorIo.getTemperature()
                .gt(motorIo.getMaxSafeTemperature().times(Constants.getMotorSafeTemperatureBuffer()));
    }

    // -------------------- Setters --------------------

    @Override
    public void setPosition(Rotation2d position) {

        subsystemInputs.targetVelocity = null;
        subsystemInputs.targetVelocityClamped = null;

        subsystemInputs.targetVoltage = null;
        subsystemInputs.targetVoltageClamped = null;

        subsystemInputs.targetPosition = position;
        subsystemInputs.targetPositionClamped = MathUtils.clamp(position, minimumPosition, maximumPosition);
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity) {

        subsystemInputs.targetVelocity = velocity;
        subsystemInputs.targetVelocityClamped = MathUtils.clamp(velocity, maximumVelocity);

        subsystemInputs.targetVoltage = null;
        subsystemInputs.targetVoltageClamped = null;

        subsystemInputs.targetPosition = null;
        subsystemInputs.targetPositionClamped = null;
    }

    @Override
    public void setVoltage(Measure<Voltage> voltage) {

        subsystemInputs.targetVelocity = null;
        subsystemInputs.targetVelocityClamped = null;

        subsystemInputs.targetVoltage = voltage;
        subsystemInputs.targetVoltageClamped = MathUtils.clamp(voltage, maximumVoltage);

        subsystemInputs.targetPosition = null;
        subsystemInputs.targetPositionClamped = null;
    }
}
