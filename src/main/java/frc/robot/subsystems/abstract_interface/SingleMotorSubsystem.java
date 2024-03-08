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

public abstract class SingleMotorSubsystem extends SubsystemBase implements MotorSubsystem {

    @AutoLog
    static class SingleMotorSubsystemInputs {

        public Measure<Velocity<Angle>> targetVelocity;
        public Measure<Voltage> targetVoltage;
        public Rotation2d targetPosition;
    }

    public static List<SingleMotorSubsystem> instantiatedSubsystems = Collections
            .synchronizedList(new LinkedList<>());

    private final MotorIo motorIo;
    private final MotorIoInputsAutoLogged ioInputs = new MotorIoInputsAutoLogged();

    private final Measure<Velocity<Angle>> defaultForwardVelocity, defaultReverseVelocity;
    private final Measure<Voltage> defaultForwardVoltage, defaultReverseVoltage;

    private final SingleMotorSubsystemInputsAutoLogged subsystemInputs = new SingleMotorSubsystemInputsAutoLogged();

    private boolean isRunning = false;

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

    private void config() {

        // Set Name.
        String[] packagePath = getClass().getPackageName().split(".");
        String packageName = packagePath[packagePath.length - 1];
        String name = packageName + "/" + getClass().getName();
        super.setName(name);

        // Add to Collection.
        instantiatedSubsystems.add(this);
    }

    // ========================= Periodic ======================================

    @Override
    public void periodic() {

        // Set Velocity.
        // Have to tell the motor to run repeatedly, or the Safety Protocols will stop
        // it. See:
        // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#motor-safety
        if (subsystemInputs.targetPosition != null) {
            motorIo.setPosition(subsystemInputs.targetPosition);
            isRunning = true;
        } else if (subsystemInputs.targetVelocity != null) {
            motorIo.setVelocity(subsystemInputs.targetVelocity);
            isRunning = true;
        } else if (subsystemInputs.targetVoltage != null) {
            motorIo.setVoltage(subsystemInputs.targetVoltage);
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
    public void stop() {
        subsystemInputs.targetPosition = null;
        subsystemInputs.targetVelocity = null;
        subsystemInputs.targetVoltage = null;
        motorIo.stop();
    }

    // -------------------- Getters --------------------

    @Override
    public Rotation2d getCurrentAbsolutePosition() {
        return motorIo.getAbsolutePosition();
    }

    @Override
    public Rotation2d getCurrentRelativePosition() {
        return motorIo.getRelativePosition();
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
    public boolean isTemperatureTooHigh() {
        return motorIo.getTemperature()
                .gt(motorIo.getMaxSafeTemperature().times(Constants.getMotorSafeTemperatureBuffer()));
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

    // -------------------- Setters --------------------

    @Override
    public void setPosition(Rotation2d position) {
        subsystemInputs.targetVelocity = null;
        subsystemInputs.targetVoltage = null;
        subsystemInputs.targetPosition = position;
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        subsystemInputs.targetPosition = null;
        subsystemInputs.targetVoltage = null;
        subsystemInputs.targetVelocity = velocity;
    }

    @Override
    public void setVoltage(Measure<Voltage> voltage) {
        subsystemInputs.targetPosition = null;
        subsystemInputs.targetVelocity = null;
        subsystemInputs.targetVoltage = voltage;
    }
}