package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.io.motor.MotorIo;
import frc.robot.io.motor.MotorIoInputsAutoLogged;

public abstract class AbstractSingleMotorSubsystem extends SubsystemBase {

    public static List<AbstractSingleMotorSubsystem> instantiatedSubsystems = Collections
            .synchronizedList(new LinkedList<>());

    protected final Measure<Velocity<Angle>> DEFAULT_FORWARDS_VELOCITY;
    protected final Measure<Velocity<Angle>> DEFAULT_REVERSE_VELOCITY;

    private final MotorIo io;
    private final MotorIoInputsAutoLogged inputs = new MotorIoInputsAutoLogged();

    private Measure<Velocity<Angle>> appliedVelocity;

    /**
     * Create a new subsystem for a single motor in velocity mode
     * 
     * @param name     Name of subsystem
     * @param io       SingleMotorIO instance for the specific motor type
     * @param velocity Will be used for forwards and backwards.
     */
    protected AbstractSingleMotorSubsystem(MotorIo io, Measure<Velocity<Angle>> velocity) {
        this(io, velocity, velocity);
    }

    protected AbstractSingleMotorSubsystem(MotorIo io, Measure<Velocity<Angle>> forwardsVelocity,
            Measure<Velocity<Angle>> reverseVelocity) {

        this.io = io;
        this.DEFAULT_FORWARDS_VELOCITY = forwardsVelocity;
        this.DEFAULT_REVERSE_VELOCITY = reverseVelocity;
        this.appliedVelocity = RevolutionsPerSecond.zero();

        String[] packagePath = getClass().getPackageName().split(".");
        String packageName = packagePath[packagePath.length - 1];
        String name = packageName + "/" + getClass().getName();
        super.setName(name);

        instantiatedSubsystems.add(this);
    }

    @Override
    public void periodic() {

        // Set Velocity.
        io.setVelocity(appliedVelocity);

        // Log Inputs.
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }

    // ========================= Functions =========================
    public boolean isTemperatureTooHigh() {
        return io.getTemperature().gt(io.getMaxSafeTemperature().times(Constants.getMotorSafeTemperatureBuffer()));
    }

    public void reverse() {
        setVelocity(DEFAULT_REVERSE_VELOCITY);
    }

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        appliedVelocity = velocity;
    }

    public void start() {
        setVelocity(DEFAULT_FORWARDS_VELOCITY);
    }

    public void stop() {
        io.stop();
    }

    // ========================= Commands =========================

    public Command reverseCommand() {
        return new InstantCommand(this::reverse, this);
    }

    public Command reverseStopCommand() {
        return new StartEndCommand(this::reverse, this::stop, this);
    }

    public Command setVelocityCommand(Measure<Velocity<Angle>> velocity) {
        return new InstantCommand(() -> setVelocity(velocity), this);
    }

    public Command startCommand() {
        return new InstantCommand(this::start, this);
    }

    public Command startStopCommand() {
        return new StartEndCommand(this::start, this::stop, this);
    }

    public Command stopCommand() {
        return new InstantCommand(this::stop, this);
    }

    public Command waitUntilTemperatureIsNotTooHighCommand() {
        Command command = new WaitUntilCommand(this::isTemperatureTooHigh);
        command.addRequirements(this);
        return command;
    }
}
