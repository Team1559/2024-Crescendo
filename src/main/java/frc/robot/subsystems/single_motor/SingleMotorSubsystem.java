package frc.robot.subsystems.single_motor;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static frc.robot.constants.AbstractConstants.CONSTANTS;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleMotorSubsystem extends SubsystemBase {

    protected final Measure<Velocity<Angle>> DEFAULT_FORWARDS_VELOCITY;
    protected final Measure<Velocity<Angle>> DEFAULT_REVERSE_VELOCITY;

    private final SingleMotorIo io;
    private final SingleMotorIoInputsAutoLogged inputs = new SingleMotorIoInputsAutoLogged();

    private Measure<Velocity<Angle>> appliedVelocity;

    /**
     * Create a new subsystem for a single motor in velocity mode
     * 
     * @param name     Name of subsystem
     * @param io       SingleMotorIO instance for the specific motor type
     * @param velocity Will be used for forwards and backwards.
     */
    protected SingleMotorSubsystem(String name, SingleMotorIo io, Measure<Velocity<Angle>> velocity) {
        this(name, io, velocity, velocity);

    }

    protected SingleMotorSubsystem(String name, SingleMotorIo io, Measure<Velocity<Angle>> forwardsVelocity,
            Measure<Velocity<Angle>> reverseVelocity) {

        super(name);

        this.io = io;
        this.DEFAULT_FORWARDS_VELOCITY = forwardsVelocity;
        this.DEFAULT_REVERSE_VELOCITY = reverseVelocity;
        this.appliedVelocity = RevolutionsPerSecond.zero();
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
        // 90% Buffer.
        return io.getTemperature().gt(io.getMaxSafeTemperature().times(CONSTANTS.getMotorSafeTemperatureBuffer()));
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
        setVelocity(RevolutionsPerSecond.zero());
    }

    // ========================= Commands =========================

    public Command startCommand() {
        return new InstantCommand(this::start, this);
    }

    public Command stopCommand() {
        return new InstantCommand(this::stop, this);
    }

    public Command reverseCommand() {
        return new InstantCommand(this::reverse, this);
    }
}
