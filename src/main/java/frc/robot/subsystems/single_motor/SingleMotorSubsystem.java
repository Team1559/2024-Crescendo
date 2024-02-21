package frc.robot.subsystems.single_motor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleMotorSubsystem extends SubsystemBase {

    private final double DEFAULT_FORWARDS_VOLTAGE;
    private final double DEFAULT_REVERSE_VOLTAGE;

    private final SingleMotorIo io;
    private final SingleMotorIoInputsAutoLogged inputs = new SingleMotorIoInputsAutoLogged();

    private double appliedVoltage;

    /**
     * Create a new subsystem for a single motor in voltage mode
     * 
     * @param name Name of subsystem
     * @param io   SingleMotorIO instance for the specific motor type
     */
    protected SingleMotorSubsystem(String name, SingleMotorIo io, double voltage) {
        this(name, io, voltage, voltage);
    }

    protected SingleMotorSubsystem(String name, SingleMotorIo io, double forwardsVoltage, double reverseVoltage) {

        super(name);

        this.io = io;
        this.DEFAULT_FORWARDS_VOLTAGE = forwardsVoltage;
        this.DEFAULT_REVERSE_VOLTAGE = reverseVoltage;

        this.appliedVoltage = 0;
    }

    @Override
    public void periodic() {

        // Set Voltages.
        io.setVoltage(appliedVoltage);

        // Log Inputs.
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }

    // ========================= Functions =========================
    public boolean isTemperatureTooHigh() {
        // 90% Buffer.
        return io.getTemperature().gt(io.getMaxSafeTemperature().times(0.9));
    }

    public void reverse() {
        setVoltage(DEFAULT_REVERSE_VOLTAGE);
    }

    private void setVoltage(double voltage) {
        appliedVoltage = voltage;
    }

    public void start() {
        setVoltage(DEFAULT_FORWARDS_VOLTAGE);
    }

    public void stop() {
        setVoltage(0.0);
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
