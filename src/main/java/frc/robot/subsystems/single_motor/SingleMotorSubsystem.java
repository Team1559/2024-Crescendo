package frc.robot.subsystems.single_motor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleMotorSubsystem extends SubsystemBase {
    private final double forwardsVoltage;
    private final double reverseVoltage;
    private double appliedVoltage;

    private final SingleMotorIo io;
    private final SingleMotorIoInputsAutoLogged inputs = new SingleMotorIoInputsAutoLogged();

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
        this.forwardsVoltage = forwardsVoltage;
        this.reverseVoltage = reverseVoltage;
        this.appliedVoltage = 0;
    }

    @Override
    public void periodic() {
        io.setVoltage(appliedVoltage);
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }

    // ========================= Functions =========================
    public void start() {
        setVoltage(forwardsVoltage);
    }

    public void stop() {
        setVoltage(0.0);
    }

    public void reverse() {
        setVoltage(reverseVoltage);
    }

    private void setVoltage(double voltage) {
        appliedVoltage = voltage;
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
