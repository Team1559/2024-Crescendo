package frc.robot.subsystems.climber;

import static frc.robot.constants.AbstractConstants.CONSTANTS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.io.motor.MotorIo;
import frc.robot.subsystems.single_motor.SingleMotorSubsystem;

public class Traverser extends SingleMotorSubsystem {

    public Traverser(MotorIo io) {
        super("Traverser", io, CONSTANTS.getTraverserVelocity(), CONSTANTS.getTraverserVelocity());
    }

    // ========================= Functions =========================
    public void traverserLeft() {
        setVelocity(DEFAULT_FORWARDS_VELOCITY);
    }

    public void traverserRight() {
        setVelocity(DEFAULT_FORWARDS_VELOCITY.negate());
    }

    // ========================= Commands =========================
    public Command traverserLeftStartStopCommand() {
        return new StartEndCommand(this::traverserLeft, this::stop, this);
    }

    public Command traverserRightStartStopCommand() {
        return new StartEndCommand(this::traverserRight, this::stop, this);
    }
}
