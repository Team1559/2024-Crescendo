package org.victorrobotics.frc.subsystems.climber;

import org.victorrobotics.frc.Constants;
import org.victorrobotics.frc.io.motor.MotorIo;
import org.victorrobotics.frc.subsystems.abstract_interface.SingleMotorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class Traverser extends SingleMotorSubsystem {

    public Traverser(MotorIo io) {
        super(io, Constants.getTraverserVelocity(), Constants.getTraverserVelocity().negate());
    }

    // ========================= Functions =========================
    public void traverserLeft() {
        forward();
    }

    public void traverserRight() {
        reverse();
    }

    // ========================= Commands =========================
    public Command traverserLeftThenStopCommand() {
        return forwardThenStopCommand();
    }

    public Command traverserRightThenStopCommand() {
        return reverseThenStopCommand();
    }
}
