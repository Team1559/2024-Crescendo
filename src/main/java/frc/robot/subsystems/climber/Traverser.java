package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.io.motor.MotorIo;
import frc.robot.subsystems.abstract_interface.SingleMotorSubsystem;

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
