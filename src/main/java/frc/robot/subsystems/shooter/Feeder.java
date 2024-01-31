package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
    private CANSparkMax feedMotorL = new CANSparkMax(Constants.LEFT_FEED_MOTOR_ID, MotorType.kBrushless);
    private CANSparkMax feedMotorR = new CANSparkMax(Constants.RIGHT_FEED_MOTOR_ID, MotorType.kBrushless);

    public Feeder() {
        feedMotorL.setInverted(false);
        feedMotorR.setInverted(true);
    }

    public void startFeeder() {
        setVoltage(Constants.FEEDER_FORWARD_VOLTAGE);
    }

    public void stopFeeder() {
        setVoltage(0.0);
    }

    public void reverseFeeder() {
        setVoltage(Constants.FEEDER_REVERSE_VOLTAGE);
    }

    private void setVoltage(double voltage) {

        feedMotorR.setVoltage(voltage);
    } // Add command methods here

    public Command startFeederCommand() {
        return new InstantCommand(this::startFeeder, this);

    }

    public Command stopFeederCommand() {
        return new InstantCommand(this::stopFeeder, this);
    }

    public Command reverFeederCommand() {
        return new InstantCommand(this::reverseFeeder, this);
    }

}
