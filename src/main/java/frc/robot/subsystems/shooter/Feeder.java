package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
    CANSparkMax feedMotorL = new CANSparkMax(Constants.LEFT_FEED_MOTOR_ID, MotorType.kBrushless);
    CANSparkMax feedMotorR = new CANSparkMax(Constants.RIGHT_FEED_MOTOR_ID, MotorType.kBrushless);

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
        feedMotorL.setVoltage(voltage);
        feedMotorR.setVoltage(voltage);
    }
    // Add command methods here
}
