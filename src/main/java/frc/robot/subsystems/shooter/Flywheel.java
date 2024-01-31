package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Flywheel extends SubsystemBase {

    private final TalonFX flywheelMotorL;
    private final TalonFX flywheelMotorR;
    private final StatusSignal<Double> flywheelLMotorVoltage, flywheelRMotorVoltage;
    private final StatusSignal<Double> flywheelLStatorCurrent, flywheelRStatorCurrent;
    private final StatusSignal<Double> flywheelLSupplyVoltage, flywheelRSupplyVoltage;
    private final StatusSignal<Double> flywheelLVelocity, flywheelRVelocity;

    public Flywheel() {
        flywheelMotorL = new TalonFX(Constants.FLYWHEEL_L_ID);
        flywheelMotorR = new TalonFX(Constants.FLYWHEEL_R_ID);
        flywheelMotorL.setInverted(true);
        flywheelMotorR.setInverted(false);
        flywheelMotorL.setNeutralMode(NeutralModeValue.Coast);
        flywheelMotorR.setNeutralMode(NeutralModeValue.Coast);
        // Define Loggable Fields
        flywheelLMotorVoltage = flywheelMotorL.getMotorVoltage();
        flywheelRMotorVoltage = flywheelMotorR.getMotorVoltage();
        flywheelLStatorCurrent = flywheelMotorL.getStatorCurrent();
        flywheelRStatorCurrent = flywheelMotorR.getStatorCurrent();
        flywheelLSupplyVoltage = flywheelMotorL.getSupplyVoltage();
        flywheelRSupplyVoltage = flywheelMotorR.getSupplyVoltage();
        flywheelLVelocity = flywheelMotorL.getVelocity();
        flywheelRVelocity = flywheelMotorR.getVelocity();
        BaseStatusSignal.setUpdateFrequencyForAll(Constants.ADVANTAGE_DEFAULT_LOG_FREQUENCY,
                flywheelLMotorVoltage,
                flywheelRMotorVoltage,
                flywheelLStatorCurrent,
                flywheelLStatorCurrent,
                flywheelRStatorCurrent,
                flywheelLSupplyVoltage,
                flywheelRSupplyVoltage,
                flywheelLVelocity,
                flywheelRVelocity);

    }

    public void startFlywheel(double voltage) {

        flywheelMotorL.setControl(new VoltageOut(voltage));
        flywheelMotorR.setControl(new VoltageOut(voltage));

    }

    public void stopFlywheel() {

        flywheelMotorL.stopMotor();
        flywheelMotorR.stopMotor();
    }

    public void reverseFlywheel() {
        startFlywheel(Constants.FLYWHEEL_REVERSE_VOLTAGE);
    }

    public Command startFlywheelCommand(double voltage) {
        return new InstantCommand(() -> startFlywheel(voltage), this);
    }

    public Command stopFlywheelCommand() {
        return new InstantCommand(this::stopFlywheel, this);
    }

    public Command reverseFlywheelCommand() {
        return new InstantCommand(this::reverseFlywheel, this);
    }

    @Override
    public void periodic() {

    }
}
