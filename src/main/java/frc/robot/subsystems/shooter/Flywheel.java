package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import java.net.Socket;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase{

    private final TalonFX flywheelMotor1;
    private final TalonFX flywheelMotor2;
    private double speed = 0;

    public Flywheel(){
        flywheelMotor1 = new TalonFX(19);
        flywheelMotor2 = new TalonFX(17);
        flywheelMotor1.setInverted(true);
        flywheelMotor2.setInverted(false);
    }
    
    public void setFlywheelSpeed(double speed){
        this.speed = speed;
        flywheelMotor1.setControl(new DutyCycleOut(speed));
        flywheelMotor2.setControl(new DutyCycleOut(speed));
    }
    public void disableShooter(){
        speed = 0;
        flywheelMotor1.setControl(new DutyCycleOut(speed));
        flywheelMotor2.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void periodic(){
        
    }
}
