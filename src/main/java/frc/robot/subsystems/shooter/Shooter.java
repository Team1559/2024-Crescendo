package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

    private final TalonFX motor1;
    private final TalonFX motor2;
    private boolean isRunning = false;
    private double speed = 0;

    public Shooter(){
        motor1 = new TalonFX(19);
        motor2 = new TalonFX(17);
        motor1.setInverted(true);
        motor2.setInverted(false);
    }

    @Override
    public void periodic(){
        if(isRunning){
        motor1.setControl(new DutyCycleOut(speed));
        motor2.setControl(new DutyCycleOut(speed));
    }
    }
}
