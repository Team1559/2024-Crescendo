package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Aimer extends SubsystemBase {
    @AutoLog
    static class AimerInputs {
        public double currentAngle;

        public double lAppliedOutput;
        public double lOutputCurrent;
        public double lMotorTemp;
        public int lFaults;
        public double lVelocity;

        public double rAppliedOutput;
        public double rOutputCurrent;
        public double rMotorTemp;
        public int rFaults;
        public double rVelocity;
    }

    private final CANSparkMax motorL;
    private final CANSparkMax motorR;
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.AIMER_ENCODER_PORT);
    private final PIDController controller = new PIDController(Constants.AIMER_KP, Constants.AIMER_KI,
            Constants.AIMER_KD);
    private final AimerInputsAutoLogged inputs = new AimerInputsAutoLogged();

    /**
     * Create a new subsystem for two motors controlled by CANspark Controller
     **/
    public Aimer() {
        motorL = new CANSparkMax(Constants.AIMER_L_ID, MotorType.kBrushless);
        motorR = new CANSparkMax(Constants.AIMER_R_ID, MotorType.kBrushless);
        motorL.setInverted(false);
        motorR.setInverted(true);
        motorL.setIdleMode(IdleMode.kBrake);
        motorR.setIdleMode(IdleMode.kBrake);
        motorL.setSmartCurrentLimit(Constants.NEO_SPARK_BRUSHLESS_CURRENT_LIMIT);
        motorR.setSmartCurrentLimit(Constants.NEO_SPARK_BRUSHLESS_CURRENT_LIMIT);
        motorL.setSecondaryCurrentLimit(Constants.NEO_SPARK_BRUSHLESS_CURRENT_SECONDARY_LIMIT);
        motorL.setSecondaryCurrentLimit(Constants.NEO_SPARK_BRUSHLESS_CURRENT_SECONDARY_LIMIT);
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Shooter/Aimer", inputs);
        double output = controller.calculate(inputs.currentAngle);
        motorL.setVoltage(output);
        motorR.setVoltage(output);
    }

    private void updateInputs() {
        inputs.currentAngle = getAngle();

        inputs.lAppliedOutput = motorL.getAppliedOutput();
        inputs.lOutputCurrent = motorL.getOutputCurrent();
        inputs.lMotorTemp = motorL.getMotorTemperature();
        inputs.lFaults = motorL.getFaults();
        inputs.lVelocity = motorL.getEncoder().getVelocity();

        inputs.rAppliedOutput = motorR.getAppliedOutput();
        inputs.rOutputCurrent = motorR.getOutputCurrent();
        inputs.rMotorTemp = motorR.getMotorTemperature();
        inputs.rFaults = motorR.getFaults();
        inputs.rVelocity = motorR.getEncoder().getVelocity();
    }

    public void setTargetAngle(double angle) {
        controller.setSetpoint(angle);
    }

    public double getAngle() {
        return encoder.getAbsolutePosition() * 360 - Constants.AIMER_ANGLE_OFFSET;
    }

}